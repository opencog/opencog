/** eval-table.cc ---
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <opencog/util/iostreamContainer.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/util/numeric.h>

#include "eval-table.h"

using namespace boost::program_options;
using boost::lexical_cast;
using boost::trim;
using namespace opencog;

/**
 * Program to output the result of a combo program given input data
 * described in DSV format.
 */
static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> input_table_opt("input-table", "i");
static const pair<string, string> target_feature_opt("target-feature", "u");
static const pair<string, string> ignore_feature_str_opt("ignore-feature", "Y");
static const pair<string, string> force_feature_opt("force-feature", "e");
static const pair<string, string> combo_str_opt("combo-program", "c");
static const pair<string, string> combo_prog_file_opt("combo-programs-file", "C");
static const pair<string, string> labels_opt("labels", "L");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> display_inputs_opt("display-inputs", "I");
static const pair<string, string> log_level_opt("log-level", "l");
static const pair<string, string> log_file_opt("log-file", "f");
static const string default_log_file_prefix = "eval-table";
static const string default_log_file_suffix = "log";
static const string default_log_file = default_log_file_prefix + "." + default_log_file_suffix;


/**
 * Convert a string representing a combo program in a combo_tree.
 *
 * @param combo_prog_str   the string containing the combo program
 * @param has_labels       true if the combo program has labels instead of
 * place holders (like and($start $up) instead of and($23 $134)
 * @param labels           a vector of labels
 * @return                 the combo_tree
 */
combo_tree str2combo_tree_label(const std::string& combo_prog_str,
                                bool has_labels,
                                const std::vector<std::string>& labels) {
    // combo pogram with place holders
    std::string combo_prog_ph_str =
        has_labels? l2ph(combo_prog_str, labels) : combo_prog_str;
    std::stringstream ss(combo_prog_ph_str);
    combo_tree tr;
    ss >> tr;
    return tr;
}

vector<string> get_all_combo_tree_str(const evalTableParameters& pa)
{
    vector<string> res(pa.combo_programs);     // from command line

    // from files
    for (const string& combo_programs_file : pa.combo_programs_files) {
        ifstream in(combo_programs_file.c_str());
        if (in) {
            while (in.good()) {
                string line;
                getline(in, line);
                if (line.empty())
                    continue;
                res += line;
            }
        } else {
            logger().error("Error: file %s can not be found.",
                           combo_programs_file.c_str());
            exit(1);
        }
    }

    return res;
}

void output_results(const evalTableParameters& pa,
                    const Table& table, const OTable& ot_tr,
                    const string output_file)
{
    if(output_file.empty())
        output_results(cout, pa, table, ot_tr);
    else {
        ofstream of(output_file.c_str());
        output_results(of, pa, table, ot_tr);
    }
}

void eval_output_results(const evalTableParameters& pa,
                         const Table& table, const vector<combo_tree>& trs)
{
    unsigned npad = ndigits(trs.size());
    OC_ASSERT(pa.output_files.size() == trs.size() || pa.output_files.size() <= 1);
    for (unsigned i = 0; i < trs.size(); i++) {
        // evaluated tr over input table
        OTable ot_tr(trs[i], table.itable);
        if (!pa.target_feature_str.empty())
            ot_tr.set_label(pa.target_feature_str);
        // determine output file name
        stringstream of_ss;
        if (pa.output_files.size() == 1) {
            of_ss << pa.output_files.front();
            if (trs.size() > 1 && pa.split_output)
                of_ss << setfill('0') << setw(npad) << i;
        }
        else if (pa.output_files.size() > 1)
            of_ss << pa.output_files[i];
        // print results
        output_results(pa, table, ot_tr, of_ss.str());
    }
}

void read_eval_output_results(evalTableParameters& pa)
{
    ostreamContainer(logger().info() << "Ignore the following features: ",
                     pa.ignore_features_str);
    OC_ASSERT(std::find(pa.ignore_features_str.begin(),
              pa.ignore_features_str.end(),
              pa.target_feature_str)
              == pa.ignore_features_str.end(),
              "You cannot ignore the target feature %s",
              pa.target_feature_str.c_str());

    // get all combo tree strings (from command line and file)
    vector<string> all_combo_tree_str = get_all_combo_tree_str(pa);

    // parse all variables from all combo tree strings
    vector<string> all_variables;
    for (string combo_tree_str : all_combo_tree_str) {
        vector<string> vars = parse_combo_variables(combo_tree_str);
        all_variables.insert(all_variables.end(), vars.begin(), vars.end());
    }
    set<string> all_unique_variables(all_variables.begin(), all_variables.end());

    // HERE WE ARE ASSUMING THAT THE INPUT FILE HAS A HEADER!!!
// XXX FIXME
    vector<string> header = get_header(pa.input_table_file);

    // get (header - all_unique_variables - target feature)
    vector<string> ignore_variables;
    for (string f : header)
        if (f != pa.target_feature_str
            && all_unique_variables.find(f) == all_unique_variables.end())
        {
            ignore_variables += f;
            logger().debug() << "Table variable not in combo tree: " << f;
        }

    // read data ITable (using ignore_variables)
    Table table;
    if (pa.target_feature_str.empty()) {
        OC_ASSERT(pa.timestamp_feature_str.empty(),
                  "Timestamp feature not implemented");
        table.itable = loadITable_optimized(pa.input_table_file,
                                            ignore_variables);
    }
    else {
        table = loadTable(pa.input_table_file, pa.target_feature_str,
                          pa.timestamp_feature_str, ignore_variables);
    }
    logger().debug() << "Done loading table from " << pa.input_table_file;

    ITable& it = table.itable;

    // parse combo programs
    vector<combo_tree> trs;
    for (const string& tr_str : all_combo_tree_str) {
        combo_tree tr = str2combo_tree_label(tr_str, pa.has_labels, it.get_labels());
        logger().fine() << "Combo str: " << tr_str << "\n";
        logger().debug() << "Parsed combo: " << tr;
        trs += tr;
    }

    // eval and output the results
    eval_output_results(pa, table, trs);
}

/**
 * Read and parse the eval-table program arguments.
 * Return the parsed results in the parameters struct.
 */
evalTableParameters eval_table_program_args(int argc, char** argv)
{
    // program options, see options_description below for their meaning
    evalTableParameters pa;
    unsigned long rand_seed;

    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")

        (opt_desc_str(rand_seed_opt).c_str(),
         value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")

        (opt_desc_str(input_table_opt).c_str(),
         value<string>(&pa.input_table_file),
         "Input table file in DSV format (seperators are comma, "
         "whitespace and tabulation).\n")

        (opt_desc_str(target_feature_opt).c_str(),
         value<string>(&pa.target_feature_str),
         "Target feature name. If empty (default) then no target feature "
         "is considered and the table is assumed to be all input data.\n")

        (opt_desc_str(ignore_feature_str_opt).c_str(),
         value<vector<string>>(&pa.ignore_features_str),
         "Ignore feature from the datasets. Can be used several times "
         "to ignore several features.\n")

        (opt_desc_str(force_feature_opt).c_str(),
         value<vector<string>>(&pa.force_features_str),
         "Force feature to be displayed (including ones which have been ignored). "
         "Can be used several times to force several features.\n")

        (opt_desc_str(combo_str_opt).c_str(),
         value<vector<string>>(&pa.combo_programs),
         "Combo program to evaluate against the input table. Note that in order "
         "to have variables not being interpreted as shell variables you may "
         "want to put the combi between single quotes. This option can be "
         "used several times so that several programs are evaluated at once.\n")

        (opt_desc_str(combo_prog_file_opt).c_str(),
         value<vector<string>>(&pa.combo_programs_files),
         "File containing combo programs to evaluate against the input table. "
         "Each combo program in the file is seperated by a new line and each "
         "results are displaied in the same order, seperated by a new line.\n")

        (opt_desc_str(labels_opt).c_str(),
         value<bool>(&pa.has_labels)->default_value(true),
         "If enabled then the combo program is expected to contain variables "
         "labels $labels1, etc, instead of place holders. For instance one "
         "provide the combo program 'and($large $tall)' instead of "
         "'and($24 $124)'. In such a case it is expected that the input data "
         "file contains the labels as first row. "
         "TODO could be detected automatically.\n")

        (opt_desc_str(output_file_opt).c_str(),
         value<vector<string>>(&pa.output_files),
         "File where to save the results. If empty then it outputs on "
         "the stdout. Can be used multiple times for multiple combo. "
         "In this case it overwrites --split-output and the number of "
         " output files must be identical to the number of combos.\n")

        ("split-output", value<bool>(&pa.split_output)->default_value(true),
         "If enabled, then if there are several combo programs the output file "
         "is used as prefix for writing multiple output files corresponding to "
         "each combo programs. In that case each output file name is appended "
         "a suffix of digits representing the index of the combo program "
         "(starting from 0). Suffixes are 0-padded to respect lexicographic "
         "order as well. If disabled, or if no output file is provided "
         "(stdout) all outputs are appended.\n")

        (opt_desc_str(display_inputs_opt).c_str(), value<bool>(&pa.display_inputs)->default_value(false),
         "Display all inputs (as well as the output and the forced features), "
         "the feature order is preserved.\n")

        (opt_desc_str(log_level_opt).c_str(),
         value<string>(&pa.log_level)->default_value("INFO"),
         "Log level, possible levels are NONE, ERROR, WARN, INFO, "
         "DEBUG, FINE. Case does not matter.\n")

        (opt_desc_str(log_file_opt).c_str(),
         value<string>(&pa.log_file)->default_value(default_log_file),
         "File name where to write the log.\n")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help") || argc == 1) {
        cout << desc << "\n";
        exit(1);
    }

    // Remove old log_file before setting the new one.
    remove(pa.log_file.c_str());
    logger().setFilename(pa.log_file);
    trim(pa.log_level);
    Logger::Level level = logger().getLevelFromString(pa.log_level);
    if (level != Logger::BAD_LEVEL)
        logger().setLevel(level);
    else {
        cerr << "Error: Log level " << pa.log_level << " is incorrect (see --help)." << endl;
        exit(1);
    }
    logger().setBackTraceLevel(Logger::ERROR);

    // init random generator
    randGen().seed(rand_seed);

    return pa;
}

/**
 * Program to evaluate a combo program over a data set repsented as csv file.
 */
int main(int argc, char** argv)
{
    evalTableParameters pa = eval_table_program_args(argc, argv);
    read_eval_output_results(pa);
    return 0;
}
