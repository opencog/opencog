/** eval-candidate.cc --- 
 *
 * Copyright (C) 2013 OpenCog Foundation
 * Copyright (C) 2014 Aidyia Limited
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
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

/**
 * Evaluate the score of a candidate w.r.t. a fitness function. All
 * fitness functions available to moses are available (or at least
 * supposed to be).
 */

#include <boost/math/special_functions/binomial.hpp>
#include <boost/range/algorithm_ext/for_each.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>

#include <opencog/util/iostreamContainer.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/oc_omp.h>

#include <opencog/comboreduct/combo/iostream_combo.h>
#include <opencog/comboreduct/table/table_io.h>
#include <opencog/learning/moses/moses/types.h>
#include <opencog/learning/moses/moses/complexity.h>

#include "../scoring/discriminating_bscore.h"
#include "../scoring/behave_cscore.h"
#include "../scoring/bscores.h"
#include "../scoring/precision_bscore.h"

#include "eval-candidate.h"

using namespace std;
using namespace opencog;
using namespace moses;
using namespace combo;
using boost::str;
using boost::trim;
using boost::math::binomial_coefficient;

/**
 * Convert a string representing a combo program in a combo_tree.
 *
 * @param combo_prog_str   the string containing the combo program
 * @param labels           a vector of labels
 * @return                 the combo_tree
 */
combo_tree str2combo_tree_label(const std::string& combo_prog_str,
                                const std::vector<std::string>& labels)
{
    // combo pogram with place holders
    std::string combo_prog_ph_str = l2ph(combo_prog_str, labels);
    std::stringstream ss(combo_prog_ph_str);
    combo_tree tr;
    ss >> tr;
    return tr;
}

/**
 * Reverse of str2combo_tree_label
 *
 * @param combo_prog       combo tree of the program
 * @param labels           a vector of labels
 * @return                 string representing the combo tree with labels
 */
std::string combo_tree2str_label(const combo_tree& tr,
                                 const std::vector<std::string>& input_labels)
{
    // Stream combo tree into a string
    stringstream ss;
    ss << tr;

    // Replace the place holders by labels
    return ph2l(ss.str(), input_labels);
}

vector<vector<string>> get_all_combo_tree_str(const eval_candidate_params& ecp)
{
    vector<vector<string>> res;

    // From command line
    if (!ecp.combo_programs.empty())
        res.push_back(ecp.combo_programs);

    // From files
    for (const std::string& combo_prg_file : ecp.combo_program_files) {
        ifstream in(combo_prg_file);
        if (in) {
            res.emplace_back();
            while (in.good()) {
                string line;
                getline(in, line);
                if(line.empty())
                    continue;
                res.back().push_back(line);
            }
        } else {
            logger().error("Error: file %s can not be found.",
                           combo_prg_file.c_str());
            exit(1);
        }
    }

    return res;
}

std::ostream& ostream_scored_trees(std::ostream& out,
                                   const vector<string>& trs_str,
                                   const vector<composite_score>& css,
                                   const eval_candidate_params& ecp,
                                   const vector<string>& ilabels) {
    unsigned size = trs_str.size();
    OC_ASSERT(size == css.size(), "size=%u != css.size()=%u", size, css.size());
    for (unsigned i = 0; i < size; ++i) {
        // Stream out score
        out << css[i].get_score() << " ";

        // Stream out tree
        out << trs_str[i];

        out << std::endl;
    }
    return out;
}

int main(int argc, char** argv)
{
    namespace po = boost::program_options;
    eval_candidate_params ecp;
    unsigned long rand_seed;
    string log_level;
    static const string default_log_file("eval-candidate.log");
    string log_file;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")

        ("r,random-seed", po::value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")

        ("j,jobs",
         po::value<unsigned>(&ecp.jobs)->default_value(1),
         "Number of jobs allocated for evaluation.\n")

        // IO
        ("input-file,i", po::value<string>(&ecp.input_file),
         "DSV file containing inputs and outputs.\n")

        ("target-feature,u", po::value<string>(&ecp.target_feature_str),
         "Name of the target feature.\n")

        ("combo-program,y",
         po::value<vector<string>>(&ecp.combo_programs),
         "Combo program to evaluate. "
         "Can be used several times for several programs. "
         "Be careful to put it between single quotes, or escape the $ signs "
         "to prevent bash from applying variable substitution.\n")

        ("combo-program-file,C",
         po::value<vector<string>>(&ecp.combo_program_files),
         "File containing combo programs. "
         "Can be used several times for several files.\n")

        ("output-file,o", po::value<vector<string>>(&ecp.output_files),
         "File to write the results. If none is given it write on the stdout. "
         "If used multiple time, then the number must be equal "
         "to that of option -C and each output corresponds to the "
         "combo program file of the same order.\n")

        ("output-with-labels,W",
         po::value<bool>(&ecp.output_with_labels)->default_value(false),
         "If 1, output the candidates with argument labels "
         "instead of argument numbers. For instance "
         "*(\"$price\" \"$temperature\") instead of *($1 $2), "
         "where price and temperature are the first two features "
         "of the input table.\n")

        ("level,l", po::value<string>(&log_level)->default_value("INFO"),
         "Log level, possible levels are NONE, ERROR, WARN, INFO, "
         "DEBUG, FINE. Case does not matter.\n")

        ("log-file,f",
         po::value<string>(&log_file)->default_value(default_log_file),
         "File name where to write the log.\n")

        // Parameters
        ("problem,H", po::value<string>(&ecp.problem)->default_value(f_one),
         "Scorer to run. Supported scorers all erquire an input table. "
         "Supported scorers are:\n\n"
         "\trecall: show recall\n"
         "\tprerec: show precision\n"
         "\tf_one:  show F1-score (geometric mean of precision and recall)\n"
         "\tbep:    break-even point (difference between precision and recall)\n"
         "\tit:     accuracy scorer\n"
         "\tpre:    precision-activation scorer\n")

        ("alpha,Q",
         po::value<double>(&ecp.activation_pressure)->default_value(1.0),
         "pre scorer: Activation pressure.\n"
         "recall, prerec, bep scorers: Hardness.\n"
         "\nIf the score is not between the minimum and maximum, it is "
         "penalized with the pressure/hardness penalty.\n" )

        (",q",
         po::value<double>(&ecp.min_activation)->default_value(0.0),
         "pre scorer: Minimum activation.\n"
         "prerec scorer: Minimum recall.\n"
         "recall scorer: Minimum precision.\n"
         "bep    scorer: Minimum difference between precision and recall.\n"
         "\nIf the score is not between the minimum and mixiimum, it is "
         "penalized with the pressure/hardness penalty.\n" )

        (",w",
         po::value<double>(&ecp.max_activation)->default_value(1.0),
         "pre scorer: Maximum activation.\n"
         "prerec scorer: Maximum recall.\n"
         "recall scorer: Maximum precision.\n"
         "bep    scorer: Maximum difference between precision and recall.\n"
         "\nIf the score is not between the minimum and mixiimum, it is "
         "penalized with the pressure/hardness penalty.\n" )

        ("pre-positive",
         po::value<bool>(&ecp.pre_positive)->default_value(true),
         "For the 'pre' problem, if 1 then precision is maximized, "
         "if 0 then negative predictive value is maximized.\n")

        ;

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    }
    catch (po::error& e) {
        OC_ASSERT(false, "Fatal error: invalid or duplicated argument:\n\t%s\n",
                  e.what());
    }
    po::notify(vm);

    if (vm.count("help") || argc == 1) {
        cout << desc << endl;
        return 1;
    }

    // Set logger
    logger().setFilename(log_file);
    trim(log_level);
    Logger::Level level = logger().getLevelFromString(log_level);
    if (level != Logger::BAD_LEVEL)
        logger().setLevel(level);
    else {
        cerr << "Error: Log level " << log_level
             << " is incorrect (see --help)." << endl;
        exit(1);
    }
    logger().setBackTraceLevel(Logger::ERROR);

    // Set multi-threading
    setting_omp(ecp.jobs, 10);

    // Record original command line
    std::stringstream ss;
    for (int i=0; i<argc; i++) {
        ss << " " << argv[i];
    }
    logger().info() << "Command line:" << ss.str();

    // Init random generator
    randGen().seed(rand_seed);

    // Get all combo tree strings (from command line and file)
    vector<vector<string>> all_combo_tree_str = get_all_combo_tree_str(ecp);

    // Flatten all combo tree strings
    vector<string> flat_all_combo_tree_str;
    for (const auto& trs_str : all_combo_tree_str)
        flat_all_combo_tree_str.insert(flat_all_combo_tree_str.end(),
                                       trs_str.begin(), trs_str.end());

    // Read data ITable
    Table table = loadTable(ecp.input_file, ecp.target_feature_str);
    ITable& it = table.itable;

    // Parse combo programs
    vector<combo_tree> trs;
    for (const string& tr_str : flat_all_combo_tree_str) {
        combo_tree tr = str2combo_tree_label(tr_str, it.get_labels());
        if (logger().isDebugEnabled()) {
            logger().fine() << "Combo str: " << tr_str;
            logger().debug() << "Parsed combo: " << tr;
        }
        trs.push_back(tr);
    }

    // Define scorer
    bscore_base* bscore = nullptr;
    if ("recall" == ecp.problem) {
        bscore = new recall_bscore(table.compressed(),
            ecp.min_activation, ecp.max_activation, ecp.activation_pressure);
    }
    else if ("prerec" == ecp.problem) {
        bscore = new prerec_bscore(table.compressed(),
            ecp.min_activation, ecp.max_activation, ecp.activation_pressure);
    }
    else if ("bep" == ecp.problem) {
        bscore = new bep_bscore(table.compressed(),
            ecp.min_activation, ecp.max_activation, ecp.activation_pressure);
    }
    else if ("f_one" == ecp.problem) {
        bscore = new f_one_bscore(table.compressed());
    }
    else if ("it" == ecp.problem) {
        bscore = new ctruth_table_bscore(table.compressed());
    }
    else if ("pre" == ecp.problem) {
        bscore = new precision_bscore(table.compressed(),
                                      ecp.activation_pressure,
                                      ecp.min_activation,
                                      ecp.max_activation,
                                      ecp.pre_positive);
    }
    else {
        OC_ASSERT(false, "Unknown scorer type.");
    }

    behave_cscore bcscore(*bscore);

    // Evaluate the fitness score of each program
    vector<composite_score> css(trs.size());
    OMP_ALGO::transform(trs.begin(), trs.end(), css.begin(),
                        [&](const combo_tree& tr) {
                            return bcscore.get_cscore(tr);
                        });

    // Output the trees preceded by their scores
    if(ecp.output_files.empty())
    { // All on stdout
        ostream_scored_trees(cout, flat_all_combo_tree_str,
                             css, ecp, it.get_labels());
    } else if (ecp.output_files.size() == 1)
    { // All on single output_file
        ofstream of(ecp.output_files.front().c_str());
        ostream_scored_trees(of, flat_all_combo_tree_str,
                             css, ecp, it.get_labels());
    } else
    { // Results from combo programs of a file is written in its
      // corresponding output file (-o is used multiple times)
        unsigned ofiles_size = ecp.output_files.size(),
            from = 0, to = 0;
        OC_ASSERT(ofiles_size == ecp.combo_program_files.size(),
                  "ofiles_size=%u != ecp.combo_program_files.size()=%u",
                  ofiles_size, ecp.combo_program_files.size());
        OC_ASSERT(ofiles_size == all_combo_tree_str.size(),
                  "ofiles_size=%u != all_combo_tree_str.size()=%u",
                  ofiles_size, all_combo_tree_str.size());
        for (unsigned i = 0; i < ofiles_size; ++i) {
            to += all_combo_tree_str[i].size();
            vector<composite_score> css_chunk(css.begin() + from,
                                              css.begin() + to);
            ofstream of(ecp.output_files[i].c_str());
            ostream_scored_trees(of, all_combo_tree_str[i],
                                 css_chunk, ecp, it.get_labels());
            from = to;
        }
    }
}
