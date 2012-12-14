/** eval-table.h ---
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


#ifndef _OPENCOG_EVAL_TABLE_H
#define _OPENCOG_EVAL_TABLE_H

#include <boost/assign/std/vector.hpp>

#include <opencog/util/algorithm.h>
#include <opencog/util/numeric.h>
#include <opencog/util/Logger.h>

#include "../table/table_io.h"
#include "../table/table.h"

using namespace std;
using namespace boost::assign;
using namespace opencog;
using namespace combo;

static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> input_table_opt("input-table", "i");
static const pair<string, string> target_feature_opt("target-feature", "u");
static const pair<string, string> ignore_feature_str_opt("ignore-feature", "Y");
static const pair<string, string> force_feature_opt("force-feature", "e");
static const pair<string, string> combo_str_opt("combo-program", "c");
static const pair<string, string> combo_prog_file_opt("combo-programs-file", "C");
static const pair<string, string> labels_opt("labels", "l");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> display_inputs_opt("display-inputs", "I");
static const pair<string, string> log_level_opt("log-level", "l");
static const pair<string, string> log_file_opt("log-file", "f");
static const string default_log_file_prefix = "eval-table";
static const string default_log_file_suffix = "log";
static const string default_log_file = default_log_file_prefix + "." + default_log_file_suffix;

string opt_desc_str(const pair<string, string>& opt) {
    return string(opt.first).append(",").append(opt.second);
}

combo_tree str2combo_tree_label(const std::string& combo_prog_str,
                                bool has_labels,
                                const std::vector<std::string>& labels);

// structure containing the options for the eval-table program
struct evalTableParameters
{
    string input_table_file;
    vector<string> combo_programs;
    string combo_programs_file;
    string target_feature_str;
    vector<string> ignore_features_str;
    vector<string> force_features_str;
    bool has_labels;
    vector<string> features;
    string features_file;
    bool display_inputs;
    string output_file;
    string log_level;
    string log_file;
};

template<typename Out>
Out& output_results(Out& out, const evalTableParameters& pa,
                    const Table& table, const OTable& ot_tr)
{
    Table eval_table = table;
    eval_table.otable = ot_tr;
    if (!pa.display_inputs)
        eval_table.itable = ITable();
    if (!pa.force_features_str.empty())
        eval_table.add_features_from_file(pa.input_table_file,
                                          pa.force_features_str);
    return ostreamTable(out, eval_table);
}

void output_results(const evalTableParameters& pa,
                    const Table& table, const OTable& ot_tr)
{
    if(pa.output_file.empty())
        output_results(cout, pa, table, ot_tr);
    else {
        ofstream of(pa.output_file.c_str());
        output_results(of, pa, table, ot_tr);
    }
}

void eval_output_results(const evalTableParameters& pa,
                         const Table& table, const vector<combo_tree>& trs)
{
    foreach(const combo_tree& tr, trs) {
        // evaluated tr over input table
        OTable ot_tr(tr, table.itable);
        if (!pa.target_feature_str.empty())
            ot_tr.set_label(pa.target_feature_str);
        // print results
        output_results(pa, table, ot_tr);
    }
}

/**
 * Get all combo tree strings, from the command line and the files
 */
vector<string> get_all_combo_tree_str(const evalTableParameters& pa)
{
    vector<string> res(pa.combo_programs);     // from command line
    
    // from a file
    if (!pa.combo_programs_file.empty()) {
        ifstream in(pa.combo_programs_file.c_str());
        while(in.good()) {
            string line;
            getline(in, line);
            if(line.empty())
                continue;
            res += line;
        }
    }

    return res;
}

void read_eval_output_results(evalTableParameters& pa)
{
    ostreamContainer(logger().info() << "Ignore the following features: ",
                     pa.ignore_features_str);
    OC_ASSERT(boost::find(pa.ignore_features_str, pa.target_feature_str)
              == pa.ignore_features_str.end(),
              "You cannot ignore the target feature %s",
              pa.target_feature_str.c_str());

    // get all combo tree strings (from command line and file)
    vector<string> all_combo_tree_str = get_all_combo_tree_str(pa);

    OC_ASSERT(all_combo_tree_str.size() == 1,
              "Using more than 1 combo, not implemented yet!");
        
    // parse all variables from all combo tree strings
    vector<string> all_variables;
    for (string combo_tree_str : all_combo_tree_str) {
        vector<string> vars = parse_combo_variables(combo_tree_str);
        all_variables.insert(all_variables.end(), vars.begin(), vars.end());
    }
    set<string> all_unique_variables(all_variables.begin(), all_variables.end());

    /// HERE WE ARE ASSUMING THAT THE INPUT FILE HAS A HEADER!!!
    vector<string> header = get_header(pa.input_table_file);

    // get (header - all_unique_variables - target feature)
    vector<string> ignore_variables;
    for (string f : header)
        if (f != pa.target_feature_str
            && all_unique_variables.find(f) == all_unique_variables.end())
            ignore_variables += f;
    
    // // read data ITable
    // Table table;
    // if (pa.target_feature_str.empty())
    //     table.itable = loadITable(pa.input_table_file, pa.ignore_features_str);
    // else {
    //     table = loadTable(pa.input_table_file, pa.target_feature_str,
    //                       pa.ignore_features_str);
    // }

    // read data ITable (using ignore_variables)
    Table table;
    if (pa.target_feature_str.empty())
        table.itable = loadITable_optimized(pa.input_table_file, ignore_variables);
    else {
        table = loadTable_optimized(pa.input_table_file, pa.target_feature_str,
                          ignore_variables);
    }
    
    ITable& it = table.itable;

    // read combo programs
    vector<combo_tree> trs;
    // from command line
    for (const string& tr_str : all_combo_tree_str)
        trs += str2combo_tree_label(tr_str, pa.has_labels, it.get_labels());

    // eval and output the results
    eval_output_results(pa, table, trs);
}

#endif // _OPENCOG_EVAL_TABLE_H
