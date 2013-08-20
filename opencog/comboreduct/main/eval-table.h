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
static const pair<string, string> labels_opt("labels", "L");
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
    vector<string> combo_programs_files;
    string target_feature_str;
    vector<string> ignore_features_str;
    vector<string> force_features_str;
    bool has_labels;
    vector<string> features;
    string features_file;
    bool display_inputs;
    vector<string> output_files;
    bool split_output;
    string log_level;
    string log_file;
};

template<typename Out>
Out& output_results(Out& out, const evalTableParameters& pa,
                    const Table& table, const OTable& ot_tr)
{
    Table eval_table = table;
    eval_table.otable = ot_tr;
    if (!pa.display_inputs) {
        eval_table.itable = ITable();
        eval_table.target_pos = 0;
    }
    if (!pa.force_features_str.empty())
        eval_table.add_features_from_file(pa.input_table_file,
                                          pa.force_features_str);
    return ostreamTable(out, eval_table);
}

void output_results(const evalTableParameters& pa,
                    const Table& table, const OTable& ot_tr,
                    const string output_file = "");

void eval_output_results(const evalTableParameters& pa,
                         const Table& table, const vector<combo_tree>& trs);

/**
 * Get all combo tree strings, from the command line and the files
 */
vector<string> get_all_combo_tree_str(const evalTableParameters& pa);

void read_eval_output_results(evalTableParameters& pa);

#endif // _OPENCOG_EVAL_TABLE_H
