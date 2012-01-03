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

#include <opencog/comboreduct/combo/table.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/numeric.h>

#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace boost::assign;
using namespace opencog::combo;

static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> input_table_opt("input-table", "i");
static const pair<string, string> target_feature_opt("target-feature", "u");
static const pair<string, string> combo_str_opt("combo-program", "c");
static const pair<string, string> combo_prog_file_opt("combo-programs-file", "C");
static const pair<string, string> labels_opt("labels", "l");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> feature_opt("feature", "f");
static const pair<string, string> features_file_opt("features-file", "F");
static const pair<string, string> compute_MI_opt("compute-MI", "m");
static const pair<string, string> display_output_table_opt("display-output-table", "d");
static const pair<string, string> display_RMSE_opt("display-RMSE", "R");
static const pair<string, string> display_STD_opt("display-STD", "S");

string opt_desc_str(const pair<string, string>& opt) {
    return string(opt.first).append(",").append(opt.second);
}

combo_tree str2combo_tree_label(const std::string& combo_prog_str,
                                bool has_labels,
                                const std::vector<std::string>& labels);

// structure containing the options for the eval-table program
struct evalTableParameters {
    string input_table_file;
    vector<string> combo_programs;
    string combo_programs_file;
    string target_feature;
    bool has_labels;
    vector<string> features;
    string features_file;
    bool display_output_table;
    bool display_RMSE;
    bool display_STD;
    string output_file;
};

template<typename Out>
Out& output_results(Out& out, const evalTableParameters& pa,
                    const OTable& ot, const OTable& ot_tr) {
    if(pa.display_output_table)
        out << ot_tr << endl; // print output table
    if(pa.display_RMSE)
        out << "Root mean square error = "
            << ot.root_mean_square_error(ot_tr) << endl;
    if(pa.display_STD)
        out << "Standard deviation of the target feature = "
            << "TODO" << endl;
    return out;
}

void output_results(const evalTableParameters& pa,
                    const OTable& ot, const OTable& ot_tr) {
    if(pa.output_file.empty())
        output_results(cout, pa, ot, ot_tr);
    else {
        ofstream of(pa.output_file.c_str(), ios_base::app);
        output_results(of, pa, ot, ot_tr);
        of.close();        
    }
}

void eval_output_results(const evalTableParameters& pa,
                         const vector<combo_tree>& trs,
                         ITable& it, const OTable& ot, opencog::RandGen& rng) {
    foreach(const combo_tree& tr, trs) {
        // evaluated tr over input table
        OTable ot_tr(tr, it, rng);        
        ot_tr.set_label(ot.get_label());
        // print results
        output_results(pa, ot, ot_tr);
    }
}

void read_eval_output_results(const evalTableParameters& pa,
                              opencog::RandGen& rng) {
    // find the position of the target feature of the data file if any
    int target_pos = 0;
    if(!pa.target_feature.empty() && !pa.input_table_file.empty())
        target_pos = findTargetFeaturePosition(pa.input_table_file,
                                               pa.target_feature);

    // read data table
    Table table = istreamTable(pa.input_table_file, target_pos);

    // read combo programs
    vector<combo_tree> trs;
    foreach(const string& tr_str, pa.combo_programs)
        trs += str2combo_tree_label(tr_str, pa.has_labels,
                                    table.itable.get_labels());
    if(!pa.combo_programs_file.empty()) {
        ifstream in(pa.combo_programs_file.c_str());
        while(in.good()) {
            string line;
            getline(in, line);
            if(line.empty())
                continue;
            trs += str2combo_tree_label(line, pa.has_labels,
                                        table.itable.get_labels());
        }
    }
    // eval and output the results
    eval_output_results(pa, trs, table.itable, table.otable, rng);
}

#endif // _OPENCOG_EVAL_TABLE_H
