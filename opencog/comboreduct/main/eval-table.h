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

//ant_combo_vocabulary is used only for the boolean core vocabulary
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>
#include <opencog/comboreduct/combo/table.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/numeric.h>

#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace boost::assign;
using namespace ant_combo;

static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> input_table_opt("input-table", "i");
static const pair<string, string> combo_str_opt("combo-program", "c");
static const pair<string, string> combo_prog_file_opt("combo-programs-file", "C");
static const pair<string, string> labels_opt("labels", "l");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> feature_opt("feature", "f");
static const pair<string, string> features_file_opt("features-file", "F");
static const pair<string, string> compute_MI_opt("compute-MI", "m");
static const pair<string, string> display_output_table_opt("display-output-table", "d");

string opt_desc_str(const pair<string, string>& opt) {
    return string(opt.first).append(",").append(opt.second);
}

// structure containing the options for the eval-table program
struct evalTableParameters {
    string input_table_file;
    vector<string> combo_programs;
    string combo_programs_file;
    bool has_labels;
    vector<string> features;
    string features_file;
    bool display_output_table;
    string output_file;
};

template<typename Out, typename OT>
Out& output_results(Out& out, const evalTableParameters& pa, const OT& ot) {
    if(pa.display_output_table)
        out << ot << endl; // print output table
    return out;
}

template<typename OT>
void output_results(const evalTableParameters& pa, const OT& ot) {
    
    if(pa.output_file.empty())
        output_results(cout, pa, ot);
    else {
        ofstream of(pa.output_file.c_str(), ios_base::app);
        output_results(of, pa, ot);
        of.close();        
    }
}

template<typename IT, typename OT>
void eval_output_results(const evalTableParameters& pa,
                         const vector<combo_tree>& trs,
                         IT& it, const OT& ot, opencog::RandGen& rng) {
    foreach(const combo_tree& tr, trs) {
        // evaluated tr over input table
        it.set_consider_args(argument_set(tr)); // to speed up ot_tr computation
        OT ot_tr(tr, it, rng);
        
        ot_tr.set_label(ot.get_label());
        
        // print results
        output_results(pa, ot_tr);
    }
}

template<typename Table>
void read_eval_output_results(const evalTableParameters& pa,
                              opencog::RandGen& rng) {
    // read data table
    Table table(pa.input_table_file);

    // read combo programs
    vector<combo_tree> trs;
    foreach(const string& tr_str, pa.combo_programs)
        trs += str2combo_tree_label(tr_str, pa.has_labels,
                                    table.input.get_labels());
    if(!pa.combo_programs_file.empty()) {
        ifstream in(pa.combo_programs_file.c_str());
        while(in.good()) {
            string line;
            getline(in, line);
            if(line.empty())
                continue;
            trs += str2combo_tree_label(line, pa.has_labels,
                                        table.input.get_labels());
        }
    }
    // eval and output the results
    eval_output_results(pa, trs, table.input, table.output, rng);
}

#endif // _OPENCOG_EVAL_TABLE_H
