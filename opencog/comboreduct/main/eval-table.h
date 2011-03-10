/** eval-table.h --- 
 *
 * Copyright (C) 2011 Nil Geisweiller
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

// structure containing the options for the eval-table program
struct evalTableParameters {
    string input_table_file;
    vector<string> combo_programs;
    string combo_programs_file;
    bool has_labels;
    vector<string> features;
    string features_file;
    bool compute_MI;
    bool display_output_table;
    string output_file;
};

template<typename Out, typename OT>
Out& output_results(Out& out, const evalTableParameters& pa, const OT& ot,
                    double mi) {
    if(pa.compute_MI)
        out << mi << endl; // print mutual information
    if(pa.display_output_table)
        out << ot << endl; // print output table
    return out;
}

template<typename OT>
void output_results(const evalTableParameters& pa, const OT& ot,
                    const vector<double>& mis) {
    
    if(pa.output_file.empty())
        foreach(double mi, mis)
            output_results(cout, pa, ot, mi);
    else {
        ofstream of(pa.output_file.c_str(), ios_base::app);
        foreach(double mi, mis)
            output_results(of, pa, ot, mi);
        of.close();        
    }
}

set<arity_t> get_features_idx(const vector<string>& features,
                              const vector<string>& labels,
                              const evalTableParameters& pa) {
    set<arity_t> res;
    foreach(const string& f, features) {
        arity_t idx = distance(labels.begin(), find(labels, f));
        OC_ASSERT((size_t)idx != labels.size(),
                  "No such a feature %s in file %s",
                  f.c_str(), pa.input_table_file.c_str());
        res.insert(idx);
    }
    return res;
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
        
        // compute MI for each feature set
        vector<double> mis;
        if(pa.compute_MI) {
            vector<string> labels = read_data_file_labels(pa.input_table_file);
            if(!pa.features.empty()) {
                set<arity_t> fs = get_features_idx(pa.features, labels, pa);
                mis += mutualInformation(it, ot_tr, fs);
            }
            if(!pa.features_file.empty()) {
                ifstream in(pa.features_file.c_str());
                while(in.good()) {
                    string line;
                    getline(in, line);
                    if(line.empty())
                        continue;
                    set<arity_t> fs =
                        get_features_idx(tokenizeRowVec<string>(line),
                                         labels, pa);
                    mis += mutualInformation(it, ot_tr, fs);
                }
            }
        }
        // print results
        output_results(pa, ot_tr, mis);
    }
}

template<typename IT, typename OT, typename Type>
void read_eval_output_results(const evalTableParameters& pa,
                              opencog::RandGen& rng) {
    IT it;
    OT ot;

    // read data table
    istreamTable<IT, OT, Type>(pa.input_table_file, it, ot);

    // read combo programs
    vector<combo_tree> trs;
    foreach(const string& tr_str, pa.combo_programs)
        trs += str2combo_tree_label(tr_str, pa.has_labels, it.get_labels());
    if(!pa.combo_programs_file.empty()) {
        ifstream in(pa.combo_programs_file.c_str());
        while(in.good()) {
            string line;
            getline(in, line);
            if(line.empty())
                continue;
            trs += str2combo_tree_label(line, pa.has_labels, it.get_labels());
        }
    }
    // eval and output the results
    eval_output_results(pa, trs, it, ot, rng);
}

#endif // _OPENCOG_EVAL_TABLE_H
