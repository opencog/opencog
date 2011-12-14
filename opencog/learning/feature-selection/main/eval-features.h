/** eval-features.h --- 
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


#ifndef _OPENCOG_EVAL_FEATURES_H
#define _OPENCOG_EVAL_FEATURES_H

#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>
#include <opencog/comboreduct/combo/table.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/numeric.h>

#include <boost/assign/std/vector.hpp>
#include <boost/range/algorithm/find.hpp>

#include "../feature_scorer.h"

using namespace std;
using namespace boost::assign;
using namespace ant_combo;
using namespace opencog;

static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> input_data_file_opt("input-file", "i");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> labels_opt("labels", "l");
static const pair<string, string> feature_opt("feature", "f");
static const pair<string, string> features_file_opt("features-file", "F");
static const pair<string, string> combo_str_opt("combo-program", "c");
static const pair<string, string> combo_prog_file_opt("combo-programs-file", "C");
static const pair<string, string> confidence_penalty_intensity_opt("confidence-penalty-intensity", "d");

string opt_desc_str(const pair<string, string>& opt) {
    return string(opt.first).append(",").append(opt.second);
}

combo_tree str2combo_tree_label(const std::string& combo_prog_str,
                                bool has_labels,
                                const std::vector<std::string>& labels);

// structure containing the options for the eval-table program
struct eval_features_parameters {
    string input_table_file;
    vector<string> combo_programs;
    string combo_programs_file;
    bool has_labels;
    vector<string> features;
    string features_file;
    string output_file;
    double confidence_penalty_intensity;
};

void output_results(const eval_features_parameters& pa,
                    const vector<double>& qs) {
    if(pa.output_file.empty())
        ostreamContainer(cout, qs, " ", "", "\n");
    else {
        ofstream of(pa.output_file.c_str(), ios_base::app);
        ostreamContainer(of, qs, " ", "", "\n");
        of.close();        
    }
}

set<arity_t> get_features_idx(const vector<string>& features,
                              const vector<string>& labels,
                              const eval_features_parameters& pa) {
    set<arity_t> res;
    foreach(const string& f, features) {
        arity_t idx = distance(labels.begin(), boost::find(labels, f));
        OC_ASSERT((size_t)idx != labels.size(),
                  "No such a feature %s in file %s",
                  f.c_str(), pa.input_table_file.c_str());
        res.insert(idx);
    }
    return res;
}

vector<set<arity_t> > feature_sets(const eval_features_parameters& pa,
                                   const vector<string>& labels) {
    vector<set<arity_t> > res;
    if(!pa.features.empty())
        res += get_features_idx(pa.features, labels, pa);
    if(!pa.features_file.empty()) {
        ifstream in(pa.features_file.c_str());
        while(in.good()) {
            string line;
            getline(in, line);
            if(line.empty())
                continue;
            res += get_features_idx(tokenizeRow<string>(line), labels, pa);
        }
    }
    return res;
}

template<typename Scorer>
void eval_output_results(const eval_features_parameters& pa,
                         const Scorer& sc,
                         const vector<set<arity_t> >& feature_sets) {
    // compute feature quality for each feature set
    vector<double> qs;
    foreach(const set<arity_t>& fs, feature_sets)
        qs += sc(fs);
    // print results
    output_results(pa, qs);
}

void eval_output_results(const eval_features_parameters& pa,
                         const vector<string>& labels,
                         const vector<set<arity_t> > fss,
                         const vector<combo_tree>& trs,
                         const ITable& it,
                         const OTable& ot,
                         opencog::RandGen& rng) {

    typedef MICScorer<set<arity_t> > FSScorer;

    if(trs.empty()) { // there is no combo programs so we use the data output
        FSScorer fs_sc(it, ot, pa.confidence_penalty_intensity);
        // compute and output the results
        eval_output_results(pa, fs_sc, fss);
    } else {
        foreach(const combo_tree& tr, trs) {
            // evaluated tr over input table
            OTable ot_tr(tr, it, rng);
            ot_tr.set_label(ot.get_label());

            FSScorer fs_sc(it, ot_tr, pa.confidence_penalty_intensity);
            
            // compute and output the results
            eval_output_results(pa, fs_sc, fss);
        }
    }
}

void read_eval_output_results(const eval_features_parameters& pa,
                              opencog::RandGen& rng) {
    Table table = istreamTable(pa.input_table_file);

    // determine labels
    vector<string> labels = readInputLabels(pa.input_table_file);

    // read feature sets
    vector<set<arity_t> > fss = feature_sets(pa, labels);

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
    eval_output_results(pa, labels, fss, trs, table.itable, table.otable, rng);
}

#endif // _OPENCOG_EVAL_FEATURES_H
