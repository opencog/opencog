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
#include <opencog/comboreduct/table/table.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/numeric.h>

#include <boost/assign/std/vector.hpp>
#include <boost/range/algorithm/find.hpp>

#include "../scorers/mutual_info.h"

using namespace std;
using namespace boost::assign;
using namespace ant_combo;
using namespace opencog;

static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> scorer_opt("scorer", "H");
static const pair<string, string> input_file_opt("input-file", "i");
static const pair<string, string> target_feature_opt("target-feature", "u");
static const pair<string, string> ignore_feature_opt("ignore-feature", "Y");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> feature_opt("feature", "f");
static const pair<string, string> features_file_opt("features-file", "F");
static const pair<string, string> confidence_penalty_intensity_opt("confidence-penalty-intensity", "d");

/// @todo this could be in some common file
// Feature selection scorers
static const string mi="mi";    // Mutual Information (see feature_scorer.h)
static const string pre="pre";  // Precision (see
                                // opencog/learning/moses/scoring/scoring.h)

string opt_desc_str(const pair<string, string>& opt)
{
    return string(opt.first).append(",").append(opt.second);
}

// structure containing the options for the eval-table program
struct eval_features_parameters
{
    string input_file;
    string scorer;
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
    }
}

set<arity_t> get_features_idx(const vector<string>& features,
                              const vector<string>& labels,
                              const eval_features_parameters& pa) {
    set<arity_t> res;
    for (const string& f : features) {
        arity_t idx = distance(labels.begin(), boost::find(labels, f));
        OC_ASSERT((size_t)idx != labels.size(),
                  "No such a feature %s in file %s",
                  f.c_str(), pa.input_file.c_str());
        res.insert(idx);
    }
    return res;
}

vector<set<arity_t> > feature_sets(const eval_features_parameters& pa,
                                   const vector<string>& labels)
{
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
                         const vector<set<arity_t> >& feature_sets)
{
    // compute feature quality for each feature set
    vector<double> qs;
    for (const set<arity_t>& fs : feature_sets)
        qs += sc(fs);
    // print results
    output_results(pa, qs);
}

void eval_output_results(const eval_features_parameters& pa,
                         const vector<set<arity_t> > fss,
                         const ITable& it,
                         const OTable& ot)
{
    
    typedef MICScorer<set<arity_t> > FSScorer;

    FSScorer fs_sc(it, ot, pa.confidence_penalty_intensity);
    // compute and output the results
    eval_output_results(pa, fs_sc, fss);
}

void read_eval_output_results(const eval_features_parameters& pa)
{
    string target_feature, timestamp_feature;
    vector<string> ignore_features;

    Table table = loadTable(pa.input_file, target_feature,
                            timestamp_feature, ignore_features);


    // determine labels
    vector<string> labels = table.get_labels();

    // read feature sets
    vector<set<arity_t> > fss = feature_sets(pa, labels);

    // eval and output the results
    eval_output_results(pa, fss, table.itable, table.otable);
}

#endif // _OPENCOG_EVAL_FEATURES_H
