/** feature-selection.cc ---
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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

#include <iostream>
#include <fstream>
#include <memory>
#include <stdio.h>

#include <boost/assign/std/vector.hpp> // for 'operator+=()'
#include <boost/range/algorithm/find.hpp>
#include <boost/range/algorithm/adjacent_find.hpp>
#include <boost/range/algorithm/set_algorithm.hpp>
#include <boost/range/irange.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>
#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/iostreamContainer.h>
#include <opencog/util/oc_omp.h>

#include <opencog/comboreduct/table/table.h>
#include <opencog/comboreduct/table/table_io.h>

#include <opencog/learning/moses/optimization/optimization.h>
#include <opencog/learning/moses/optimization/hill-climbing.h>

#include "feature-selection.h"
#include "../algo/deme_optimize.h"
#include "../algo/stochastic_max_dependency.h"
#include "../algo/incremental.h"

namespace opencog {
    
using namespace combo;
using namespace std;
using namespace boost::assign; // bring 'operator+=()' into scope

void err_empty_features(const string& outfile)
{
    // If the dataset is degenerate, it can happen that feature
    // selection will fail to find any features.  Rather than just
    // crashing as a result, write out an empty file, which at least
    // allows downstream processes to infer what happened (i.e. to tell
    // apart this case from an outright crash.)
    logger().warn() << "No features have been selected.";
    cerr << "No features have been selected." << endl;
    if (!outfile.empty()) {
        ofstream out(outfile.c_str());
        out << "# No features have been selected." << endl;
    }
    exit(0);
}

// Score all individual features of Table
vector<double> score_individual_features(const Table& table,
                                         const feature_selection_parameters& fs_params)
{
    typedef set<arity_t> FS;
    CTable ctable = table.compressed();
    fs_scorer<FS> fs_sc(ctable, fs_params);
    vector<double> res;
    boost::transform(boost::irange(0, table.get_arity()), back_inserter(res),
                     [&](arity_t idx) { FS fs = {idx}; return fs_sc(fs); });
    return res;
}

// log the set of features and its number
void log_selected_features(arity_t old_arity, const Table& ftable,
                           const feature_selection_parameters& fs_params)
{
    // log the number selected features
    logger().info("%d out of %d have been selected",
                  ftable.get_arity(), old_arity);
    // log set of selected feature set
    {
        stringstream ss;
        ss << "The following features have been selected: ";
        ostreamContainer(ss, ftable.itable.get_labels(), ",");
        logger().info(ss.str());
    }
    // log the score of each feature individually
    {
        stringstream ss;
        ss << "With the following scores (individually): ";
        ostreamContainer(ss, score_individual_features(ftable, fs_params), ",");
        logger().info(ss.str());            
    }
}

/**
 * Get indices (aka positions or offsets) of a list of labels given a header
 */
vector<unsigned> get_indices(const vector<string>& labels,
                             const vector<string>& header) {
    vector<unsigned> res;
    for (unsigned i = 0; i < header.size(); ++i)
        if (boost::find(labels, header[i]) != labels.end())
            res.push_back(i);
    return res;
}
unsigned get_index(const string& label, const vector<string>& header) {
    return distance(header.begin(), boost::find(header, label));
}

void write_results(const Table& selected_table,
                   const feature_selection_parameters& fs_params)
{
    Table table_wff = selected_table;
    table_wff.add_features_from_file(fs_params.input_file,
                                     fs_params.force_features_str);
    if (fs_params.output_file.empty())
        ostreamTable(cout, table_wff);
    else
        saveTable(fs_params.output_file, table_wff);
}

feature_set initial_features(const vector<string>& ilabels,
                             const feature_selection_parameters& fs_params)
{
    vector<string> vif; // valid initial features, used for logging
    feature_set res;

    for (const string& f : fs_params.initial_features) {
        size_t idx = distance(ilabels.begin(), boost::find(ilabels, f));
        if(idx < ilabels.size()) { // feature found
            res.insert(idx);
            // for logging
            vif += f;
        }
        else // feature not found
            logger().warn("No such a feature #%s in file %s. It will be ignored as initial feature.", f.c_str(), fs_params.input_file.c_str());
    }
    // Logger
    if(vif.empty())
        logger().info("The search will start with the empty feature set");
    else {
        stringstream ss;
        ss << "The search will start with the following feature set: ";
        ostreamContainer(ss, vif, ",");
        logger().info(ss.str());
    }
    // ~Logger
    return res;
}

feature_set incremental_select_features(const CTable& ctable,
                                        const feature_selection_parameters& fs_params)
{
    auto ir = boost::irange(0, ctable.get_arity());
    feature_set all_features(ir.begin(), ir.end());
    if (fs_params.threshold > 0 || fs_params.target_size > 0) {
        typedef MutualInformation<feature_set> FeatureScorer;
        FeatureScorer fsc(ctable);
        return fs_params.target_size > 0?
            cached_adaptive_incremental_selection(all_features, fsc,
                                                  fs_params.target_size,
                                                  fs_params.inc_interaction_terms,
                                                  fs_params.inc_red_intensity,
                                                  0, 1,
                                                  fs_params.inc_target_size_epsilon)
            : cached_incremental_selection(all_features, fsc,
                                           fs_params.threshold,
                                           fs_params.inc_interaction_terms,
                                           fs_params.inc_red_intensity);
    } else {
        // Nothing happened, return all features by default
        return all_features;
    }
}

feature_set smd_select_features(const CTable& ctable,
                                const feature_selection_parameters& fs_params)
{
    auto ir = boost::irange(0, ctable.get_arity());
    feature_set all_features(ir.begin(), ir.end()),
        init_features = initial_features(ctable.get_input_labels(), fs_params);
    if (fs_params.target_size > 0) {
        fs_scorer<set<arity_t> > fs_sc(ctable, fs_params);
        return stochastic_max_dependency_selection(all_features, init_features,
                                                   fs_sc,
                                                   (unsigned) fs_params.target_size,
                                                   fs_params.threshold,
                                                   fs_params.smd_top_size);
    } else {
        // Nothing happened, return the all features by default
        return all_features;
    }
}

feature_set select_features(const CTable& ctable,
                            const feature_selection_parameters& fs_params) {
    if (fs_params.algorithm == moses::hc) {
        // setting moses optimization parameters
        double pop_size_ratio = 20;
        size_t max_dist = 4;
        score_t min_score_improv = 0.0;
        optim_parameters op_params;
        op_params.opt_algo = moses::hc;
        op_params.pop_size_ratio = pop_size_ratio;
        op_params.terminate_if_gte = fs_params.hc_max_score;
        op_params.max_dist = max_dist;
        op_params.set_min_score_improv(min_score_improv);
        hc_parameters hc_params;
        hc_params.widen_search = true;
        hc_params.single_step = false;
        hc_params.crossover = false;
        hill_climbing hc(op_params, hc_params);
        return moses_select_features(ctable, hc, fs_params);
    } else if (fs_params.algorithm == inc) {
        return incremental_select_features(ctable, fs_params);
    } else if (fs_params.algorithm == smd) {
        return smd_select_features(ctable, fs_params);
    } else {
        cerr << "Fatal Error: Algorithm '" << fs_params.algorithm
             << "' is unknown, please consult the help for the "
             << "list of algorithms." << endl;
        exit(1);
        return feature_set(); // to please Mr compiler
    }
}

feature_set select_features(const Table& table,
                            const feature_selection_parameters& fs_params) {
    CTable ctable = table.compressed();
    return select_features(ctable, fs_params);
}

void feature_selection(const Table& table,
                       const feature_selection_parameters& fs_params)
{
    feature_set selected_features = select_features(table, fs_params);
    if (selected_features.empty())
        err_empty_features(fs_params.output_file);
    else {
        Table filt_table = table.filtered(selected_features);
        log_selected_features(table.get_arity(), filt_table, fs_params);
        write_results(filt_table, fs_params);
    }
}

} // ~namespace opencog
