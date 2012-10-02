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

#include <opencog/comboreduct/combo/table.h>
#include <opencog/comboreduct/combo/table_io.h>

#include <opencog/learning/moses/optimization/optimization.h>
#include <opencog/learning/moses/optimization/hill-climbing.h>

#include "feature-selection.h"
#include "../feature_optimization.h"
#include "../feature_scorer.h"

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
    
/// Add back in any required features that were not selected.
///
/// 'selected_table' is the table of features that were selected.
///     (it is a filtered version of the full table, holding only the
///     selected features.)
/// 'full_table' is the original input table (with all features in it)
//
Table add_force_features(const Table& selected_table,
                         const Table& full_table,
                         const feature_selection_parameters& fs_params)
{
    // get selected features
    const vector<string>& fsel = selected_table.itable.get_labels();
    
    // get forced features that have not been selected
    vector<string> fnsel;
    foreach (const string& fn, fs_params.force_features_str)
        if (boost::find(fsel, fn) == fsel.cend())
            fnsel.push_back(fn);

    // If no feature to force, we are done.
    if (fnsel.empty())
        return selected_table;

    // header of the DSV file
    //
    // WARNING: by doing that we assume that the datafile is dense and
    // has a header
    vector<string> header = get_header(fs_params.input_file);

    // [0, header.size())
    vector<unsigned> header_pos = get_indices(header, header);
    // indices of selected features
    vector<unsigned> fsel_pos =  get_indices(fsel, header);
    // indices of forced non selected features
    vector<unsigned> fnsel_pos = get_indices(fnsel, header);
    
    // Get the complement: all features except the non selected forced
    // ones. We can't use the strings themselves because they are not
    // ordered and set_difference assumes they are.
    vector<unsigned> fnsel_pos_comp;
    boost::set_difference(header_pos, fnsel_pos, back_inserter(fnsel_pos_comp));
    
    // load the table with force_non_selected features with all
    // selected features with types string that way the content is
    // unchanged (convenient when the data contains stuff that
    // loadITable does not know how to interpret)
    //
    // I'm using istreamRawITable_ignore_indices because loading via
    // LoadITable is too slow, usually only a handful of features are
    // forced, deleting all the others takes time (several seconds in
    // my test case).
    ITable fns_table;
    ifstream in(fs_params.input_file.c_str());
    istreamRawITable_ingore_indices(in, fns_table, fnsel_pos_comp);

    // set the first row as header
    auto first_row_it = fns_table.begin();
    vector<string> fnsel_labels;
    foreach(const vertex& v, *first_row_it)
        fnsel_labels.push_back(boost::get<string>(v));
    fns_table.set_labels(fnsel_labels);
    fns_table.erase(first_row_it);

    // Insert the forced features in the right order. We want to keep
    // the features in order because that is likely what the user
    // expects.
    type_tree bogus;            // not implemented but maybe it
                                // doesn't matter
    Table new_table(selected_table.otable, selected_table.itable, bogus);

    // insert missing columns from fns_itable to new_table.itable
    for (auto lit = fnsel_pos.cbegin(), rit = fsel_pos.cbegin();
         lit != fnsel_pos.cend(); ++lit) {
        int lpos = distance(fnsel_pos.cbegin(), lit);
        vertex_seq cd = fns_table.get_column_data(lpos);
        string cl = fnsel_labels[lpos];
        while(rit != fsel_pos.cend() && *lit > *rit) ++rit;
        int rpos = rit != fsel_pos.cend() ?
            distance(fsel_pos.cbegin(), rit) + lpos : -1;
        new_table.itable.insert_col(cl, cd, rpos);
    }
    
    return new_table;
}

/**
 * After selecting the features the position of the target isn't
 * necessarily valid anymore so we must find it's correct relative
 * position.
 */
int update_target_feature_pos(const Table& table,
                              const feature_selection_parameters& fs_params)
{
    vector<string> header = get_header(fs_params.input_file);
    unsigned tfp = get_index(fs_params.target_feature_str, header);
    // Find the positions of the selected features
    vector<unsigned> fsel_pos = get_indices(table.itable.get_labels(), header);
    if (tfp < fsel_pos.front()) // it is first
        return 0;
    else if (tfp > fsel_pos.back()) // it is last
        return -1;
    else {                  // it is somewhere in between
        auto it = boost::adjacent_find(fsel_pos, [tfp](unsigned l, unsigned r) {
                return l < tfp && tfp < r; });
        return distance(fsel_pos.begin(), ++it);
    }
}

void write_results(const Table& selected_table,
                   const Table& full_table,
                   const feature_selection_parameters& fs_params)
{
    Table table_wff = add_force_features(selected_table, full_table, fs_params);
    int tfp = update_target_feature_pos(table_wff, fs_params);
    if (fs_params.output_file.empty())
        ostreamTable(cout, table_wff, tfp);
    else
        saveTable(fs_params.output_file, table_wff, tfp);
}

instance initial_instance(const feature_selection_parameters& fs_params,
                          const field_set& fields,
                          const vector<string>& labels)
{
    vector<string> vif; // valid initial features, used for logging
    instance res(fields.packed_width());

    foreach(const string& f, fs_params.hc_initial_features) {
        size_t idx = distance(labels.begin(), boost::find(labels, f));
        if(idx < labels.size()) { // feature found
            *(fields.begin_bit(res) + idx) = true;
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
    feature_set all_features(ir.begin(), ir.end());
    if (fs_params.target_size > 0) {
        fs_scorer<set<arity_t> > fs_sc(ctable, fs_params);
        return stochastic_max_dependency_selection(all_features, fs_sc,
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
        write_results(filt_table, table, fs_params);
    }
}

} // ~namespace opencog
