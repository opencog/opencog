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

#include <opencog/learning/moses/optimization/optimization.h>

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
    std::cerr << "No features have been selected." << std::endl;
    if (!outfile.empty()) {
        ofstream out(outfile.c_str());
        out << "# No features have been selected." << std::endl;
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

Table add_force_features(const Table& table,
                         const feature_selection_parameters& fs_params) {
    const ITable& itable = table.itable;
    // get forced features that have not been selected
    std::vector<std::string> fnsel;
    const auto& ilabels = itable.get_labels();
    foreach(const std::string& fn, fs_params.force_features_str)
        if (boost::find(ilabels, fn) == ilabels.cend())
            fnsel.push_back(fn);

    // get their positions
    std::vector<int> fnsel_pos =
        find_features_positions(fs_params.input_file, fnsel);
    boost::sort(fnsel_pos);

    // get the complementary of their positions
    std::vector<int> fnsel_pos_comp;
    auto ir = boost::irange(0, dataFileArity(fs_params.input_file) + 1);
    boost::set_difference(ir, fnsel_pos, back_inserter(fnsel_pos_comp));

    // get header of the input table
    auto header = loadHeader(fs_params.input_file);
    
    // load the table with force_non_selected features with all
    // selected features with types definite_object (i.e. string) that
    // way the content is unchanged (convenient when the data contains
    // stuff that loadITable does not know how to interpret)
    ITable fns_itable;          // ITable from fnsel (+ output)
    type_tree tt = gen_signature(id::definite_object_type, fnsel.size());
    loadITable(fs_params.input_file, fns_itable, tt, fnsel_pos_comp);

    // Find the positions of the selected features
    std::vector<int> fsel_pos =
        find_features_positions(fs_params.input_file, ilabels);

    // insert the forced features in the right order
    Table new_table;
    new_table.otable = table.otable;
    new_table.itable = itable;
    // insert missing columns from fns_itable to new_table.itable
    for (auto lit = fnsel_pos.cbegin(), rit = fsel_pos.cbegin();
         lit != fnsel_pos.cend(); ++lit) {
        int lpos = distance(fnsel_pos.cbegin(), lit);
        auto lc = fns_itable.get_col(lpos);
        while(rit != fsel_pos.cend() && *lit > *rit) ++rit;
        int rpos = rit != fsel_pos.cend() ?
            distance(fsel_pos.cbegin(), rit) + lpos : -1;
        new_table.itable.insert_col(lc.first, lc.second, rpos);
    }

    return new_table;
}

int update_target_feature(const Table& table,
                          const feature_selection_parameters& fs_params) {
    int tfp = fs_params.target_feature;
    if (tfp <= 0)               // it is either first or last
        return tfp;
    else {
        // Find the positions of the selected features
        std::vector<int> fsel_pos =
            find_features_positions(fs_params.input_file,
                                    table.itable.get_labels());
        if (tfp < fsel_pos.front()) // it is first
            return 0;
        else if (tfp > fsel_pos.back()) // it is last
            return -1;
        else {                  // it is somewhere in between
            auto it = boost::adjacent_find(fsel_pos, [tfp](int l, int r) {
                    return l < tfp && tfp < r; });
            return distance(fsel_pos.begin(), ++it);
        }
    }
}

void write_results(const Table& table,
                   const feature_selection_parameters& fs_params)
{
    Table table_wff = add_force_features(table, fs_params);
    int tfp = update_target_feature(table_wff, fs_params);
    if (fs_params.output_file.empty())
        ostreamTable(std::cout, table_wff, tfp);
    else
        saveTable(fs_params.output_file, table_wff, tfp);
}

instance initial_instance(const feature_selection_parameters& fs_params,
                          const field_set& fields) {
    instance res(fields.packed_width());
    vector<std::string> labels = readInputLabels(fs_params.input_file,
                                                 fs_params.target_feature,
                                                 fs_params.ignore_features);
    vector<std::string> vif; // valid initial features, used for logging
    foreach(const std::string& f, fs_params.hc_initial_features) {
        size_t idx = std::distance(labels.begin(), boost::find(labels, f));
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

feature_set max_mi_select_features(const CTable& ctable,
                                   const feature_selection_parameters& fs_params)
{
    auto ir = boost::irange(0, ctable.get_arity());
    feature_set all_features(ir.begin(), ir.end());
    if (fs_params.target_size > 0) {
        typedef MutualInformation<feature_set> FeatureScorer;
        FeatureScorer fsc(ctable);
        return max_mi_selection(all_features, fsc,
                                (unsigned) fs_params.target_size,
                                fs_params.threshold);
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
        op_params.hc_params.widen_search = true;
        op_params.hc_params.single_step = false;
        op_params.hc_params.crossover = false;
        hill_climbing hc(op_params);
        return moses_select_features(ctable, hc, fs_params);
    } else if (fs_params.algorithm == inc) {
        return incremental_select_features(ctable, fs_params);
    } else if (fs_params.algorithm == mmi) {
        return max_mi_select_features(ctable, fs_params);
    } else {
        std::cerr << "Fatal Error: Algorithm '" << fs_params.algorithm
                  << "' is unknown, please consult the help for the "
                     "list of algorithms." << std::endl;
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
        Table ftable = table.filtered(selected_features);
        log_selected_features(table.get_arity(), ftable, fs_params);
        write_results(ftable, fs_params);
    }
}

} // ~namespace opencog
