/**
 * deme_optimize.h ---
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


#ifndef _OPENCOG_FEATURE_SELECTION_DEME_OPTIMIZE_H
#define _OPENCOG_FEATURE_SELECTION_DEME_OPTIMIZE_H

#include <boost/range/algorithm/sort.hpp>

#include <opencog/learning/moses/optimization/optimization.h>
#include <opencog/learning/moses/representation/field_set.h>
#include <opencog/learning/moses/representation/instance_set.h>
#include <opencog/learning/moses/moses/scoring.h>
#include <opencog/comboreduct/table/table.h>

#include "../main/feature-selection.h"

namespace opencog {

using namespace moses;
using namespace combo;

template<typename Optimize, typename Scorer>
feature_set optimize_deme_select_features(const field_set& fields,
                                          instance_set<composite_score>& deme,
                                          instance& init_inst,
                                          Optimize& optimize, const Scorer& scorer,
                                          const feature_selection_parameters& fs_params)
{
    // optimize feature set
    unsigned ae; // actual number of evaluations to reached the best candidate
    unsigned evals = optimize(deme, init_inst, scorer,
                             fs_params.hc_max_evals, fs_params.max_time,
                             &ae);

    // get the best one
    boost::sort(deme, std::greater<scored_instance<composite_score> >());
    instance best_inst = evals > 0 ? *deme.begin_instances() : init_inst;
    composite_score best_score =
        evals > 0 ? *deme.begin_scores() : worst_composite_score;

    // get the best feature set
    feature_set selected_features = get_feature_set(fields, best_inst);
    // Logger
    {
        // log its score
        stringstream ss;
        ss << "Selected feature set has composite score: ";
        if (evals > 0)
            ss << best_score;
        else
            ss << "Unknown";
        logger().info(ss.str());
    }
    {
        // Log the actual number of evaluations
        logger().info("Total number of evaluations performed: %u", evals);
        logger().info("Actual number of evaluations to reach the best feature set: %u", ae);
    }
    // ~Logger
    return selected_features;
}

// run feature selection given a moses optimizer and a scorer, create
// a deme a define the wrap the scorer for that deme. Possibly add
// cache as well.
template<typename Optimize, typename Scorer>
feature_set create_deme_select_features(const CTable& ctable,
                                        Optimize& optimize,
                                        const Scorer& scorer,
                                        const feature_selection_parameters& fs_params)
{
    arity_t arity = ctable.get_arity();
    field_set fields(field_set::disc_spec(2), arity);
    instance_set<composite_score> deme(fields);
    // determine the initial instance given the initial feature set
    instance init_inst = initial_instance(fs_params, fields, ctable.get_input_labels());
    // define moses based scorer
    typedef deme_based_scorer<Scorer> DBScorer;
    DBScorer db_sc(scorer, fields);
    // possibly wrap in a cache
    if(fs_params.hc_cache_size > 0) {
        // typedef prr_cache_threaded<DBScorer> ScorerCache;
        typedef iscorer_cache<DBScorer> ScorerCache;
        ScorerCache sc_cache(fs_params.hc_cache_size, db_sc);
        feature_set selected_features =
            optimize_deme_select_features(fields, deme, init_inst, optimize,
                                          sc_cache, fs_params);
        // Logger
        logger().info("Number of cache misses = %u", sc_cache.get_misses());
        // ~Logger
        return selected_features;
    } else {
        return optimize_deme_select_features(fields, deme, init_inst, optimize,
                                             db_sc, fs_params);
    }
}

// run feature selection given a moses optimizer
template<typename Optimize>
feature_set moses_select_features(const CTable& ctable,
                                  Optimize& optimize,
                                  const feature_selection_parameters& fs_params) {
    fs_scorer<set<arity_t> > fs_sc(ctable, fs_params);
    return create_deme_select_features(ctable, optimize, fs_sc, fs_params);
}

} // ~namespace opencog

#endif // _OPENCOG_FEATURE-SELECTION_DEME_OPTIMIZE_H
