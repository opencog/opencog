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
#include <opencog/comboreduct/table/table.h>

#include "../main/feature-selection.h" // needed for feature_selection_params

namespace opencog {

using namespace moses;
using namespace combo;

/** For the MOSES algo, generate the intial instance */
instance initial_instance(const feature_selection_parameters& fs_params,
                          const field_set& fields,
                          const std::vector<std::string>& labels);

template<typename Optimize, typename Scorer>
feature_set_pop optimize_deme_select_feature_sets(const field_set& fields,
                                                  instance_set<composite_score>& deme,
                                                  instance& init_inst,
                                                  Optimize& optimize, const Scorer& scorer,
                                                  const feature_selection_parameters& fs_params)
{
    // optimize feature set
    optimize(deme, init_inst, scorer, fs_params.hc_max_evals, fs_params.max_time);

    // convert the deme into feature_set_pop (ignoring redundant sets)
    feature_set_pop fs_pop;
    for (const auto& inst : deme) {
        feature_set_pop::value_type p(select_tag()(inst).get_score(),
                                      get_feature_set(fields, inst));
        if (std::find(fs_pop.begin(), fs_pop.end(), p) == fs_pop.end())
            fs_pop.insert(p);
    }

    // Logger
    {
        // log its score
        std::stringstream ss;
        ss << "Selected feature set has composite score: ";
        if (deme.n_evals > 0)
            ss << fs_pop.begin()->first;
        else
            ss << "Unknown";
        logger().info(ss.str());
    }
    {
        // Log the actual number of evaluations
        logger().info("Total number of evaluations performed: %u", deme.n_evals);
        logger().info("Actual number of evaluations to reach the best feature set: %u",
                      deme.n_best_evals);
    }
    // ~Logger
    return fs_pop;
}

// run feature selection given a moses optimizer and a scorer, create
// a deme a define the wrap the scorer for that deme. Possibly add
// cache as well.
template<typename Optimize, typename Scorer>
feature_set_pop create_deme_select_feature_sets(const CTable& ctable,
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
        feature_set_pop sf_pop =
            optimize_deme_select_feature_sets(fields, deme, init_inst, optimize,
                                              sc_cache, fs_params);
        // Logger
        logger().info("Number of cache misses = %u", sc_cache.get_misses());
        // ~Logger
        return sf_pop;
    } else {
        return optimize_deme_select_feature_sets(fields, deme, init_inst, optimize,
                                                 db_sc, fs_params);
    }
}

// run feature selection given a moses optimizer
template<typename Optimize>
feature_set_pop moses_select_feature_sets(const CTable& ctable,
                                          Optimize& optimize,
                                          const feature_selection_parameters& fs_params)
{
    fs_scorer<std::set<arity_t>> fs_sc(ctable, fs_params);
    return create_deme_select_feature_sets(ctable, optimize, fs_sc, fs_params);
}

} // ~namespace opencog

#endif // _OPENCOG_FEATURE-SELECTION_DEME_OPTIMIZE_H
