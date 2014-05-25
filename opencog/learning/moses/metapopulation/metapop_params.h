/** metapop_params.h --- 
 *
 * Copyright (C) 2013 OpenCog Foundation
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


#ifndef _OPENCOG_METAPOP_PARAMETERS_H
#define _OPENCOG_METAPOP_PARAMETERS_H

#include "feature_selector.h"

namespace opencog { namespace moses {

static const operator_set empty_ignore_ops = operator_set();

struct diversity_parameters
{
    typedef score_t dp_t;

    diversity_parameters(bool _include_dominated = true);

    // Ignore behavioral score domination when merging candidates in
    // the metapopulation. Keeping dominated candidates may improves
    // performance by avoiding local maxima. Discarding dominated
    // candidates may increase the diversity of the metapopulation.
    bool include_dominated;

    // Diversity pressure of to enforce diversification of the
    // metapop. 0 means no diversity pressure, the higher the value
    // the strong the pressure.
    dp_t pressure;

    // exponent of the generalized mean (or sum, see below) used to
    // aggregate the diversity penalties of a candidate between a set
    // of candidates. If the exponent is negative (default) then the
    // max is used instead (generalized mean with infinite exponent).
    dp_t exponent;

    // If normalize is false then the aggregation of diversity
    // penalties is a generalize mean, otherwise it is a generalize
    // sum, defined here as
    //
    // generalized_mean(X) * |X|
    //
    // in other words it is a generalized mean without normalization
    // by the number of elements.
    //
    // If diversity_exponent is negative then this has no effect, it
    // is the max anyway.
    bool normalize;

    // There are 3 distances available to compare bscores, the p-norm,
    // the Tanimoto and the angular distances.
    enum dst_enum_t { p_norm, tanimoto, angular };
    void set_dst(dst_enum_t de, dp_t p = 0.0 /* optional distance parameter */);
    std::function<dp_t(const behavioral_score&, const behavioral_score&)> dst;

    // Function to convert the distance into diversity penalty. There
    // are 3 possible functions
    //
    // The inserve (offset by 1)
    //
    // f(x) = pressure / (1+x)
    //
    // The complement
    //
    // f(x) = pressure * (1-x)
    //
    // The pth-power
    //
    // f(x) = x^pressure
    //
    // The idea is that it should tend to pressure when the distance
    // tends to 0, and tends to 0 when the distance tends to its
    // maximum. Obviously the inverse is adequate when the maximum is
    // infinity and the complement is adequate when it is 1.  The
    // power is adequate when the diversity represent a probability
    // and that probability will later be multiplied by the score.
    enum dst2dp_enum_t { inverse, complement, pthpower };
    dst2dp_enum_t dst2dp_type;

    void set_dst2dp(dst2dp_enum_t d2de);
    std::function<dp_t(dp_t)> dst2dp;
};

/**
 * parameters about deme management
 */
struct metapop_parameters
{
    metapop_parameters(int _max_candidates = -1,
                       bool _reduce_all = true,
                       unsigned _revisit = 0,
                       score_t _complexity_temperature = 3.0f,
                       const operator_set& _ignore_ops = empty_ignore_ops,
                       // bool _enable_cache = false,    // adaptive_cache
                       unsigned _cache_size = 100000,     // is disabled
                       unsigned _jobs = 1,
                       diversity_parameters _diversity = diversity_parameters(),
                       const combo_tree_ns_set* _perceptions = NULL,
                       const combo_tree_ns_set* _actions = NULL,
                       const feature_selector* _fstor = NULL) :
        max_candidates(_max_candidates),
        reduce_all(_reduce_all),
        revisit(_revisit),
        keep_bscore(false),
        complexity_temperature(_complexity_temperature),
        cap_coef(50),
        ignore_ops(_ignore_ops),
        // enable_cache(_enable_cache),   // adaptive_cache
        cache_size(_cache_size),          // is disabled
        jobs(_jobs),
        diversity(_diversity),
        perceptions(_perceptions),
        actions(_actions),
        merge_callback(NULL),
        callback_user_data(NULL),
        fstor(_fstor),
        linear_contin(true)
        {}

    // The max number of candidates considered to be added to the
    // metapopulation, if negative then all candidates are considered.
    int max_candidates;

    // If true then all candidates are reduced before evaluation.
    bool reduce_all;

    // The number of times the same exemplar can be revisited. If 0
    // then an exemplar can only be visited once.
    unsigned revisit;

    // keep track of the bscores even if not needed (in case the user
    // wants to keep them around)
    bool keep_bscore;

    // Boltzmann temperature ...
    score_t complexity_temperature;

    // The metapopulation size is capped according to the following
    // formula:
    //
    // cap = cap_coef*(x+250)*(1+2*exp(-x/500))
    //
    // where x is the number of generations so far
    double cap_coef;

    // the set of operators to ignore
    operator_set ignore_ops;

    // Enable caching of scores.
    // bool enable_cache;   // adaptive_cache
    unsigned cache_size;    // is disabled

    // Number of jobs for metapopulation maintenance such as merging
    // candidates to the metapopulation.
    unsigned jobs;

    // parameters to control diversity
    diversity_parameters diversity;

    // the set of perceptions of an optional interactive agent
    const combo_tree_ns_set* perceptions;
    // the set of actions of an optional interactive agent
    const combo_tree_ns_set* actions;

    bool (*merge_callback)(scored_combo_tree_set&, void*);
    void *callback_user_data;

    const feature_selector* fstor;

    // Build only linear expressions involving contin features.
    // This can greatly decrease the number of knobs created during
    // representation building, resulting in much smaller field sets,
    // and instances that can be searched more quickly. However, in
    // order to fit the data, linear expressions may not be as good,
    // and thus may require more time overall to find...
    bool linear_contin;

    // Defines how many pairs of literals constituting subtrees op(l1
    // l2) are considered while creating the prototype of an
    // exemplar. It ranges from 0 to 1, 0 means arity positive
    // literals and arity pairs of literals, 1 means arity positive
    // literals and arity*(arity-1) pairs of literals
    float perm_ratio;
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_METAPOP_PARAMETERS_H
