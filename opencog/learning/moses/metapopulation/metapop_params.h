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

#include "../moses/types.h"

namespace opencog { namespace moses {

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
 * parameters for metapopulation management
 */
struct metapop_parameters
{
    metapop_parameters(int _max_candidates = -1,
                       unsigned _revisit = 0,
                       score_t _complexity_temperature = 6.0f,
                       unsigned _jobs = 1,
                       diversity_parameters _diversity = diversity_parameters()) :
        max_candidates(_max_candidates),
        revisit(_revisit),
        do_boosting(false),
        keep_bscore(false),
        complexity_temperature(_complexity_temperature),
        cap_coef(50.0),
        jobs(_jobs),
        diversity(_diversity),
        merge_callback(NULL),
        callback_user_data(NULL)
        {}

    // The max number of candidates considered to be added to the
    // metapopulation, if negative then all candidates are considered.
    int max_candidates;

    // The number of times the same exemplar can be revisited. If 0
    // then an exemplar can only be visited once.
    unsigned revisit;

    // If true, then an Ada-Boost-style algorithm is applied during
    // the metapopulation merge. In this case, the scorer must be
    // capable of maintaining sample weights.
    bool do_boosting;

    // Keep track of the bscores even if not needed (in case the user
    // wants to keep them around, e.g. for debug printing.)
    bool keep_bscore;

    // Boltzmann temperature ...
    score_t complexity_temperature;

    // The metapopulation size is capped according to the following
    // formula:
    //
    // cap = cap_coef*(x+250)*(1+2*exp(-x/500))
    //
    // where x is the number of generations so far.  The goal of capping
    // is to keep the metapop small enough that it does not blow out the
    // available RAM on the machine, but large enough that deme expansion
    // can always find some suitable exemplar to explore.  The above
    // formula was arrived at via some ad-hoc experimentation.  A default
    // value of cap_coef=50 seems to work well.
    double cap_coef;

    // Number of jobs for metapopulation maintenance such as merging
    // candidates to the metapopulation.
    unsigned jobs;

    // parameters to control diversity
    diversity_parameters diversity;

    bool (*merge_callback)(scored_combo_tree_set&, void*);
    void *callback_user_data;
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_METAPOP_PARAMETERS_H
