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

#include "ensemble_params.h"
#include "../moses/types.h"

namespace opencog { namespace moses {

struct diversity_parameters
{
    typedef score_t dp_t;

    diversity_parameters();

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
 * Parameters about subsample deme filter
 */
struct subsample_deme_filter_parameters
{
    subsample_deme_filter_parameters(unsigned _n_subsample_demes = 0,
                                     unsigned _n_top_candidates = 1) :
        by_time(true),
        contiguous_time(true),
        n_subsample_demes(_n_subsample_demes),
        n_top_candidates(_n_top_candidates),
        n_tuples(UINT_MAX) {}

    // Subsample by time
    bool by_time;

    // Contiguous time segment
    bool contiguous_time;

    // Number of subsample demes. If set 0 or 1 subsample_deme_filter
    // remains disactivated.
    unsigned n_subsample_demes;

    // Top candidates to consider for computing the score variance and
    // selecting the candidates from all subsamples
    unsigned n_top_candidates;

    // Number of tuples used to estimate the mean of the variance of
    // scores across the top candidates
    unsigned n_tuples;

    // Filter standard deviation threshold. A deme is only selected if
    // the average score standard deviation goes below std_dev_threshold
    float std_dev_threshold;

    // Filter mean, geometric mean and max tanimoto distance thresholds.
    float tanimoto_mean_threshold,
        tanimoto_geo_mean_threshold,
        tanimoto_max_threshold;

    // Instead of filtering accoring to threshold select the breadth first demes
    unsigned n_best_bfdemes;

    // Tanimoto mean, geometric mean and max weights used to calculate
    // the aggregate agreement distance for n_best_bfdemes
    float tanimoto_mean_weight,
        tanimoto_geo_mean_weight,
        tanimoto_max_weight;

    unsigned n_subsample_fitnesses;
    float low_dev_pressure;
};

/**
 * parameters about metapopulation management
 */
struct metapop_parameters
{
    metapop_parameters(int _max_candidates = -1,
                       int _revisit = 0,
                       score_t _complexity_temperature = 6.0f,
                       unsigned _jobs = 1,
                       diversity_parameters _diversity = diversity_parameters()) :
        max_candidates(_max_candidates),
        revisit(_revisit),
        do_boosting(false),
        discard_dominated(false),
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
    // then an exemplar can only be visited once. If negative then the
    // number of revisit is infinite
    int revisit;

    // If true, then an Ada-Boost-style algorithm is applied during
    // the metapopulation merge. In this case, the scorer must be
    // capable of maintaining sample weights.
    bool do_boosting;

    // Discard dominated candidates when merging candidates into
    // the metapopulation.  A candidate is "dominated" when some
    // existing member of the metapopulation scores better on
    // *every* sample in the scoring dataset. Naively, one might
    // think that a candidate that does worse, in every possible way,
    // is useless, and can be safely thrown away.  It turns out that
    // this is a bad assumption; dominated candidates, when selected
    // for deme expansion, often have far fitter off-spring than the
    // off-spring of the top-scoring (dominating) members of the
    // metapopulation. Thus, the "weak", dominated members of the
    // metapopulation are important for ensuring the vitality of
    // the metapopulation, and are discarded only at considerable risk.
    //
    // Note that the algorithms to compute domination are quite slow,
    // and so one has a double benefit: not only is the metapopulation
    // healthier, but candidate merging runs faster.
    bool discard_dominated;

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

    // parameters that control the ensemble
    ensemble_parameters ensemble_params;

    bool (*merge_callback)(scored_combo_tree_set&, void*);
    void *callback_user_data;
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_METAPOP_PARAMETERS_H
