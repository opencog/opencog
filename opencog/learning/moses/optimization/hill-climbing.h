/*
 * opencog/learning/moses/optimization/hill-climbing.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Predrag Janicic
 *            Nil Geisweiller
 *            Xiaohui Liu
 *            Linas Vepstas
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
#ifndef _MOSES_HILL_CLIMBING_H
#define _MOSES_HILL_CLIMBING_H

#include <opencog/util/oc_assert.h>

#include "../representation/instance_set.h"
#include "optimization.h"

namespace opencog { namespace moses {

/// Hill-climbing paramters
struct hc_parameters
{
    hc_parameters(bool widen = false,
                  bool step = false,
                  bool cross = true,
                  unsigned max_evals = 20000,
                  double _fraction_of_nn = 2.0) // > 1.0 since works on estimate only.
        : widen_search(widen),
          single_step(step),
          crossover(cross),
          crossover_pop_size(120),
          crossover_min_neighbors(400),
          max_nn_evals (max_evals),
          fraction_of_nn(_fraction_of_nn),
          score_range(5.0),
          max_allowed_instances(10000),
          resize_to_fit_ram(false)
    {
        OC_ASSERT(0.0 < fraction_of_nn);
    }

    bool widen_search;
    bool single_step;
    bool crossover;

    // Number of created instances by crossover
    unsigned crossover_pop_size;

    // Threshold to control when crossover is gonna be triggered. If
    // the number of neighbors to explore is above that threshold
    // (and at least 2 iterations have occured), the crossover kicks
    // in.
    unsigned crossover_min_neighbors;

    // Evaluate no more than this number of instances per iteration.
    // Problems with 100 or more features easily lead to exemplars with
    // complexity in excess of 100; these in turn, after decorating with
    // knobs, can have 20K or more nearest neighbors.  Doing a full
    // nearest neighbor scan for such cases will eat up a huge amount of
    // RAM in the instance_set, and its not currently obvious that a full
    // scan is that much better than a random sampling.  XXX Or is it?
    //
    // XXX This parameter should probably be automatically adjusted with
    // free RAM availability!?  Or something like that !?
    //
    // Nil: If it is adjusted with free RAM it should be able to be
    // turned off to keep as much determinism as possible
    unsigned max_nn_evals;

    // fraction_of_nn is the fraction of the nearest-neighbrohood (NN)
    // that should be searched.  For exhaustive NN search, this fraction
    // should be 1.0.  However, setting this lower seems to be OK,
    // especially when used with cross-over, since a fraction seems enough
    // to discover some decent instances that can be crossed-over.  And,
    // by not doing an exhaustive search, the run-time can be significantly
    // improved.
    //
    // XXX I don't understand what the below is saying.
    // One should probably try first to tweak pop_size_ratio to
    // control the allocation of resources. However in some cases (for
    // instance when hill_climbing is used for feature-selection),
    // there is only one deme to explore and tweaking that parameter
    // can make a difference (breadth vs depth)
    // XXX pop_size_ratio disabled in hill-climbing, since its definition
    // was insane/non-sensical.  I can't figure out how it was supposed
    // to work.
    double fraction_of_nn;

    // Range of scores for which to keep instances.  This *should* be
    // set to the value given by metapopulation::useful_score_range().
    // XXX TODO make sure this value is appropriately updated.
    //
    // The range of scores is used to keep the size of the deme in check.
    // The issue is that, for large feature sets, a large number of knobs
    // get created, which means that instances are huge.  It is easy to
    // end up with demes in the tens-of-gigabytes in size, and that's bad,
    // especially when most of the instances have terrible scores.
    score_t score_range;

    // Maximum allowed size of the deme.
    // This is used to keep the size of the deme in check.
    // The issue is that, for large feature sets, a large number of knobs
    // get created, which means that instances are huge.  It is easy to
    // end up with demes in the tens-of-gigabytes in size, and that's bad.
    size_t max_allowed_instances;

    // Flag to allow resizing the deme to keep memory usage under
    // control. Note, however, since it depends on the size of the
    // installed RAM, that two different runs of MOSES on two different
    // machines but otheriwse identical inputs and parameters, may not
    // behave identically to one-another.
    bool resize_to_fit_ram;

    // Deme stat name. String to indicate that TAB seperated deme
    // statistics are logged. By default 'Demes'.
    std::string prefix_stat_deme;
};

///////////////////
// Hill Climbing //
///////////////////

/**
 * Hill Climbing: search the local neighborhood of an instance for the
 * highest score, move to that spot, and repeat until no further
 * improvment is possible.
 *
 * This optimizatin algo performs an exhaustive search of the local
 * neighborhood centered upon a specific instance. The search is
 * normally limited to the neighborhood at Hamming distance=1 from the
 * central instance. The point with the best score (the "highest point")
 * is declared the new center, and the exhaustive search of the nearest
 * neighbors is repeated.  This loop is repeated until no further
 * improvement is possible.
 *
 * The number of nearest neighbors is roughly equal to the number of
 * knobs in the exemplar, and, more precisely, to the information-
 * theoretic bit length of the field set.
 *
 * The operation of this algorithm is modified in several important
 * ways. If the number of nearest neighbors exceeeds a significant
 * fraction of the max-allowed scoring function evaluations, then the
 * nearest neighborhood is sub-sampled, instead of being exhaustively
 * searched.
 *
 * If the single_step flag is set, then the search will terminate if
 * a higher-scoring instance is found.  Since the search is made only
 * to distance=1, using this flag is silly, unless the widen_search
 * flag is also used, which will cause the algo to search increasingly
 * larger neighborhoods.  In this case, the algo stops when improvement
 * is seen, or all neighborhoods up to the maximum distance have been
 * explored.
 *
 * In call cases, the neighborhood searched is 'spherical'; that is,
 * only the instances that are equi-distant from the exemplar are
 * explored (i.e. at the same Hamming distance).
 *
 * NB: most problems seem to do just fine without the single-step,
 * broaden-search flag combination.  However, some problems, esp. those
 * with a deceptive scoring function (e.g. polynomial factoring, -Hsr)
 * seem to work better with -L1 -T1 -I0.
 */
struct hill_climbing : optimizer_base
{
    hill_climbing(const optim_parameters& op = optim_parameters(),
                  const hc_parameters& hc = hc_parameters())
        : optimizer_base(op), hc_params(hc), _total_RAM_bytes(getTotalRAM())
    {}

protected:
    // log legend for graph stats
    void log_stats_legend();

    // Return an estimate of the size of the neighborhood at distance
    // 'distance'
    size_t estimate_neighborhood(size_t distance, const field_set& fields);

    // Return an estimate of the number of new instances to search
    size_t n_new_instances(size_t distance, unsigned max_evals,
                           size_t current_number_of_evals,
                           size_t total_number_of_neighbors);

    /**
     * Cross the single top-scoring instance against the next-highest scorers.
     *
     * As arguments, accepts a range of scored instances ("the sample"),
     * and a single instance from which these were all derived ("the base"
     * or center instance).  This will create a number of new instances,
     * which will be a cross of the highest-scoring instance with the
     * next-highest scoring instances.
     *
     * @deme:         the deme holding current instances, and where
     *                new instances will be placed.
     * @deme_size:    the current size of the deme. New instances
     *                will be appended at the end.
     * @base:         the base instance from which the sample was
     *                was derived.
     * @sample_start: the count, within the deme, at which the
     *                scored instances start. These are assumed to
     *                have been derived from the base instance.
     * @sample_size:  the number of instances in the sample. These
     *                are assumed to be in sequential order, starting
     *                at sample_start.
     * @num_to_make:  Number of new instances to create.  The actual
     *                number created will be the lesser of this and
     *                sample_size-1.
     */
    size_t cross_top_one(deme_t& deme,
                         size_t deme_size,
                         size_t num_to_make,
                         size_t sample_start,
                         size_t sample_size,
                         const instance& base);

    /** two-dimensional simplex version of above. */
    size_t cross_top_two(deme_t& deme,
                         size_t deme_size,
                         size_t num_to_make,
                         size_t sample_start,
                         size_t sample_size,
                         const instance& base);

    /** three-dimensional simplex version of above. */
    size_t cross_top_three(deme_t& deme,
                           size_t deme_size,
                           size_t num_to_make,
                           size_t sample_start,
                           size_t sample_size,
                           const instance& base);

    // chain the 3 crossovers methods above and return the number of new instances
    size_t crossover(deme_t& deme, size_t deme_size,
                     size_t sample_start, size_t sample_size,
                     const instance& base);

    bool resize_deme(deme_t& deme, score_t score_cutoff);
    size_t resize_by_score(deme_t& deme, score_t score_cutoff);

public:
    /**
     * Perform search of the local neighborhood of an instance.  The
     * search is exhaustive if the local neighborhood is small; else
     * the local neighborhood is randomly sampled.
     *
     * @param deme      Where to store the candidates searched. The deme
     *                  is assumed to be empty.  If it is not empty, it
     *                  will be overwritten.
     * @prama init_inst Start the seach from this instance.
     * @param iscorer   the Scoring function.
     * @param max_evals The maximum number of evaluations to perform.
     */
    void operator()(deme_t& deme,
                    const instance& init_inst,
                    const iscorer_base& iscorer,
                    unsigned max_evals,
                    time_t max_time);

    // Like above but assumes that init_inst is null (backward compatibility)
    // XXX In fact, all of the current code uses this entry point, no one
    // bothers to supply an initial instance.
    void operator()(deme_t& deme,
                    const iscorer_base& iscorer,
                    unsigned max_evals,
                    time_t max_time)
    {
        instance init_inst(deme.fields().packed_width());
        operator()(deme, init_inst, iscorer, max_evals, max_time);
    }

protected:
    const hc_parameters hc_params;
    const uint64_t _total_RAM_bytes;
    size_t _instance_bytes;
};


} // ~namespace moses
} // ~namespace opencog

#endif
