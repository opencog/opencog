/*
 * opencog/learning/moses/optimization/hill-climbing.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings LLC
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

#include <math.h>   // for sqrtf, cbrtf

#include <boost/algorithm/minmax_element.hpp>

#include <opencog/util/oc_omp.h>

#include "../moses/neighborhood_sampling.h"

#include "hill-climbing.h"

namespace opencog { namespace moses {


///////////////////
// Hill Climbing //
///////////////////

void hill_climbing::operator()(deme_t& deme,
                               const instance& init_inst,
                               const iscorer_base& iscorer,
                               unsigned max_evals,
                               time_t max_time)
{
    logger().debug("Local Search Optimization");

    log_stats_legend();

    // Collect statistics about the run, in struct optim_stats
    nsteps = 0;
    demeID = deme.getID();
    over_budget = false;
    struct timeval start;
    gettimeofday(&start, NULL);

    const field_set& fields = deme.fields();

    // Track RAM usage. Instances can chew up boat-loads of RAM.
    _instance_bytes = sizeof(instance)
        + sizeof(packed_t) * fields.packed_width();

    size_t current_number_of_evals = 0;
    size_t current_number_of_instances = 0;

    const unsigned max_distance = opt_params.max_distance(fields);

    // center_inst is the current location on the hill.
    // Copy it, don't reference it, since sorting will mess up a ref.
    instance center_inst(init_inst);
    composite_score best_cscore = worst_composite_score;
    score_t best_score = very_worst_score;
    score_t best_raw_score = very_worst_score;

    // Initial distance is zero, so that the first time through
    // the loop, we handle just one instance, the initial instance.
    // (which is at distance zero from itself, of course).
    unsigned distance = 0;
    unsigned iteration = 0;

    // Needed for correlated neighborhood exploration.
    instance prev_center = center_inst;
    size_t prev_start = 0;
    size_t prev_size = 0;
    
    // keep track whether the population generated from the same
    // center has already been crossover. This is to prevent to redo a
    // useless crossover when distance widening is occurs
    bool already_xover = false;
    
    // try a last crossover if no improvements (it's cheap)
    bool last_chance = false;

    // Whether the score has improved during an iteration
    while (true)
    {
        ++iteration;
        logger().debug("Iteration: %u", iteration);

        // Estimate the number of neighbors at the distance d.
        // This is faster than actually counting.
        size_t total_number_of_neighbors =
            estimate_neighborhood(distance, fields);

        // Number of instances to try, this go-around.
        size_t number_of_new_instances =
            n_new_instances(distance, max_evals, current_number_of_instances,
                            total_number_of_neighbors);

        // The first few times through, (or, if we decided on a full
        // rescan), explore the entire nearest neighborhood.
        // Otherwise, make the optimistic assumption that the best new
        // instances are likely to be genetic cross-overs of the
        // current top-scoring instances.  This assumption seems to
        // work really quite well.
        //
        // Based on experiments, crossing over the 40 highest-scoring
        // instances seems to offer plenty of luck, so
        // crossover_pop_size is defined to be 120 by default (40 for
        // cross_top_one, 40 for cross_top_two, 40 for
        // cross_top_three).
        //
        // If the size of the nearest neighborhood is small enough,
        // then don't bother with the optimistic guessing.  The
        // cross-over guessing will generate crossover_pop_size
        // instances.  Our guestimate for number of neighbors
        // intentionally over-estimates by a factor of two. So the
        // breakeven point would be 2*crossover_pop_size, and we pad this
        // a bit, as small exhaustive searches do beat guessing...
        size_t xover_min_neighbors = hc_params.crossover_min_neighbors;
        //
        // current_number_of_instances can drop as low as 1 if the
        // population trimmer wiped out everything, which can happen.
        // Anyway, cross-over generates nothing when its that small.
        size_t xover_min_deme = 3;

        // whether crossover must be attempted for the current iteration
        bool large_nbh = total_number_of_neighbors >= xover_min_neighbors,
            xover = hc_params.crossover
            && (iteration > 2)
            && !already_xover
            && current_number_of_instances >= xover_min_deme
            && (large_nbh || last_chance);

        if (xover) {
            // log why crossover is enabled
            std::stringstream why_xover;
            if (large_nbh)
                why_xover << "too large neighborhood "
                          << total_number_of_neighbors << ">="
                          << xover_min_neighbors;
            else if (last_chance)
                why_xover << "last chance";
            else
                OC_ASSERT(false, "There must be a bug");
            logger().debug() << "Crossover enabled ("
                             << why_xover.str() << ")";

            number_of_new_instances =
                crossover(deme, current_number_of_instances,
                          prev_start, prev_size, prev_center);

            already_xover = true;
        } else {
            // log why crossover is disabled
            std::stringstream whynot_xover;
            if (!hc_params.crossover)
                whynot_xover << "user option";
            else if (iteration <= 2)
                whynot_xover << "first 2 iterations";
            else if (already_xover)
                whynot_xover << "already done from that center";
            else if (current_number_of_instances < xover_min_deme)
                whynot_xover << "not enough instances to cross";
            else if (!large_nbh)
                whynot_xover << "small enough neighborhood for full search";
            else
                OC_ASSERT(false, "There must be a bug");
            logger().debug() << "Crossover disabled ("
                             << whynot_xover.str() << ")";
            
            // The current_number_of_instances arg is needed only to
            // be able to manage the size of the deme appropriately.
            number_of_new_instances =
                sample_new_instances(total_number_of_neighbors,
                                     number_of_new_instances,
                                     current_number_of_instances,
                                     center_inst, deme, distance);
        }
        prev_start = current_number_of_instances;
        prev_size = number_of_new_instances;
        prev_center = center_inst;

        auto deme_from = next(deme.begin(), current_number_of_instances);
        auto deme_inst_from = next(deme.begin_instances(),
                                   current_number_of_instances);
        auto deme_score_from = next(deme.begin_scores(),
                                    current_number_of_instances);

        // log neighborhood distance
        if (logger().isDebugEnabled()) {
            std::stringstream nbh_dst;
            nbh_dst << "Evaluate " << number_of_new_instances << " neighbors";
            if (number_of_new_instances > 0) {
                if (xover) {
                    // compute the min and max hamming distances between the
                    // center instance and the crossed-over instances
                    auto dst_from_center = [&](const instance& inst) {
                        return deme.fields().hamming_distance(inst, center_inst);
                    };
                    std::vector<int> dsts;
                    std::transform(deme_inst_from, deme.end_instances(),
                                   back_inserter(dsts), dst_from_center);
                    auto pmm = boost::minmax_element(dsts.begin(), dsts.end());
                    nbh_dst << " from distance " << *pmm.first
                            << " to " << *pmm.second;
                }
                else nbh_dst << " at distance " << distance;
            }
            logger().debug(nbh_dst.str());
        }

        // score all new instances in the deme
        OMP_ALGO::transform(deme_from, deme.end(), deme_score_from,
                            // using bind cref so that score is passed by
                            // ref instead of by copy
                            boost::bind(boost::cref(iscorer), _1));

        // Check if there is an instance in the deme better than
        // the best candidate
        score_t prev_hi = best_score;
        score_t prev_best_raw = best_raw_score;

        unsigned ibest = current_number_of_instances;
        for (unsigned i = current_number_of_instances;
             deme.begin() + i != deme.end(); ++i) {
            const composite_score &inst_cscore = deme[i].second;
            score_t iscore = inst_cscore.get_penalized_score();
            if (iscore >  best_score) {
                best_cscore = inst_cscore;
                best_score = iscore;
                ibest = i;
            }

            // The instance with the best raw score will typically
            // *not* be the same as the the one with the best
            // weighted score.  We need the raw score for the
            // termination condition, as, in the final answer, we
            // want the best raw score, not the best weighted score.
            score_t rscore = inst_cscore.get_score();
            if (rscore >  best_raw_score) {
                best_raw_score = rscore;
            }
        }

        bool has_improved = opt_params.score_improved(best_score, prev_hi);

        // Make a copy of the best instance.
        if (has_improved) {
            center_inst = deme[ibest].first;
            already_xover = false;
        }

#ifdef GATHER_STATS
        if (iteration > 1) {
            if (scores.size() < number_of_new_instances) {
                unsigned old = scores.size();
                scores.resize(number_of_new_instances);
                counts.resize(number_of_new_instances);
                for (unsigned i=old; i<number_of_new_instances; i++) {
                    scores[i] = 0.0;
                    counts[i] = 0.0;
                }
            }

            // Gather statistics: compute the average distribution
            // of scores, as compared to the previous high score.
            std::sort(next(deme.begin(), current_number_of_instances),
                      deme.end(),
                      std::greater<deme_inst_t>());
            for (unsigned i = current_number_of_instances;
                 next(deme.begin(), i) != deme.end(); ++i)
            {
                composite_score inst_cscore = deme[i].second;
                score_t iscore = get_penalized_score(inst_cscore);
                unsigned j = i - current_number_of_instances;
                scores[j] += iscore;
                counts[j] += 1.0;

                if (prev_hi < iscore) num_improved += 1.0;
            }
            hiscore += prev_hi;
            hicount += 1.0;
            count_improved += 1.0;
        }
#endif
        current_number_of_evals += number_of_new_instances;
        current_number_of_instances += number_of_new_instances;

        if (hc_params.allow_resize_deme) {
            // Keep the size of the deme at a managable level.
            // Large populations can easily blow out the RAM on a machine,
            // so we want to keep it at some reasonably trim level.
            //
            // To avoid wasting cpu time pointlessly, don't bother with
            // population size management if its already small.  Thus, 
            // ACCEPTABLE_SIZE is fairly "large".
            //
            // "deme_usage" is the RAM usage; the goal is to avoid OOM
            // kills by keeping RAM usage to a reasonable level.
            uint64_t deme_usage = _instance_bytes * current_number_of_instances;

#define ACCEPTABLE_SIZE 5000
#define ACCEPTABLE_RAM_FRACTION 0.5
            if ((ACCEPTABLE_SIZE < current_number_of_instances) or
                (ACCEPTABLE_RAM_FRACTION * _total_RAM_bytes < deme_usage))
            {
                bool did_resize = resize_deme(deme, best_score);

                // After resizing, some of the variables set above become
                // invalid.  Some are not needed any more: e.g. ibest.
                // Others, we need, esp prev_start and prev_size for the
                // cross-over to work.
                if (did_resize) {
                    current_number_of_instances = deme.size();
                    prev_start = 0;
                    prev_size = current_number_of_instances;
                }
            }
        }

        if (has_improved) {
            distance = 1;
            deme.n_best_evals = current_number_of_instances;

            if (logger().isDebugEnabled()) {
                logger().debug() << "Best score: " << best_cscore;
                if (logger().isFineEnabled()) {
                    logger().fine() << "Best instance: "
                                    << fields.to_string(center_inst);
                }
            }
        }
        else if (!xover)
            distance++;

        // Collect statistics about the run, in struct optim_stats
        nsteps ++;
        total_steps ++;
        total_evals += number_of_new_instances;
        struct timeval stop, elapsed;
        gettimeofday(&stop, NULL);
        timersub(&stop, &start, &elapsed);
        start = stop;
        unsigned usec = 1000000 * elapsed.tv_sec + elapsed.tv_usec;

        // Deme statistics, for performance graphing.
        if (logger().isInfoEnabled()) {
            double ram_usage = current_number_of_instances;
            ram_usage *= _instance_bytes;
            ram_usage /= 1024 * 1024;
            logger().info() << hc_params.prefix_stat_deme << "Hill: "
                << demeID << "\t"
                << iteration << "\t"
                << total_steps << "\t"
                << total_evals << "\t"
                << usec << "\t"
                << number_of_new_instances << "\t"
                << current_number_of_instances << "\t"
                << ram_usage << "\t"
                << current_number_of_evals << "\t"
                << has_improved << "\t"
                << best_score << "\t"   /* weighted score */
                << best_score - prev_hi << "\t"  /* previous weighted */
                << best_raw_score << "\t"     /* non-weighted, raw score */
                << best_raw_score - prev_best_raw << "\t"
                << best_cscore.get_complexity();
        }

        /* If this is the first time through the loop, then
         * distance was zero, there was only one instance at
         * dist=0, and we just scored it. Be sure to go around and
         * do at least the distance == 1 nearest-neighbor
         * exploration.  Note that it is possible to have only 1
         * neighbor with a distance greater than 0 (when the
         * distance has reached the deme dimension), which is why
         * we check that distance == 1 as well.
         */
        if (number_of_new_instances == 1 && distance == 1) continue;

        /* If we've blown our budget for evaluating the scorer,
         * then we are done. */
        if (max_evals <= current_number_of_evals) {
            over_budget = true;
            logger().debug("Terminate Local Search: Over budget");
            break;
        }

        if (max_time <= elapsed.tv_sec) {
            over_budget = true;
            logger().debug("Terminate Local Search: Out of time");
            break;
        }
        max_time -= elapsed.tv_sec; // count-down to zero.

        /* If we've aleady gotten the best possible score, we are done. */
        if (opt_params.terminate_if_gte <= best_raw_score) {
            logger().debug("Terminate Local Search: Found best score");
            break;
        }

        if (hc_params.crossover) {
            /* If the score hasn't taken a big step recently, then
             * re-survey the immediate local neighborhood.  We may get
             * lucky.
             */
            bool big_step = opt_params.score_improved(best_score, prev_hi);
            if ((not big_step) and (not last_chance)) {
                /* If we just did the nearest neighbors, and found no
                 * improvement, then try again with the simplexes.  That's
                 * cheap & quick and one last chance to get lucky ...
                 */
                if (not already_xover
                    // in the case we can widen the search check that
                    // the max distance has been reached, otherwise,
                    // it's not really a last_chance
                    and (not hc_params.widen_search or max_distance < distance))
                {
                    last_chance = true;
                    continue;
                }
            }

            /* Reset back to normal mode. */
            has_improved = big_step;
            last_chance = false;
        }

        /* If we've widened the search out to the max distance, we're done. */
        if (max_distance < distance) {
            logger().debug("Terminate Local Search: Max search distance exceeded");
            break;
        }

        /* If things haven't improved, we must be at the top of the hill.
         * Terminate, unless we've been asked to widen the search.
         * (i.e. to search for other nearby hills) */
        if (!has_improved && !hc_params.widen_search) {
            logger().debug("Terminate Local Search: No improvement");
            break;
        }

        /* If we're in single-step mode, then exit the loop as soon
         * as we find improvement. */
        if (hc_params.single_step && has_improved) {
            logger().debug("Terminate Local Search: Single-step and found improvment");
            break;
        }
    }

    deme.n_evals = current_number_of_evals;
}

size_t hill_climbing::estimate_neighborhood(size_t distance,
                                            const field_set& fields)
{
    if (distance == 0)
        return 1;
    
    const size_t nn_estimate = information_theoretic_bits(fields);

    if (nn_estimate < distance) {
        logger().warn("hill_climbing::estimate_neighborhood : "
                      "the distance %u is greater than the "
                      "theoretic bit field size %u. This could be a bug "
                      "(but not necessarily due to corner cases "
                      "and the fact that it's an approximation of the actual "
                      "field size)", distance, nn_estimate);
        distance = nn_estimate;
    }

    if (distance == 1)
        return 2*nn_estimate; // be large given nn_estimate is only... an estimate
    else {  // more than one

        // Distances greater than 1 occurs only when the -L1 or
        // -T1 flags are used.  This puts this algo into a very
        // different mode of operation, in an attempt to overcome
        // deceptive scoring functions.
        
        // For large-distance searches, there is a combinatorial
        // explosion of the size of the search volume. Thus, be
        // careful budget our available cycles.
        return safe_binomial_coefficient(nn_estimate, distance);
    }
}

size_t hill_climbing::n_new_instances(size_t distance, unsigned max_evals,
                                      size_t current_number_of_evals,
                                      size_t total_number_of_neighbors)
{
    if (distance == 0)
        return 1;
    
    size_t number_of_new_instances = total_number_of_neighbors;
    
    // binomial coefficient has a combinatoric explosion to the power
    // distance. So throttle back by fraction raised to power dist.
    for (unsigned k=0; k<distance; k++)
        number_of_new_instances *= hc_params.fraction_of_nn;
    
    // Clamp the number of nearest-neighbor evaluations
    // we'll do.  This is necessitated because some systems
    // have a vast number of nearest neighbors, and searching
    // them all explodes the RAM usage, for no gain.
    if (number_of_new_instances > hc_params.max_nn_evals)
        number_of_new_instances = hc_params.max_nn_evals;
    
    // avoid overflow.
    size_t nleft =
        max_evals - current_number_of_evals;
    
// we choose the number 100 because below that multithreading is
// disabled and it leads to some massive slow down because then most
// of the computational power is spent on successive representation
// building
#define MINIMUM_DEME_SIZE         100

    // If fraction is small, just use up the rest of the cycles.
    if (number_of_new_instances < MINIMUM_DEME_SIZE)
        number_of_new_instances = nleft;
    
    if (nleft < number_of_new_instances)
        number_of_new_instances = nleft;
    
    if (hc_params.allow_resize_deme) {
        // To avoid being OOM-killed, set ACCEPTABLE_RAM_FRACTION to
        // half of installed RAM on the machine. This should work in a
        // more or less scalable fashion on all machines, and still
        // allow instances that are dozens of megabytes in size.
        // (This is targeting machines with 4 GB to 100 GB of RAM).
        //
        // Currently, each disc knob takes 2 bits per instance, and
        // each contin knob takes 10 bits per instance.  Thus, the
        // practical limit is about 1M contin knobs on current-era
        // machines, assuming that max_num_instances is about 10K.
        //
        // Note: A similar calculation is performed when the scored
        // instances are merged into the deme, to limit the deme size.
#define MAX_RAM_LIMIT 0.9
        uint64_t new_usage = _instance_bytes * number_of_new_instances;

        if (ACCEPTABLE_RAM_FRACTION * _total_RAM_bytes < new_usage)
        {
            // Cap ram usage at the lesser of the desired usage,
            // or the actual available space.
            uint64_t free_ram = getFreeRAM();
            uint64_t cap = std::min(ACCEPTABLE_RAM_FRACTION * _total_RAM_bytes, 
                               MAX_RAM_LIMIT * free_ram);
            number_of_new_instances = cap / _instance_bytes;
            logger().debug("Cap new instances. "
                           "Tot RAM=%Ld new_usage=%Ld Free RAM=%Ld cap=%Ld",
                           _total_RAM_bytes, new_usage, free_ram, cap);
        }
    }

    logger().debug("Budget %u samples out of estimated %u neighbors",
                   number_of_new_instances, total_number_of_neighbors);

    return number_of_new_instances;
}

size_t hill_climbing::cross_top_one(deme_t& deme,
                                    size_t deme_size,
                                    size_t num_to_make,
                                    size_t sample_start,
                                    size_t sample_size,
                                    const instance& base)
{
    OC_ASSERT (sample_size > 0, "Cross-over sample size must be positive");
    if (sample_size-1 < num_to_make) num_to_make = sample_size-1;

    // We need to access the high-scorers.
    // We don't actually need them all sorted; we only need
    // the highest-scoring num_to_make+1 to appear first, and
    // after that, we don't care about the ordering.
    std::partial_sort(next(deme.begin(), sample_start),
                      next(deme.begin(), sample_start + num_to_make+1),
                      next(deme.begin(), sample_start + sample_size),
                      std::greater<deme_inst_t>());

    deme.resize(deme_size + num_to_make);

    // best instance
    const instance &reference = deme[sample_start].first;

    // Skip the first entry; its the top scorer.
    for (unsigned i = 0; i< num_to_make; i++) {
        unsigned j = deme_size + i;
        deme[j] = deme[sample_start + i + 1]; // +1 to skip the first one
        deme.fields().merge_instance(deme[j], base, reference);
    }
    return num_to_make;
}

/** two-dimensional simplex version of above. */
size_t hill_climbing::cross_top_two(deme_t& deme,
                                    size_t deme_size,
                                    size_t num_to_make,
                                    size_t sample_start,
                                    size_t sample_size,
                                    const instance& base)
{
    // sample_size choose two.
    unsigned max = sample_size * (sample_size-1) / 2;
    if (max < num_to_make) num_to_make = max;

    unsigned num_to_sort = sqrtf(2*num_to_make) + 3;
    if (sample_size < num_to_sort) num_to_sort = sample_size;
    std::partial_sort(next(deme.begin(), sample_start),
                      next(deme.begin(), sample_start + num_to_sort),
                      next(deme.begin(), sample_start + sample_size),
                      std::greater<deme_inst_t >());

    deme.resize(deme_size + num_to_make);

    // Summation is over a 2-simplex
    for (unsigned i = 1; i < sample_size; i++) {

        // (i+1)th best instance
        const instance& reference = deme[sample_start+i].first;

        // merge with all best instance (better than reference) but
        // using reference as the one containing the mutations to pass
        // on the other (better) instance.
        for (unsigned j = 0; j < i; j++) {
            unsigned n = i*(i-1)/2 + j;
            if (num_to_make <= n) return num_to_make;
            unsigned ntgt = deme_size + n;
            deme[ntgt] = deme[sample_start + j];
            deme.fields().merge_instance(deme[ntgt], base, reference);
        }
    }
    return num_to_make;
}

/** three-dimensional simplex version of above. */
size_t hill_climbing::cross_top_three(deme_t& deme,
                                      size_t deme_size,
                                      size_t num_to_make,
                                      size_t sample_start,
                                      size_t sample_size,
                                      const instance& base)
{
    // sample_size choose three.
    unsigned max = sample_size * (sample_size-1) * (sample_size-2) / 6;
    if (max < num_to_make) num_to_make = max;

    unsigned num_to_sort = cbrtf(6*num_to_make) + 3;
    if (sample_size < num_to_sort) num_to_sort = sample_size;
    std::partial_sort(next(deme.begin(), sample_start),
                      next(deme.begin(), sample_start + num_to_sort),
                      next(deme.begin(), sample_start + sample_size),
                      std::greater<deme_inst_t>());
    deme.resize(deme_size + num_to_make);

    // Summation is over a 3-simplex
    for (unsigned i = 2; i < sample_size; i++) {
        const instance& iref = deme[sample_start+i].first;
        for (unsigned j = 1; j < i; j++) {
            const instance& jref = deme[sample_start+j].first;
            for (unsigned k = 0; k < i; k++) {
                unsigned n = i*(i-1)*(i-2)/6 + j*(j-1)/2 + k;
                if (num_to_make <= n) return num_to_make;
                unsigned ntgt = deme_size + n;
                deme[ntgt] = deme[sample_start + k];
                deme.fields().merge_instance(deme[ntgt], base, iref);
                deme.fields().merge_instance(deme[ntgt], base, jref);
            }
        }
    }
    return num_to_make;
}

size_t hill_climbing::crossover(deme_t& deme, size_t deme_size,
                                size_t sample_start, size_t sample_size,
                                const instance& base) {
    // These cross-over (in the genetic sense) the
    // top-scoring one, two and three instances,respectively.
    size_t number_of_new_instances =
        cross_top_one(deme, deme_size, hc_params.crossover_pop_size / 3,
                      sample_start, sample_size, base);
    
    number_of_new_instances +=
        cross_top_two(deme, deme_size + number_of_new_instances,
                      hc_params.crossover_pop_size / 3,
                      sample_start, sample_size, base);

    number_of_new_instances +=
        cross_top_three(deme, deme_size + number_of_new_instances,
                        hc_params.crossover_pop_size / 3,
                        sample_start, sample_size, base);

    return number_of_new_instances;
}

        
/// Shrink the deme, by removing all instances with score less than
/// 'cutoff'.  This is implemented with in-place deletion of elements
/// from a vector, with at least a token attempt to delete contigous
/// regions of low scores, in one go.  It is possible that a faster
/// algorithm would be to sort first, and then delete the tail-end of
/// the vector.  But this ixsn't know ... XXX experiment with this!?
/// ... err, but right now, trimming takes a small fraction of a second,
/// so there is no rush to fis this.
size_t hill_climbing::resize_by_score(deme_t& deme, score_t cutoff)
{
    size_t ndeleted = 0;
    while (true) {
         auto first = deme.end();
         auto last = deme.end();
         size_t contig = 0;
         for (auto it = deme.begin(); it != deme.end(); it++) {
             score_t iscore = it->second.get_penalized_score();
             if (iscore <= cutoff) {
                 if (0 == contig) first = it;
                 last = it;
                 contig ++;
             } else {
                 if (0 < contig)
                     break;
             }
         }

         if (0 == contig) 
             break;

         if (last != deme.end()) last++;
         deme.erase(first, last);
         ndeleted += contig;

         // Keep around at least 20 instances; useful for cross-over.
#define MIN_TO_KEEP 20
         if (deme.size() < MIN_TO_KEEP)
             break;
    }
    logger().debug() << "Trimmed "
            << ndeleted << " low scoring instances.";
    return deme.size();
}

/// Keep the size of the deme at a managable level.
/// Large populations can easily blow out the RAM on a machine,
/// so we want to keep it at some reasonably trim level.
//
bool hill_climbing::resize_deme(deme_t& deme, score_t best_score)
{
    bool did_resize =  false;
    // Lets see how many we might be able to trounce.
    score_t cutoff = best_score - hc_params.score_range;
    size_t bad_score_cnt = 0;

    // To find the number of bad scores, we have to look
    // at the *whole* deme.
    for (const deme_inst_t& si : deme) {
        score_t iscore = si.second.get_penalized_score();
        if (iscore <=  cutoff)
            bad_score_cnt++;
    }


    // To avoid wasting cpu time pointlessly, don't bother with
    // population size management if we don't get any bang out
    // of it.
#define DONT_BOTHER_SIZE 500
    uint64_t usage = _instance_bytes * deme.size();

    if ((DONT_BOTHER_SIZE < bad_score_cnt) or
        (ACCEPTABLE_RAM_FRACTION * _total_RAM_bytes < usage))
    {

        logger().debug() << "Will trim " << bad_score_cnt
            << " low scoring instances out of " << deme.size();
        resize_by_score(deme, cutoff);
        did_resize = true;
    }

    // Are we still too large? Whack more, if needed.
    // We want to whack only the worst scorerers, and thus
    // a partial sort up front.
    if ((hc_params.max_allowed_instances < deme.size()) or
        (ACCEPTABLE_RAM_FRACTION * _total_RAM_bytes < usage))
    {
        std::partial_sort(deme.begin(),
                          next(deme.begin(), hc_params.max_allowed_instances),
                          deme.end(),
                          std::greater<deme_inst_t>());

        deme.erase(next(deme.begin(), hc_params.max_allowed_instances),
                   deme.end());
        did_resize = true;
    }
    return did_resize;
}

void hill_climbing::log_stats_legend()
{
    logger().info() << hc_params.prefix_stat_deme << "Hill: # "   /* Legend for graph stats */
        "demeID\t"
        "iteration\t"
        "total_steps\t"
        "total_evals\t"
        "microseconds\t"
        "new_instances\t"
        "num_instances\t"
        "inst_RAM\t"
        "num_evals\t"
        "has_improved\t"
        "best_weighted_score\t"
        "delta_weighted\t"
        "best_raw\t"
        "delta_raw\t"
        "complexity";
}
 
} // ~namespace moses
} // ~namespace opencog

