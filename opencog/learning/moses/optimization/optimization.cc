/*
 * opencog/learning/moses/optimization/optimization.cc
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

#include <math.h>   // for sqrtf, cbrtf

#include <opencog/util/oc_omp.h>

#include "../eda/termination.h"
#include "../eda/replacement.h"
#include "../eda/logging.h"
#include "../eda/local_structure.h"
#include "../eda/optimize.h"
#include "../moses/neighborhood_sampling.h"

#include "optimization.h"

namespace opencog { namespace moses {

double
information_theoretic_bits(const field_set& fs)
{
    static double log_five = log2<double>(5.0);

    double res = 0;

    size_t n_disc_fields = fs.n_disc_fields();
    vector<field_set::disc_spec>::const_iterator it = fs.disc_and_bit().begin();
    for (size_t cnt=0; cnt < n_disc_fields; cnt++, it++) {
        const field_set::disc_spec& d = *it;
        res += log2<double>(d.multy);
    }

    res += fs.n_bits();  // log_2(2)==1
    res += fs.contin().size() * log_five;

    foreach (const field_set::term_spec& o, fs.term())
        res += log2<double>(o.branching) * double(o.depth);

    return res;
}

optim_parameters::optim_parameters(const string& _opt_algo,
                 double _pop_size_ratio,
                 score_t _terminate_if_gte,
                 size_t _max_dist,
                 score_t _min_score_improv) :
    opt_algo(_opt_algo),
    term_improv(1.0),

    window_size_pop(0.05), //window size for RTR is
    window_size_len(1),    //min(windowsize_pop*N,windowsize_len*n)

    pop_size_ratio(_pop_size_ratio),    
    terminate_if_gte(_terminate_if_gte),
    max_dist(_max_dist)
{
    set_min_score_improv(_min_score_improv);
}

// N = p.popsize_ratio * n^1.05
// XXX Why n^1.05 ??? This is going to have a significant effect
// (as compared to n^1.00) only when n is many thousands or bigger...
unsigned optim_parameters::pop_size(const field_set& fs)
{
    return ceil(pop_size_ratio *
                 pow(information_theoretic_bits(fs), 1.05));
}

// term_improv*sqrt(n/w)  Huh?
unsigned optim_parameters::max_gens_improv(const field_set& fs)
{
    return ceil(term_improv*
                sqrt(information_theoretic_bits(fs) /
                     rtr_window_size(fs)));
}

// min(windowsize_pop*N,windowsize_len*n)
unsigned optim_parameters::rtr_window_size(const field_set& fs)
{
    return ceil(min(window_size_pop*pop_size(fs),
                    window_size_len*information_theoretic_bits(fs)));
}

unsigned optim_parameters::max_distance(const field_set& fs)
{
    return std::min(max_dist, fs.dim_size());
}

/// The score must improve by at least 's' to be considered; else
/// the search is terminated.  If 's' is negative, then it is
/// interpreted as a fraction: so 's=0.05' means 'the score must
/// improve 5 percent'.  
void optim_parameters::set_min_score_improv(score_t s)
{
    min_score_improvement = s;
}

score_t optim_parameters::min_score_improv()
{
    return min_score_improvement;
}

bool optim_parameters::score_improved(score_t best_score, score_t prev_hi)
{
    bool big_step = false;
    score_t imp = min_score_improv();

    if (0.0 <= imp)
         big_step = (best_score >  prev_hi + imp);
    else {
         // Score has improved if it increased by 0.5, or if it
         // increased by |imp| percent.  One extra minus sign
         // because imp is negative...
         big_step = (best_score >  prev_hi - imp * fabs(prev_hi));
         // big_step | = (best_score >  prev_hi + 0.5);
    }

    return big_step;
}

/////////////////////////////
// Univariate Optimization //
/////////////////////////////

//return # of evaluations actually performed
unsigned univariate_optimization::operator()(instance_set<composite_score>& deme,
                    const iscorer_base& iscorer, unsigned max_evals)
{
    unsigned pop_size = opt_params.pop_size(deme.fields());
    unsigned max_gens_total = information_theoretic_bits(deme.fields());
    unsigned max_gens_improv = opt_params.max_gens_improv(deme.fields());
    unsigned n_select = double(pop_size) * eda_params.selection_ratio;
    unsigned n_generate = double(pop_size) * eda_params.replacement_ratio;

    // Adjust parameters based on the maximal # of evaluations allowed
    if (max_evals < pop_size) {
        pop_size = max_evals;
        max_gens_total = 0;
    } else {
        max_gens_total = min(max_gens_total,
                             (max_evals - pop_size) / n_generate);
    }

    // Create the initial sample
    // Generate the initial sample to populate the deme
    deme.resize(pop_size);
    generate_initial_sample(deme.fields(), pop_size, deme.begin(), deme.end());

    if (eda_params.is_tournament_selection()) {
        cout_log_best_and_gen logger;
        return optimize
               (deme, n_select, n_generate, max_gens_total, iscorer,
                terminate_if_gte_or_no_improv<composite_score>
                (composite_score(opt_params.terminate_if_gte,
                                  get_complexity(worst_composite_score),
                                  0),
                 max_gens_improv),
                tournament_selection((unsigned)eda_params.selection),
                univariate(), local_structure_probs_learning(),
                // rtr_replacement(deme.fields(),
                //                      opt_params.rtr_window_size(deme.fields())),
                replace_the_worst(),
                logger);
    } else { //truncation selection
        OC_ASSERT(false,
                  "Trunction selection not implemented."
                  " Tournament should be used instead.");
        return 42;
        /*
        return optimize(deme,n_select,n_generate,args.max_gens,score,
          terminate_if_gte_or_no_improv(opt_params.terminate_if_gte,
                   max_gens_improv),
          //truncation selection goes here
          univariate(),local_structure_probs_learning(),
          replace_the_worst(),cout_log_best_and_gen());
        */
    }
}

///////////////////
// Hill Climbing //
///////////////////

unsigned hill_climbing::operator()(instance_set<composite_score>& deme,
                    const instance& init_inst,
                    const iscorer_base& iscorer, unsigned max_evals,
                    unsigned* eval_best)
{
    logger().debug("Local Search Optimization");
    logger().info() << "Demes: # "   /* Legend for graph stats */
            "deme_count\t"
            "iteration\t"
            "total_steps\t"
            "total_evals\t"
            "microseconds\t"
            "new_instances\t"
            "num_instances\t"
            "has_improved\t"
            "best_weighted_score\t"
            "delta_weighted\t"
            "best_raw\t"
            "delta_raw\t"
            "complexity";


    // Collect statistics about the run, in struct optim_stats
    nsteps = 0;
    deme_count ++;
    over_budget = false;
    struct timeval start;
    gettimeofday(&start, NULL);

    // Initial eval_best in case nothing is found.
    if (eval_best)
        *eval_best = 0;

    const field_set& fields = deme.fields();

    // Estimate the number of nearest neighbors.
    deme_size_t nn_estimate = information_theoretic_bits(fields);
    field_set_size = nn_estimate;  // optim stats, printed by moses.

    deme_size_t current_number_of_instances = 0;

    unsigned max_distance = opt_params.max_distance(fields);

    // center_inst is the current location on the hill.
    // Copy it, don't reference it, since sorting will mess up a ref.
    instance center_inst(init_inst);
    composite_score best_cscore = worst_composite_score;
    score_t best_score = worst_score;
    score_t best_raw_score = worst_score;

    // Initial distance is zero, so that the first time through
    // the loop, we handle just one instance, the initial instance.
    // (which is at distance zero from itself, of course).
    unsigned distance = 0;
    unsigned iteration = 0;

    // Needed for correlated neighborhood exploration.
    instance prev_center = center_inst;
    deme_size_t prev_start = 0;
    deme_size_t prev_size = 0;

    bool rescan = false;
    bool last_chance = false;

    // Whether the score has improved during an iteration
    while (true)
    {
        logger().debug("Iteration: %u", ++iteration);

        // Estimate the number of neighbours at the distance d.
        // This is faster than actually counting.
        deme_size_t total_number_of_neighbours;

        // Number of instances to try, this go-around.
        deme_size_t number_of_new_instances;

        // For a distance of one, we use
        // information_theoretic_bits as an approximation of the
        // number of neighbors, over-estimated by a factor of 2,
        // just to decrease the chance of going into 'sample' mode
        // when working with the deme. This saves cpu time.
        if (distance == 1)
        {
            total_number_of_neighbours = 2*nn_estimate;
            number_of_new_instances = 2*nn_estimate;

            // fraction_of_nn is 1 by default
            number_of_new_instances *= hc_params.fraction_of_nn;

            // Clamp the number of nearest-neighbor evaluations
            // we'll do.  This is necessitated because some systems
            // have a vast number of nearest neighbors, and searching
            // them all explodes the RAM usage, for no gain.
            if (number_of_new_instances > hc_params.max_nn_evals)
                number_of_new_instances = hc_params.max_nn_evals;

            // avoid overflow.
            deme_size_t nleft =
                max_evals - current_number_of_instances;
            if (nleft < number_of_new_instances)
                number_of_new_instances = nleft;

        }
        else if (distance == 0)
        {
            total_number_of_neighbours = 1;
            number_of_new_instances = 1;
        }
        else // distance two or greater
        {
            // Distances greater than 1 occurs only when the -L1 flag
            // is used.  This puts this algo into a very different mode
            // of operation, in an attempt to overcome deceptive scoring
            // functions.

            // For large-distance searches, there is a combinatorial
            // explosion of the size of the search volume. Thus, be
            // careful budget our available cycles.
            total_number_of_neighbours =
                safe_binomial_coefficient(nn_estimate, distance);

            number_of_new_instances = total_number_of_neighbours;

            // binomial coefficient has a combinatoric explosion to
            // the power distance. So throttle back by fraction raised
            // to power dist.
            for (unsigned k=0; k<distance; k++)
               number_of_new_instances *= hc_params.fraction_of_nn;

            // Clamp the number of nearest-neighbor evaluations
            // we'll do.  This is necessitated because some systems
            // have a vast number of nearest neighbors, and searching
            // them all explodes the RAM usage, for no gain.
            if (number_of_new_instances > hc_params.max_nn_evals)
                number_of_new_instances = hc_params.max_nn_evals;

            deme_size_t nleft =
                max_evals - current_number_of_instances;

            // If fraction is small, just use up the rest of the cycles.
            if (number_of_new_instances < MINIMUM_DEME_SIZE)
                number_of_new_instances = nleft;

            // avoid overflow.
            if (nleft < number_of_new_instances)
                number_of_new_instances = nleft;

        }
        logger().debug(
            "Budget %u samples out of estimated %u neighbours",
            number_of_new_instances, total_number_of_neighbours);

        // The first few times through, (or, if we decided on a
        // full rescan), explore the entire nearest neighborhood.
        // Otherwise, make the optimistic assumption that the best
        // the best new instances are likely to be genric cross-overs
        // of the current top-scoring instances.  This assumption
        // seems to work really quite well.
        //
        // Based on experiments, crossing over the 40 highest-scoring
        // instances seems to offer plenty of luck, to TOP_POP_SIZE
        // is currently defined to be 40.
        //
        // If the size of the nearest neighborhood is small enough,
        // then don't bother with the optimistic guessing.  The
        // cross-over guessing will generate 3*TOP_POP_SIZE
        // instances.  Our guestimate for number of neighbors
        // intentionally over-estimates by a factor of two. So the
        // breakeven point would be 6*TOP_POP_SIZE, and we pad this
        // a bit, as small exhaustive searches do beat guessing...
#define TOP_POP_SIZE 40
        if (!hc_params.crossover || (iteration <= 2) || rescan ||
            total_number_of_neighbours < 10*TOP_POP_SIZE) {

            // The current_number_of_instances arg is needed only to
            // be able to manage the size of the deme appropriately.
            number_of_new_instances =
                sample_new_instances(total_number_of_neighbours,
                                     number_of_new_instances,
                                     current_number_of_instances,
                                     center_inst, deme, distance);
        } else {
            // These cross-over (in the genetic sense) the
            // top-scoring one, two and three instances,respectively.
            number_of_new_instances =
                cross_top_one(deme, current_number_of_instances, TOP_POP_SIZE,
                              prev_start, prev_size, prev_center);

            number_of_new_instances +=
                cross_top_two(deme,
                              current_number_of_instances + number_of_new_instances,
                              TOP_POP_SIZE,
                              prev_start, prev_size, prev_center);

            number_of_new_instances +=
                cross_top_three(deme,
                              current_number_of_instances + number_of_new_instances,
                              TOP_POP_SIZE,
                              prev_start, prev_size, prev_center);
        }
        prev_start = current_number_of_instances;
        prev_size = number_of_new_instances;
        prev_center = center_inst;

        logger().debug("Evaluate %u neighbors at distance %u",
                       number_of_new_instances, distance);

        // score all new instances in the deme
        OMP_ALGO::transform
            (next(deme.begin(), current_number_of_instances), deme.end(),
             next(deme.begin_scores(), current_number_of_instances),
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
            composite_score inst_cscore = deme[i].second;
            score_t iscore = get_penalized_score(inst_cscore);
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
            score_t rscore = get_score(inst_cscore);
            if (rscore >  best_raw_score) {
                best_raw_score = rscore;
            }
        }

        bool has_improved = opt_params.score_improved(best_score, prev_hi);

        // Make a copy of the best instance.
        if (has_improved)
            center_inst = deme[ibest].first;

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
                      std::greater<scored_instance<composite_score> >());
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
        current_number_of_instances += number_of_new_instances;

        if (has_improved) {
            distance = 1;
            if (eval_best)
                *eval_best = current_number_of_instances;

            if (logger().isDebugEnabled()) {
                logger().debug() << "Best score: " << best_cscore;
                if (logger().isFineEnabled()) {
                    logger().fine() << "Best instance: "
                                    << fields.stream(center_inst);
                }
            }
        }
        else
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
            logger().info() << "Demes: "
                << deme_count << "\t"
                << iteration << "\t"
                << total_steps << "\t"
                << total_evals << "\t"
                << usec << "\t"
                << number_of_new_instances << "\t"
                << current_number_of_instances << "\t"
                << has_improved << "\t"
                << best_score << "\t"   /* weighted score */
                << best_score - prev_hi << "\t"  /* previous weighted */
                << best_raw_score << "\t"     /* non-weighted, raw score */
                << best_raw_score - prev_best_raw << "\t"
                << get_complexity(best_cscore);
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
        if (max_evals <= current_number_of_instances) {
            over_budget = true;
            logger().debug("Terminate Local Search: Over budget");
            break;
        }

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
            if (!big_step && !last_chance) {

                /* If we've been using the simplex extrapolation
                 * (which is the case when 2<iteration), and there's
                 * been no improvement, then try a full nearest-
                 * neighborhood scan.  This tends to refresh the pool
                 * of candidates, and keep things going a while longer.
                 */
                if (!rescan && (2 < iteration)) {
                    rescan = true;
                    distance = 1;
                    continue;
                }

                /* If we just did the nearest neighbors, and found no
                 * improvement, then try again with the simplexes.  That's
                 * cheap & quick and one last chance to get lucky ...
                 */
                if (rescan || (2 == iteration)) {
                    rescan = false;
                    last_chance = true;
                    distance = 1;
                    continue;
                }
            }

            /* Reset back to normal mode. */
            has_improved = big_step;
            rescan = false;
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

    return current_number_of_instances;
}

deme_size_t hill_climbing::cross_top_one(instance_set<composite_score>& deme,
                          deme_size_t deme_size,
                          deme_size_t num_to_make,
                          deme_size_t sample_start,
                          deme_size_t sample_size,
                          const instance& base)
{
    if (sample_size-1 < num_to_make) num_to_make = sample_size-1;

    // We need to access the high-scorers.
    // We don't actually need them all sorted; we only need
    // the highest-scoring num_to_make+1 to appear first, and
    // after that, we don't care about the ordering.
    // std::sort(next(deme.begin(), sample_start),
    //          next(deme.begin(), sample_start + sample_size),
    //          std::greater<scored_instance<composite_score> >());
    std::partial_sort(next(deme.begin(), sample_start),
                      next(deme.begin(), sample_start + num_to_make+1),
                      next(deme.begin(), sample_start + sample_size),
                      std::greater<scored_instance<composite_score> >());

    deme.resize(deme_size + num_to_make);

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
deme_size_t hill_climbing::cross_top_two(instance_set<composite_score>& deme,
                          deme_size_t deme_size,
                          deme_size_t num_to_make,
                          deme_size_t sample_start,
                          deme_size_t sample_size,
                          const instance& base)
{
    // sample_size choose two.
    unsigned max = sample_size * (sample_size-1) / 2;
    if (max < num_to_make) num_to_make = max;

    // std::sort(next(deme.begin(), sample_start),
    //          next(deme.begin(), sample_start + sample_size),
    //          std::greater<scored_instance<composite_score> >());
    //
    unsigned num_to_sort = sqrtf(2*num_to_make) + 3;
    if (sample_size < num_to_sort) num_to_sort = sample_size;
    std::partial_sort(next(deme.begin(), sample_start),
                      next(deme.begin(), sample_start + num_to_sort),
                      next(deme.begin(), sample_start + sample_size),
                      std::greater<scored_instance<composite_score> >());

    deme.resize(deme_size + num_to_make);

    // Summation is over a 2-simplex
    for (unsigned i = 1; i < sample_size; i++) {
        const instance& reference = deme[sample_start+i].first;
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
deme_size_t hill_climbing::cross_top_three(instance_set<composite_score>& deme,
                          deme_size_t deme_size,
                          deme_size_t num_to_make,
                          deme_size_t sample_start,
                          deme_size_t sample_size,
                          const instance& base)
{
    // sample_size choose three.
    unsigned max = sample_size * (sample_size-1) * (sample_size-2) / 6;
    if (max < num_to_make) num_to_make = max;

    // std::sort(next(deme.begin(), sample_start),
    //           next(deme.begin(), sample_start + sample_size),
    //           std::greater<scored_instance<composite_score> >());

    unsigned num_to_sort = cbrtf(6*num_to_make) + 3;
    if (sample_size < num_to_sort) num_to_sort = sample_size;
    std::partial_sort(next(deme.begin(), sample_start),
                      next(deme.begin(), sample_start + num_to_make),
                      next(deme.begin(), sample_start + sample_size),
                      std::greater<scored_instance<composite_score> >());
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

/////////////////////////
// Star-shaped search  //
/////////////////////////

unsigned simulated_annealing::operator()(instance_set<composite_score>& deme,
                    const instance& init_inst,
                    const iscorer_base& iscorer, unsigned max_evals)
{
    const field_set& fields = deme.fields();
    max_distance = opt_params.max_distance(fields);

    // @todo this should be adapted for SA
    unsigned pop_size = opt_params.pop_size(fields);
    // unsigned max_gens_total = information_theoretic_bits(deme.fields());
    deme_size_t max_number_of_instances =
        /*(deme_size_t)max_gens_total * */
        (deme_size_t)pop_size;
    if (max_number_of_instances > max_evals)
        max_number_of_instances = max_evals;

    deme_size_t current_number_of_instances = 0;

    unsigned step = 1;

    // Score the initial instance
    instance center_instance(init_inst);
    scored_instance<composite_score> scored_center_inst =
        score_instance(center_instance, iscorer);
    energy_t center_instance_energy = energy(scored_center_inst);
    double current_temp = sa_params.init_temp;

    // Logger
    {
        std::stringstream ss;
        ss << "Star search initial instance: " << fields.stream(center_instance);
        logger().debug(ss.str());
        logger().debug("Energy = %f", center_instance_energy);
    }
    // ~Logger

    do {
        unsigned current_distance = dist_temp(current_temp);

        logger().debug("Step: %u  Distance = %u, Temperature = %f",
                       step, current_distance, current_temp);

        deme_size_t number_of_new_instances =
            sample_new_instances(sa_params.max_new_instances,
                                 current_number_of_instances,
                                 center_instance, deme,
                                 current_distance);

        // If the temperature is too high, then the distance will
        // be too large, and no instances will be found at that
        // distance.
        if (0 == number_of_new_instances) {
            current_temp = cooling_schedule((double)step
                                            * sa_params.temp_step_size);
            step++;
            continue;
         }

        // score all new instances in the deme
        OMP_ALGO::transform(next(deme.begin(), current_number_of_instances),
                            deme.end(),
                            next(deme.begin_scores(), current_number_of_instances),
                            boost::bind(boost::cref(iscorer), _1));

        // get the best instance
        scored_instance<composite_score>& best_scored_instance =
            *OMP_ALGO::max_element(next(deme.begin(), current_number_of_instances),
                                   deme.end());

        instance& best_instance = best_scored_instance.first;
        energy_t best_instance_energy = energy(best_scored_instance);

        // check if the current instance in the deme is better than
        // the center_instance
        double actual_accept_prob = sa_params.accept_prob_temp_intensity *
            accept_probability(best_instance_energy,
                               best_instance_energy, current_temp);

        if (actual_accept_prob >= randGen().randdouble()) {
            center_instance_energy = best_instance_energy;
            center_instance = best_instance;
            // Logger
            {
                std::stringstream ss;
                ss << "Center instance: "
                   << fields.stream(center_instance);
                logger().debug(ss.str());
                logger().debug("Energy = %f", center_instance_energy);
            }
            // ~Logger
        }

        current_number_of_instances += number_of_new_instances;

        current_temp = cooling_schedule((double)step
                                        * sa_params.temp_step_size);
        step++;
    } while(current_number_of_instances < max_number_of_instances &&
            current_temp >= sa_params.min_temp &&
            center_instance_energy > energy(opt_params.terminate_if_gte));

    return current_number_of_instances;
}

} // ~namespace moses
} // ~namespace opencog

