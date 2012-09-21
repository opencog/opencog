/*
 * opencog/learning/moses/optimization/optimization.h
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
#ifndef _MOSES_OPTIMIZATION_H
#define _MOSES_OPTIMIZATION_H

#include <math.h>   // for sqrtf, cbrtf
#include <opencog/util/selection.h>
#include <opencog/util/dorepeat.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/oc_omp.h>

#include "../eda/termination.h"
#include "../eda/replacement.h"
#include "../eda/logging.h"
#include "../eda/local_structure.h"
#include "../eda/optimize.h"
#include "../representation/instance_set.h"
#include "../moses/neighborhood_sampling.h"
#include "../moses/scoring.h"

// we choose the number 100 because below that multithreading is
// disabled and it leads to some massive slow down because then most
// of the computational power is spent on successive representation
// building
#define MINIMUM_DEME_SIZE         100

namespace opencog { namespace moses {

/**
 * information_theoretic_bits -- return information content of the field set.
 *
 * The information content of a single variable is defined as log base
 * two of the number of possible values that the variable can take.
 * Thus, for example, a single bit has two possible values, and an
 * information content of exactly one.
 *
 * The information-theoretic content of a contin variable is log_2(5).
 * This is because there are five ways to alter a contin: twiddle the
 * least-significant pseudo-bit (2 choices: L,R), erase the least
 * significant bit (1 choice), add a new least-significant bit
 * (2 choices: L,R).
 *
 * This routine sums the total information content in a field set,
 * including that in contin and term fields, as well as the discrete
 * and boolean fields.
 *
 * This is usually a good estimate for the total number of nearest
 * neighbors of an instance, rarely differing by more than twenty
 * percent. This is because the nearest neighbors of an instance are
 * those that differ by a Hamming distance of one, and the total
 * length of the instance is approximately equal to info-theo-bits!
 */
double information_theoretic_bits(const field_set& fs);

/// Hill-climbing paramters
struct hc_parameters
{
    hc_parameters(bool widen = false,
                  bool step = false,
                  bool cross = false,
                  unsigned max_evals = 20000,
                  double _fraction_of_nn = 2.0) // > 1.0 since works on estimate only.
        : widen_search(widen),
          single_step(step),
          crossover(cross),
          max_nn_evals (max_evals),
          fraction_of_nn(_fraction_of_nn)
    {
        OC_ASSERT(0.0 < fraction_of_nn);
    }
    
    bool widen_search;
    bool single_step;
    bool crossover;

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
};

// optimization algorithms
static const std::string un("un"); // univariate
static const std::string sa("sa"); // star-shaped search
static const std::string hc("hc"); // local search

// Parameters used mostly for EDA algorithms but also possibly by
// other algo
struct optim_parameters
{
    optim_parameters(const string& _opt_algo = hc,
                     double _pop_size_ratio = 20,
                     score_t _terminate_if_gte = 0,
                     size_t _max_dist = 4,
                     score_t _min_score_improv = 0.5);

    // N = p.popsize_ratio * n^1.05
    // XXX Why n^1.05 ??? This is going to have a significant effect
    // (as compared to n^1.00) only when n is many thousands or bigger...
    unsigned pop_size(const field_set& fs);

    // term_improv*sqrt(n/w)  Huh?
    unsigned max_gens_improv(const field_set& fs);

    // min(windowsize_pop*N,windowsize_len*n)
    unsigned rtr_window_size(const field_set& fs);

    unsigned max_distance(const field_set& fs);

    /// The score must improve by at least 's' to be considered; else
    /// the search is terminated.  If 's' is negative, then it is
    /// interpreted as a fraction: so 's=0.05' means 'the score must
    /// improve 5 percent'.  
    void set_min_score_improv(score_t s);

    score_t min_score_improv();

    bool score_improved(score_t best_score, score_t prev_hi);

    // String name of the optimization algo to employ
    string opt_algo;

    // optimization is terminated after n generations, or
    // term_improv*sqrt(n/w) consecutive generations with no
    // improvement (w=windowsize)
    double term_improv;

    double window_size_pop;
    double window_size_len;

    // Populations are sized at N = popsize_ratio*n^1.05 where n is
    // problem size in info-theoretic bits.  (XXX Why 1.05 ???)
    double pop_size_ratio;

    // Optimization is terminated if best score is >= terminate_if_gte
    score_t terminate_if_gte;

    // Defines the max distance to search during one iteration (used
    // in method max_distance)
    size_t max_dist;

    // Some hill-climbing-specific parameters.
    hc_parameters hc_params;

private:
    // We accept improvement only if the score improved by that amount
    // or better, to avoid fine-tuning. It may be better to restart
    // the search from a new exemplar rather wasting time climbing a
    // near-plateau. Most problems have 1.0 as the smallest meaningful
    // score change, so 0.5 as default seems reasonable...
    score_t min_score_improvement;
};

// Enable some additional algorithm-dependent statistics gathering.
// Turned off by default because it's a CPU-waster.
// #define GATHER_STATS 1

/// Statistics obtained during optimization run, useful for tuning.
struct optim_stats
{
    optim_stats()
        : nsteps(0), deme_count(0), total_steps(0), total_evals(0),
        field_set_size(0), over_budget(false)
#ifdef GATHER_STATS
          , hiscore(0.0), hicount(0.0), 
          num_improved(0.0), count_improved(0.0)
#endif
    {}
    unsigned nsteps;
    unsigned deme_count;
    unsigned total_steps;
    unsigned total_evals;
    unsigned field_set_size;
    bool over_budget;

    // Additional stats.
#ifdef GATHER_STATS
    vector<double> scores;
    vector<double> counts;
    double hiscore;
    double hicount;
    double num_improved;
    double count_improved;
#endif
};

// Parameters specific to EDA optimization
struct eda_parameters
{
    eda_parameters() :
        selection(2),          //if <=1, truncation selection ratio,
                               //if >1, tournament selection size (should be int)
        selection_ratio(1),    //ratio of population size selected for modeling

        replacement_ratio(0.5),//ratio of population size sampled and integrated

        model_complexity(1)    //model parsimony term log(N)*model_complexity
    {}

    bool is_tournament_selection() {
        return selection > 1;
    }
    bool is_truncation_selection() {
        return selection <= 1;
    }

    double selection;
    double selection_ratio;
    double replacement_ratio;
    double model_complexity;
};

struct univariate_optimization : optim_stats
{
    univariate_optimization(const optim_parameters& op = optim_parameters(),
                            const eda_parameters& ep = eda_parameters())
        : opt_params(op), eda_params(ep) {}

    //return # of evaluations actually performed
    unsigned operator()(instance_set<composite_score>& deme,
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

    optim_parameters opt_params;
    eda_parameters eda_params;
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
struct hill_climbing : optim_stats
{
    hill_climbing(const optim_parameters& op = optim_parameters())
        : opt_params(op)
    {
        hc_params = opt_params.hc_params;
    }

    /**
     * Cross the single top-scoring instance against the next-highest scorers.
     *
     * As arguments, accepts a range of scored instances ("the sample"),
     * and a singlee instance from which these were all derived ("the base"
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
    deme_size_t cross_top_one(instance_set<composite_score>& deme,
                              deme_size_t deme_size,
                              deme_size_t num_to_make,
                              deme_size_t sample_start,
                              deme_size_t sample_size,
                              const instance& base);

    /** two-dimensional simplex version of above. */
    deme_size_t cross_top_two(instance_set<composite_score>& deme,
                              deme_size_t deme_size,
                              deme_size_t num_to_make,
                              deme_size_t sample_start,
                              deme_size_t sample_size,
                              const instance& base);

    /** three-dimensional simplex version of above. */
    deme_size_t cross_top_three(instance_set<composite_score>& deme,
                              deme_size_t deme_size,
                              deme_size_t num_to_make,
                              deme_size_t sample_start,
                              deme_size_t sample_size,
                              const instance& base);

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
     * @param max_evals The maximum number of evaluations to eperform.
     * @param eval_best returned: The number of evaluations performed
     *                  to reach the best solution.
     * @return number of evaluations actually performed. This will always
     *         be equal or larger than the eval_best return, as not all
     *         evaluations lead to the best solution.
     */
    unsigned operator()(instance_set<composite_score>& deme,
                        const instance& init_inst,
                        const iscorer_base& iscorer, unsigned max_evals,
                        unsigned* eval_best = NULL)
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

    // Like above but assumes that init_inst is null (backward compatibility)
    // XXX In fact, all of the current code uses this entry point, no one
    // bothers to supply an initial instance.
    unsigned operator()(instance_set<composite_score>& deme,
                        const iscorer_base& iscorer, unsigned max_evals)
    {
        instance init_inst(deme.fields().packed_width());
        return operator()(deme, init_inst, iscorer, max_evals);
    }

    optim_parameters opt_params;
    hc_parameters hc_params;
};

/////////////////////////
// Star-shaped search  //
/////////////////////////

// Parameters specific for Star-shaped set search
struct sa_parameters
{
    sa_parameters() :
        init_temp(30),
        min_temp(0),
        temp_step_size(0.5),
        accept_prob_temp_intensity(0.5),
        max_new_instances(100) {}

    double init_temp;
    double min_temp;
    double temp_step_size;
    double accept_prob_temp_intensity;
    deme_size_t max_new_instances;
};

/**
 * Star-shaped search: Apply a modified smulated annealing-style search.
 * Deprecated; its not really simulated annealing, but something else,
 * and this really doesn't work very well. Its here for backward
 * compatibility, for the moment.
 *
 * The search pattern used is similar to the -L1 -T1 -I0 variant of
 * hill-climbing, except that, instead of always searching the nearest
 * neighbors, it occasionally broadens the search to a larger
 * neighborhood, looking at more distant neighbors (thus the name "star
 * shape", as the common term for such non-convex sets in mathematics).
 * The fraction of more distant neighbors explored is determined by a
 * temperature parameter, thus the allusion to annealing. In addition,
 * the same temperature is also used to keep eithr the new best instance,
 * or the old one.  Why these two different, unrelated things should ever
 * have the same temperature is one of the reasons the implementation
 * below is broken/deprecated.
 *
 * A "true" simulated-annealing algo would have worked like this:
 * a) Explore the nearest-neighborhood of a "center instance".
 * b) Using a Boltzmann distribution, select either the highest scroring
 *    instance, or one of the next-best few, as the new center instance.
 * c) Goto step a).
 * but that's NOT AT ALL what this algo does.
 */
struct simulated_annealing : optim_stats
{
    typedef score_t energy_t;

    simulated_annealing(const optim_parameters& op = optim_parameters(),
                        const sa_parameters& sa = sa_parameters())
        : opt_params(op), sa_params(sa) {}

    double accept_probability(energy_t energy_new, energy_t energy_old,
                              double temperature)
    {
        if (energy_new < energy_old)
            return 1.0;
        else
            return std::exp((energy_old - energy_new) / temperature);
    }

    double cooling_schedule(double t)
    {
        OC_ASSERT(t > 0, "t should greater than 0");
        //return (double) init_temp / std::log(1.0 + t);
        return (double) sa_params.init_temp / (1.0 + t);
    }

    energy_t energy(score_t sc)
    {
        // The energy is minus the score. This is because better scores
        // are bigger numbers, whereas, in SA, better scores correspond
        // to lower energies.
        return -sc;
    }

    energy_t energy(const scored_instance<composite_score>& inst)
    {
        return energy(get_score(inst.second));
    }

    /**
     * This method calculate the distance of the jump according to the
     * temperature. The higher the temperature the higher the
     * distance.  @todo: it may be better to have the distance
     * decreasing exponentially instead of linearly
     */
    unsigned dist_temp(double current_temp)
    {
        return (unsigned)( ((current_temp - sa_params.min_temp)
                            /(sa_params.init_temp - sa_params.min_temp))
                           *
                           (max_distance - 1) + 1 );
    }

    unsigned operator()(instance_set<composite_score>& deme,
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

    // like above but assumes that the initial instance is null
    unsigned operator()(instance_set<composite_score>& deme,
                        const iscorer_base& iscorer, unsigned max_evals)
    {
        const instance init_inst(deme.fields().packed_width());
        return operator()(deme, init_inst, iscorer, max_evals);
    }

    optim_parameters opt_params;
    sa_parameters sa_params;
protected:
    unsigned max_distance;
};

} // ~namespace moses
} // ~namespace opencog

#endif
