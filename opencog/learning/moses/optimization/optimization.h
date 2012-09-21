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

#include <opencog/util/selection.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>

#include "../representation/instance_set.h"
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
                        const iscorer_base& iscorer, unsigned max_evals);

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
                        unsigned* eval_best = NULL);

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
                        const iscorer_base& iscorer, unsigned max_evals);

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
