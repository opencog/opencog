/*
 * opencog/learning/moses/moses/optimization.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Predrag Janicic
 *            Nil Geisweiller
 *            Xiaohui Liu
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
#include "../moses/moses.h"
#include "../moses/neighborhood_sampling.h"

// we choose the number 100 because below that multithreading is
// disabled and it leads to some massive slow down because then most
// of the computational power is spent on successive representation
// building
#define MINIMUM_DEME_SIZE         100

#define MAX_EVALS_PER_SLICE       10

namespace opencog { namespace moses {

/**
 * information_theoretic_bits -- return information content of the field set.
 *
 * The information content of a single variable is defined as log base
 * two of the number of possible values that the variable can take.
 * Thus, for example, a single bit has two possible values, and an
 * information content of exactly one.
 *
 * This routine sums the total information content in a field set,
 * including that in contin and term fields, as well as the discrete
 * and boolean fields.
 */
inline double
information_theoretic_bits(const field_set& fs)
{
    double res = 0;
    foreach (const field_set::disc_spec& d, fs.disc_and_bits())
        res += log2<double>(d.multy);

    foreach (const field_set::contin_spec& c, fs.contin())
    {
        // number of possible contins with depth d is 2^(d+1)-1 because
        // after a Stop only Stop is allowed which is why it is not 3^d
        unsigned contin_count = pow2(c.depth + 1) - 1;
        res += log2<double>(contin_count);
    }

    foreach (const field_set::term_spec& o, fs.term())
        res += log2<double>(o.branching) * double(o.depth);

    return res;
}

// Parameters used mostly for EDA algorithms but also possibly by
// other algo
struct optim_parameters
{
    optim_parameters(double _pop_size_ratio = 20,
                     score_t _terminate_if_gte = 0,
                     double _max_dist_ratio = 1) :
        term_total(1.0),
        term_improv(1.0),

        window_size_pop(0.05), //window size for RTR is
        window_size_len(1),    //min(windowsize_pop*N,windowsize_len*n)

        pop_size_ratio(_pop_size_ratio),
        terminate_if_gte(_terminate_if_gte),
        max_dist_ratio(_max_dist_ratio) {}

    // N = p.popsize_ratio * n^1.05
    // XXX Why n^1.05 ??? This is going to have a significant effect
    // (as compared to n^1.00) only when n is many thousands or bigger...
    inline unsigned pop_size(const field_set& fs)
    {
        return ceil(pop_size_ratio *
                     pow(information_theoretic_bits(fs), 1.05));
    }

    // term_total * n
    inline unsigned max_gens_total(const field_set& fs)
    {
        return ceil(term_total * information_theoretic_bits(fs));
    }

    // term_improv*sqrt(n/w)
    inline unsigned max_gens_improv(const field_set& fs)
    {
        return ceil(term_improv*
                    sqrt(information_theoretic_bits(fs) /
                         rtr_window_size(fs)));
    }
    //min(windowsize_pop*N,windowsize_len*n)
    inline unsigned rtr_window_size(const field_set& fs) {
        return ceil(min(window_size_pop*pop_size(fs),
                        window_size_len*information_theoretic_bits(fs)));
    }

    // return max_dist_log_ratio * log2(information_theoretic_bits(fs))
    // XXX why is log2 a good idea here? Is this just a pragmatic choice?
    // or is there a theoretical reason?
    inline unsigned max_distance(const field_set& fs)
    {
        double md = max_dist_ratio*log2(information_theoretic_bits(fs));
        return max(1U, numeric_cast<unsigned>(md));
    }

    // optimization is terminated after term_total*n generations, or
    // term_improv*sqrt(n/w) consecutive generations with no
    // improvement (w=windowsize)
    double term_total;
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
    double max_dist_ratio;
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

struct univariate_optimization
{
    univariate_optimization(RandGen& _rng,
                            const optim_parameters& op = optim_parameters(),
                            const eda_parameters& ep = eda_parameters())
        : rng(_rng), opt_params(op), eda_params(ep) {}

    //return # of evaluations actually performed
    template<typename Scoring>
    unsigned operator()(instance_set<composite_score>& deme,
                        const Scoring& score, unsigned max_evals)
    {
        unsigned pop_size = opt_params.pop_size(deme.fields());
        unsigned max_gens_total = opt_params.max_gens_total(deme.fields());
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
        generate_initial_sample(deme.fields(), pop_size, deme.begin(),
                                deme.end(), rng);

        if (eda_params.is_tournament_selection()) {
            cout_log_best_and_gen logger;
            return optimize
                   (deme, n_select, n_generate, max_gens_total, score,
                    terminate_if_gte_or_no_improv<composite_score>
                    (composite_score(opt_params.terminate_if_gte,
                                      worst_composite_score.second),
                     max_gens_improv),
                    tournament_selection((unsigned)eda_params.selection, rng),
                    univariate(), local_structure_probs_learning(),
                    // rtr_replacement(deme.fields(),
                    //                      opt_params.rtr_window_size(deme.fields()),
                    //                      rng),
                    replace_the_worst(),
                    logger, rng);
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

    RandGen& rng;
    optim_parameters opt_params;
    eda_parameters eda_params;
};

struct hc_parameters
{
    hc_parameters(bool _terminate_if_improvement = true,
                  double _fraction_of_remaining = 0.1)
        : terminate_if_improvement(_terminate_if_improvement),
          fraction_of_remaining(_fraction_of_remaining) {}
    bool terminate_if_improvement;

    // Things work better if the pop_size_ratio is used to control
    // the allocation of resources.
    // XXX So we should probably remove this!?
    double fraction_of_remaining;
};

/**
 * local_search: search the local neighborhood of an instance.
 *
 * This optimizatin algo performs an exhaustive search of the local
 * neighborhood centered upon a specific instance. Search begins with
 * the neighborhood at Hamming distance=1 from the central instance.
 * If no improvement is found, then the size of the neighborhood is
 * increased, by incrementing the Hamming distance defining the
 * neighborhood.
 *
 * If the local neighborhood is too large, i.e. if it exceeds the 
 * number of allowed scoring-function evaluations, then this algo
 * samples just a part of it, but then exhaustively searches this
 * sub-sample.
 *
 * In call cases, the neighborhood searched is 'spherical'; that is,
 * only the instances that are equi-distant from the exemplar are
 * explored (i.e. at the same Hamming distance).  Contrast this with
 * the 'star_search' class below, which searches a star-shaped set
 * (exhaustively).
 */
struct local_search
{
    local_search(RandGen& _rng,
                           const optim_parameters& op = optim_parameters(),
                           const hc_parameters& hc = hc_parameters())
        : rng(_rng), opt_params(op), hc_params(hc) {}

    /**
     * Heuristic to estimate the probability that an improvement can
     * be gotten given that
     * - we sample the neighborhood at distance d,
     * - with N samples,
     * - the total number of neighbors at that distance is T
     */
    float prob_improvement(deme_size_t N, deme_size_t T, unsigned d,
                           const field_set& fields)
    {
        // The following is based on the (not always true) assumption
        // that there is an improvement in the neighborhood.
        static const deme_size_t NB = 10000; // number of better candidates in
                                             // the neighborhood at distance d
                                             // that number is a big lie!

        // If the entire neighborhood is explored then the improvement
        // is sure to be found.
        if (N >= T)
            return 1;

        // Approximation of the total number of candidates at distance
        // d. This figure is lower than the true value because the
        // knobs are not all binary. If the field set has only binary
        // knobs, then the binomial would be exactly correct.  We can
        // use this approximation because T is not the actual number of
        // neighbors either, but is also a bit smaller.
        //
        // information_theoretic_bits() counts the information content
        // in the field set. (sum log_2 of all field sizes).
        deme_size_t bT =
            safe_binomial_coefficient(information_theoretic_bits(fields), d);

        // proportion of good candidates in the neighborhood
        double B = std::min(1.0, (double)NB/(double)std::max(T, bT));

        return 1 - pow(1 - B, double(N));
    }

    /**
     * Perform search of the local neighborhood of an instance.  The
     * search is exhaustive if the local neighborhood is small; else
     * the local neighborhood is randomly sampled.
     *
     * @param deme     Where to store the candidates searched. The deme
     *                 is assumed to be empty.  If it is not empty, it
     *                 will be overwritten.
     * @prama init_inst Start the seach from this instance.
     * @param score the Scoring function.
     * @param max_evals The maximum number of evaluations to eperform.
     * @param eval_best returned: The number of evaluations performed
     *                  to reach the best solution.
     * @return number of evaluations actually performed. This will always
     *         be equal or larger than the eval_best return, as not all
     *         evaluations lead to the best solution.
     */
    template<typename Scoring>
    unsigned operator()(instance_set<composite_score>& deme,
                        const instance& init_inst,
                        const Scoring& score, unsigned max_evals,
                        unsigned* eval_best = NULL)
    {
        // Logger
        logger(). debug("Local Search Optimization");
        // ~Logger

        // Initial eval_best in case nothing is found.
        if (eval_best)
            *eval_best = 0;

        const field_set& fields = deme.fields();

        // pop_size == 20 * number of info-theoretic-bits in the field.
        // max_gens_total == number of info-theoretic-bits in the field.
        unsigned pop_size = opt_params.pop_size(fields);
        unsigned max_gens_total = opt_params.max_gens_total(fields);

        deme_size_t current_number_of_instances = 0;

        // max_number_of_instances == 20 * info-theo-bits squared.
        deme_size_t max_number_of_instances =
            (deme_size_t)max_gens_total * (deme_size_t)pop_size;

        // Clamp to the maximal # of evaluations allowed.
        // Note that for most short-running problems, that max_evals
        // will be *much* smaller than the product above, and so will
        // always be clamped.  Note also that 'max_evals' is actually
        // 'number of evals remaining', and so is constantly shrinking.
        // Later, in the loop below, we multiply this number by 1/10th
        // so that we explore only 1/10th of 'number evals remaining'
        // in this go-around.
        //
        // This has the perverse effect of wasting a lot of the
        // evaluations early, when its easy to find improvement, and
        // thus having very few evals left over when the going gets
        // tough.  XXX This needs to be fixed. XXX
        // 
        if (max_number_of_instances > max_evals)
            max_number_of_instances = max_evals;

        unsigned max_distance = opt_params.max_distance(fields);

        // Score the initial instance.
        instance center_inst(init_inst);
        composite_score best_cscore = worst_composite_score;

        unsigned distance = 0;
        unsigned iteration = 0;

        // Whether the score has improved during an iteration
        bool has_improved;
        do {
            has_improved = false;

            // Logger
            {
                logger().debug("Iteration: %u", iteration++);
            }
            // ~Logger

            // Number of instances to try, this go-around.
            // fraction_of_remaining is 1/10th
            deme_size_t number_of_new_instances =
                (max_number_of_instances - current_number_of_instances)
                * hc_params.fraction_of_remaining;

            if (number_of_new_instances < MINIMUM_DEME_SIZE)
                number_of_new_instances =
                    (max_number_of_instances - current_number_of_instances);

            // The number of all neighbours at the distance d. Stops
            // counting when above number_of_new_instances. Thus we have
            // two cases:
            // if (total_number_of_neighbours < number_of_new_instances)
            //    then total_number_of_neighbours is an exact count
            //    else its incorrect but slightly larger.
            deme_size_t total_number_of_neighbours =
                count_neighborhood_size(deme.fields(), center_inst, distance,
                                      number_of_new_instances);

            // Estimate the probability of an improvement and halt if too low
            // XXX This estimate is pretty hokey....
            float p_improv = prob_improvement(number_of_new_instances,
                                              total_number_of_neighbours,
                                              distance, fields);

            logger().debug("Estimated probability to find an improvement = %f",
                           p_improv);

            if (p_improv < 0.01) {
                logger().debug("The probability is too low to pursue the search",
                               p_improv);
                break;
            }

            // The current_number_of_instances arg is needed only to
            // be able to manage the size of the deme appropriately.
            number_of_new_instances =
                sample_new_instances(total_number_of_neighbours,
                                     number_of_new_instances,
                                     current_number_of_instances,
                                     center_inst, deme, distance, rng);

            // Logger
            logger().debug("Evaluate %u neighbors at distance %u",
                           number_of_new_instances, distance);
            // ~Logger

            // score all new instances in the deme
            OMP_ALGO::transform
                (deme.begin() + current_number_of_instances, deme.end(),
                 deme.begin_scores() + current_number_of_instances,
                 // using bind cref so that score is passed by
                 // ref instead of by copy
                 boost::bind(boost::cref(score), _1));

            // check if there is an instance in the deme better than
            // the best candidate
            for (unsigned i = current_number_of_instances;
                 deme.begin() + i != deme.end(); ++i) {
                composite_score inst_cscore = deme[i].second;
                if (inst_cscore > best_cscore) {
                    best_cscore = inst_cscore;
                    center_inst = deme[i].first;
                    has_improved = true;
                }
            }

            current_number_of_instances += number_of_new_instances;

            if (has_improved) {
                distance = 1;
                if (eval_best)
                    *eval_best = current_number_of_instances;
                // Logger
                {
                    std::stringstream ss;
                    ss << "Center instance: " << fields.stream(center_inst)
                       << " " << best_cscore;
                    logger().debug(ss.str());
                }
                // ~Logger
            }
            else
                distance++;

        } while ((!hc_params.terminate_if_improvement ||
                  !has_improved ||
                  current_number_of_instances == 1) &&
                 distance <= max_distance &&
                 current_number_of_instances < max_number_of_instances &&
                 get_score(best_cscore) < opt_params.terminate_if_gte);

        return current_number_of_instances;
    }

    // like above but assumes that init_inst is null (backward compatibility)
    template<typename Scoring>
    unsigned operator()(instance_set<composite_score>& deme,
                        const Scoring& score, unsigned max_evals)
    {
        instance init_inst(deme.fields().packed_width());
        return operator()(deme, init_inst, score, max_evals);
    }

    RandGen& rng;
    optim_parameters opt_params;
    hc_parameters hc_params;
};


////////////////////////////
// Star-shaped Set Search //
////////////////////////////

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
 * star_search: search a star-shaped set around the center instance.
 *
 * A 'star-shaped set' is a neighborhood of the exemplar with instances
 * at a variety of different (Hamming) distances from the center, with
 * some near, and some far.  The fraction of 'distant' instances is
 * controlled by a 'temperature'; the higher the temperature, the
 * greater the fraction of distant instances.  At zero temperature,
 * only the closest instances are explored, and this algo reduces to
 * the local search algo, above.  The higher the temperature, the
 * 'pointier' the star.
 *
 * The total number of instances investigated is controlled by parameters
 * passed in.
 */
struct star_search
{
    typedef score_t energy_t;

    star_search(RandGen& _rng,
                const optim_parameters& op = optim_parameters(),
                const sa_parameters& sa = sa_parameters())
        : rng(_rng),
          opt_params(op), sa_params(sa) {}

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

    template<typename Scoring>
    unsigned operator()(instance_set<composite_score>& deme,
                        const instance& init_inst,
                        const Scoring& score, unsigned max_evals)
    {
        const field_set& fields = deme.fields();
        max_distance = opt_params.max_distance(fields);

        // @todo this should be adapted for SA
        unsigned pop_size = opt_params.pop_size(fields);
        // unsigned max_gens_total = opt_params.max_gens_total(deme.fields());
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
            score_instance(center_instance, score);
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

            // Logger
            {
                logger().debug("Step: %u", step);
                logger().debug("Distance = %u, Temperature = %f",
                               current_distance, current_temp);
            }
            // ~Logger

            deme_size_t number_of_new_instances =
                sample_new_instances(sa_params.max_new_instances,
                                     current_number_of_instances,
                                     center_instance, deme,
                                     current_distance, rng);

            // score all new instances in the deme
            OMP_ALGO::transform(deme.begin() + current_number_of_instances,
                                deme.end(),
                                deme.begin_scores() + current_number_of_instances,
                                boost::bind(boost::cref(score), _1));

            // get the best instance
            scored_instance<composite_score>& best_scored_instance =
                *OMP_ALGO::max_element(deme.begin()
                                       + current_number_of_instances,
                                       deme.end());

            instance& best_instance = best_scored_instance.first;
            energy_t best_instance_energy = energy(best_scored_instance);

            // check if the current instance in the deme is better than
            // the center_instance
            double actual_accept_prob = sa_params.accept_prob_temp_intensity *
                accept_probability(best_instance_energy,
                                   best_instance_energy, current_temp);

            if (actual_accept_prob >= rng.randdouble()) {
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
    template<typename Scoring>
    unsigned operator()(instance_set<composite_score>& deme,
                        const Scoring& score, unsigned max_evals)
    {
        const instance init_inst(deme.fields().packed_width());
        return operator()(deme, init_inst, score, max_evals);
    }

    RandGen& rng;
    optim_parameters opt_params;
    sa_parameters sa_params;
protected:
    unsigned max_distance;
};

// for testing only
struct dummy_optimization {
};

} // ~namespace moses
} // ~namespace opencog

#endif
