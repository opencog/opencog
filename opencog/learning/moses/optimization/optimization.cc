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

