/*
 * opencog/learning/moses/optimization/star-anneal.cc
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

#include <opencog/util/oc_omp.h>

#include "../moses/neighborhood_sampling.h"

#include "star-anneal.h"

namespace opencog { namespace moses {

/////////////////////////
// Star-shaped search  //
/////////////////////////

// XXX TODO the annealing temperature control code should be ported over
// to the hill-climbing code, thus rendering the below obsolete.  The
// hill-climbing code is much more sophisticated in every way: correct
// definition of the temperature, termination conditions, exploration of
// likely instances via cross-over, sparse search when the problem
// becomes large, etc. and so the code below is esentially obsolete,
// except for the temperature-scaling idea.
//
// This ort is not urgent: it is not at all clear that simulated
// annealing (for moses) actually improves results or speeds
// convergence.  In particular, the optimal annealing schedule is
// unknown. (i.e. what the temperatures should be at each iteration).
//
void simulated_annealing::operator()(deme_t& deme,
                                     const instance& init_inst,
                                     const iscorer_base& iscorer,
                                     unsigned max_evals,
                                     time_t max_time)
{
    const field_set& fields = deme.fields();
    max_distance = opt_params.max_distance(fields);

    // @todo this should be adapted for SA
    unsigned pop_size = opt_params.pop_size(fields);
    // unsigned max_gens_total = information_theoretic_bits(deme.fields());
    size_t max_number_of_instances =
        /*(size_t)max_gens_total * */
        (size_t)pop_size;
    if (max_number_of_instances > max_evals)
        max_number_of_instances = max_evals;

    size_t current_number_of_instances = 0;

    unsigned step = 1;

    // Score the initial instance
    instance center_instance(init_inst);
    deme_inst_t scored_center_inst(center_instance, iscorer(center_instance));
    
    energy_t center_instance_energy = energy(scored_center_inst);
    double current_temp = sa_params.init_temp;

    // Logger
    {
        std::stringstream ss;
        ss << "Star search initial instance: " << fields.to_string(center_instance);
        logger().debug(ss.str());
        logger().debug("Energy = %f", center_instance_energy);
    }
    // ~Logger

    do {
        unsigned current_distance = dist_temp(current_temp);

        logger().debug("Step: %u  Distance = %u, Temperature = %f",
                       step, current_distance, current_temp);

        size_t number_of_new_instances =
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
        deme_inst_t& best_scored_instance =
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
                   << fields.to_string(center_instance);
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

    deme.n_evals = current_number_of_instances;
}

} // ~namespace moses
} // ~namespace opencog

