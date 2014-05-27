/*
 * opencog/learning/moses/optimization/star-anneal.h
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
#ifndef _MOSES_STAR_ANNEAL_H
#define _MOSES_STAR_ANNEAL_H

#include <opencog/util/oc_assert.h>

#include "../representation/instance_set.h"
#include "../scoring/scoring.h"
#include "optimization.h"

namespace opencog { namespace moses {

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
    size_t max_new_instances;
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
struct simulated_annealing : optimizer_base
{
    typedef score_t energy_t;

    simulated_annealing(const optim_parameters& op = optim_parameters(),
                        const sa_parameters& sa = sa_parameters())
        : optimizer_base(op), sa_params(sa) {}

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

    energy_t energy(const deme_inst_t& inst)
    {
        return energy(inst.second.get_score());
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

    unsigned operator()(deme_t& deme,
                        const instance& init_inst,
                        const iscorer_base& iscorer,
                        unsigned max_evals,
                        time_t max_time);

    // like above but assumes that the initial instance is null
    unsigned operator()(deme_t& deme,
                        const iscorer_base& iscorer,
                        unsigned max_evals,
                        time_t max_time)
    {
        const instance init_inst(deme.fields().packed_width());
        return operator()(deme, init_inst, iscorer, max_evals, max_time);
    }

    sa_parameters sa_params;
protected:
    unsigned max_distance;
};

} // ~namespace moses
} // ~namespace opencog

#endif
