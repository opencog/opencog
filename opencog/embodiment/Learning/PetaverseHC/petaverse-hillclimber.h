/*
 * opencog/embodiment/Learning/PetaverseHC/petaverse-hillclimber.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef _HILLCLIMBING_PETAVERSE_HILLCLIMBER_H
#define _HILLCLIMBING_PETAVERSE_HILLCLIMBER_H

#include "hillclimber.h"
#include <opencog/embodiment/Learning/FitnessEstimator/DistortedComboSize.h>
#include <opencog/embodiment/Learning/ImitationLearningAPI/PetaverseImitationLearning.h>

namespace opencog { namespace hillclimbing {

class petaverse_hillclimber : public PetaverseImitationLearningBase
{

    typedef FitnessEstimator::NoSpaceLifeFitnessEstimator FE;
    typedef FitnessEstimator::DistortedComboSizeOrder Comp;

public:

    //- abibb stands for action_boolean_if_both_branches and is true
    //  the neighborhood generates both branches of a conditional
    //- neic stands for new_exemplar_initializes_center
    //  and if is true then everytime a new exemplar comes
    //  the center is initialized with the empty program instead of the best one
    petaverse_hillclimber(int nepc,
                          const FE& fitness_estimator,
                          const definite_object_set& dos,
                          const operator_set& eo,
                          const combo_tree_ns_set& conditions,
                          const combo_tree_ns_set& actions,
                          bool abibb,
                          bool neic,
                          bool reduct_enabled);

    ~petaverse_hillclimber();

    void operator()();

    const combo_tree& best_program();

    const combo_tree& best_program_estimated();

    const combo_tree& current_program();

    void set_current_fitness(fitness_t f);

    void reset_estimator();

private:
    Comp _comp;

    const std::set<vertex>& _elementary_operators;
    const combo_tree_ns_set& _conditions;
    const combo_tree_ns_set& _actions;

    hillclimber<FE, Comp> _hillclimber;

    combo_tree _best_program;
    combo_tree _best_program_estimated;
    combo_tree _current_program;

};

} // ~namespace opencog
} // ~namespace hillclimbing


#endif
