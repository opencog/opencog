/*
 * opencog/embodiment/Learning/PetaverseHC/interactive-hillclimber.h
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

#ifndef _HILLCLIMBING_INTERACTIVE_HILLCLIMBER_H
#define _HILLCLIMBING_INTERACTIVE_HILLCLIMBER_H

#include <opencog/learning/hillclimbing/hillclimber.h>
#include <opencog/embodiment/Learning/FitnessEstimator/DistortedComboSize.h>
#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>

namespace opencog { namespace hillclimbing {

using namespace ant_combo;
    
template<typename FitnessEstimator>
struct interactive_hillclimber {
    interactive_hillclimber(const FitnessEstimator& fe, int fepc) {
        //enumerate operators used in hillclimbing
        _elementary_operators.insert(id::sequential_and);
        _elementary_operators.insert(id::action_boolean_if);
        //enumerate actions used in hillclimbing
        combo_tree a1(ant_builtin_action::get_instance(id::turn_left));
        combo_tree a2(ant_builtin_action::get_instance(id::turn_right));
        combo_tree a3(ant_builtin_action::get_instance(id::move_forward));
        _actions.insert(a1);
        _actions.insert(a2);
        _actions.insert(a3);
        //enumarate intuitions used in hillclimbing
        combo_tree p(ant_perception::get_instance(id::is_food_ahead));
        _perceptions.insert(p);

        _hillclimber = new hillclimber<FitnessEstimator>
        (fe, fepc, _elementary_operators, _perceptions, _actions, _comp,
         action_reduction(), action_reduction(),
         false, false);

    }

    ~interactive_hillclimber() {
        delete(_hillclimber);
    }

    void operator()() {
        (*_hillclimber)();
    }

    fitness_t best_fitness() const {
        return _hillclimber->best_fitness();
    }
    const combo_tree& best_program() const {
        return _hillclimber->best_program();
    }

    fitness_t best_fitness_estimated() const {
        return _hillclimber->best_fitness_estimated();
    }
    const combo_tree& best_program_estimated() const {
        return _hillclimber->best_program_estimated();
    }

    fitness_t current_fitness() const {
        return _hillclimber->current_fitness();
    }
    void set_current_fitness(fitness_t f) {
        _hillclimber->set_current_fitness(f);
    }
    const combo_tree& current_program(RandGen& rng) {
        return _hillclimber->current_program(rng);
    }

    void reset_cache() {
        _hillclimber->reset_cache();
    }

private:
    size_tree_order<vertex> _comp;
    hillclimber<FitnessEstimator>* _hillclimber;
    std::set<vertex> _elementary_operators;
    combo_tree_ns_set _actions;
    combo_tree_ns_set _perceptions;
};

}// ~namespace opencog
}// ~namespace hillclimbing

#endif
