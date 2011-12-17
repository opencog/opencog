/*
 * opencog/embodiment/Learning/PetaverseHC/petaverse-hillclimber.cc
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

#include "petaverse-hillclimber.h"
#include <opencog/embodiment/Learning/RewritingRules/RewritingRules.h>

namespace opencog { namespace hillclimbing {

//constructor, generate/filter conditions and actions
petaverse_hillclimber::petaverse_hillclimber(int nepc,
        const FE& fitness_estimator,
        const definite_object_set& dos,
        const operator_set& eo,
        const combo_tree_ns_set& conditions,
        const combo_tree_ns_set& actions,
        bool abibb,
        bool neic,
        bool reduct_enabled,
        opencog::RandGen& rng)
        : _comp(dos),
        _elementary_operators(eo), _conditions(conditions),
        _actions(actions), _rng(rng),
        _hillclimber(fitness_estimator, nepc, _elementary_operators,
                     _conditions, _actions, _comp,
                     hillclimbing_action_reduction(),
                     hillclimbing_full_reduction(),
                     abibb, neic, reduct_enabled)
{

    //right after run the operator once to have already a learned candidate
    _hillclimber();
}

petaverse_hillclimber::~petaverse_hillclimber() { }

void petaverse_hillclimber::operator()()
{
    _hillclimber();
}

const combo_tree& petaverse_hillclimber::best_program()
{
    _best_program = _hillclimber.best_program();
    reduct::post_learning_rewrite(_best_program);
    return _best_program;
}

const combo_tree& petaverse_hillclimber::best_program_estimated()
{
    _best_program_estimated = _hillclimber.best_program_estimated();
    reduct::post_learning_rewrite(_best_program_estimated);
    return _best_program_estimated;
}

const combo_tree& petaverse_hillclimber::current_program()
{
    _current_program = _hillclimber.current_program(_rng);
    reduct::post_learning_rewrite(_current_program);
    return _current_program;
}

void petaverse_hillclimber::set_current_fitness(fitness_t f)
{
    _hillclimber.set_current_fitness(f);
}

void petaverse_hillclimber::reset_estimator()
{
    //reset hillclimber
    _hillclimber.reset_estimator();
}

} // ~namespace opencog
} // ~namespace hillclimbing
