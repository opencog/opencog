/*
 * opencog/embodiment/Control/Procedure/BuiltInProcedureRepository.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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
#include <opencog/util/StringManipulator.h>
#include <opencog/util/exceptions.h>

#include <opencog/embodiment/Control/Procedure/BuiltIn/PetActionSchema.h>

#include "BuiltInProcedureRepository.h"

using namespace Procedure;
using namespace opencog::pai;

BuiltInProcedureRepository::BuiltInProcedureRepository(PAI& pai)
{

    // Schemata for each single Pet Action
    // TODO: The lines commented out bellow corresponds to the pet actions that have
    // Vector or Rotation as type of one of their arguments. These arg types has no corresponding
    // combo type for now. See TODO marks in PetActionSchema.cc file.
    add(new PetActionSchema(pai, ActionType::BARK()));
    add(new PetActionSchema(pai, ActionType::TRICK_FOR_FOOD()));
    add(new PetActionSchema(pai, ActionType::EAT()));
    //add(new PetActionSchema(pai, ActionType::WALK()));
    add(new PetActionSchema(pai, ActionType::GRAB()));
    add(new PetActionSchema(pai, ActionType::DROP()));
    add(new PetActionSchema(pai, ActionType::SIT()));
    //add(new PetActionSchema(pai, ActionType::JUMP()));
    add(new PetActionSchema(pai, ActionType::LIE_DOWN()));
    //add(new PetActionSchema(pai, ActionType::FLY()));
    add(new PetActionSchema(pai, ActionType::STRETCH()));
    add(new PetActionSchema(pai, ActionType::SCRATCH_SELF_NOSE())); // TODO: Add other scratchSelf* actions
    add(new PetActionSchema(pai, ActionType::RUN_IN_CIRCLE()));
    add(new PetActionSchema(pai, ActionType::ANTICIPATE_PLAY()));
    add(new PetActionSchema(pai, ActionType::BEG()));
    add(new PetActionSchema(pai, ActionType::HEEL()));
    add(new PetActionSchema(pai, ActionType::HIDE_FACE()));
    add(new PetActionSchema(pai, ActionType::PLAY_DEAD()));
    add(new PetActionSchema(pai, ActionType::FOLLOW()));
    add(new PetActionSchema(pai, ActionType::LICK()));
    //add(new PetActionSchema(pai, ActionType::NUDGE()));
    add(new PetActionSchema(pai, ActionType::TAP_DANCE()));
    add(new PetActionSchema(pai, ActionType::BARE_TEETH()));
    add(new PetActionSchema(pai, ActionType::GROWL()));
    add(new PetActionSchema(pai, ActionType::LOOK_UP_TURN_HEAD()));
    add(new PetActionSchema(pai, ActionType::WHINE()));
    add(new PetActionSchema(pai, ActionType::SNIFF()));
    add(new PetActionSchema(pai, ActionType::SHAKE_HEAD()));
    add(new PetActionSchema(pai, ActionType::EARS_BACK()));
    add(new PetActionSchema(pai, ActionType::EARS_TWITCH()));
    //add(new PetActionSchema(pai, ActionType::MOVE_HEAD()));
    add(new PetActionSchema(pai, ActionType::WAKE()));
    add(new PetActionSchema(pai, ActionType::SLEEP()));
    add(new PetActionSchema(pai, ActionType::DRINK()));
    add(new PetActionSchema(pai, ActionType::PEE()));
    add(new PetActionSchema(pai, ActionType::POO()));

    add(new PetActionSchema(pai, ActionType::PAY_ATTENTION()));
    //add(new PetActionSchema(pai, ActionType::WAG_TAIL()));
    //add(new PetActionSchema(pai, ActionType::MOVE_TAIL()));

    add(new PetActionSchema(pai, ActionType::BUILD_BLOCK_AT()));
    add(new PetActionSchema(pai, ActionType::DESTROY_BLOCK_AT()));

    // TODO: Create and add all other builtIn procedures here
}

BuiltInProcedureRepository::~BuiltInProcedureRepository()
{
    for (Name2ProcedureMap::iterator itr = _map.begin(); itr != _map.end(); itr++) {
//  std::cout << itr->second->getName() << "\n";
        delete itr->second;
    }
}

void BuiltInProcedureRepository::add(BuiltInProcedure* proc)
{
    OC_ASSERT(!contains(proc->getName()),
                     "BuitInProcedureRepository - Repository already contains procedure %s.",
                     proc->getName().c_str());
    _map[proc->getName()] = proc;
}

bool BuiltInProcedureRepository::contains(const std::string& name) const
{
    return _map.find(name) != _map.end();
}

const BuiltInProcedure& BuiltInProcedureRepository::get(const std::string& name) const
{
    return *(_map.find(name)->second);
}

bool BuiltInProcedureRepository::update(AtomSpace& atomspace)
{

    return true;
}
