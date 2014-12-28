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

#include <opencog/embodiment/Control/Procedure/BuiltIn/AvatarActionSchema.h>

#include "BuiltInProcedureRepository.h"

namespace opencog { namespace Procedure {

using namespace pai;

BuiltInProcedureRepository::BuiltInProcedureRepository(PAI& pai)
{

    // Schemata for each single Pet Action
    // TODO: The lines commented out bellow corresponds to the pet actions that have
    // Vector or Rotation as type of one of their arguments. These arg types has no corresponding
    // combo type for now. See TODO marks in AvatarActionSchema.cc file.
    add(new AvatarActionSchema(pai, ActionType::EAT()));
    //add(new AvatarActionSchema(pai, ActionType::WALK()));
    add(new AvatarActionSchema(pai, ActionType::GRAB()));
    add(new AvatarActionSchema(pai, ActionType::DROP()));
    add(new AvatarActionSchema(pai, ActionType::SIT()));
    //add(new AvatarActionSchema(pai, ActionType::JUMP()));
    //add(new AvatarActionSchema(pai, ActionType::FLY()));
    add(new AvatarActionSchema(pai, ActionType::FOLLOW()));
    //add(new AvatarActionSchema(pai, ActionType::NUDGE()));
    //add(new AvatarActionSchema(pai, ActionType::MOVE_HEAD()));
    add(new AvatarActionSchema(pai, ActionType::WAKE()));
    add(new AvatarActionSchema(pai, ActionType::SLEEP()));
    add(new AvatarActionSchema(pai, ActionType::DRINK()));

    add(new AvatarActionSchema(pai, ActionType::PAY_ATTENTION()));
    //add(new AvatarActionSchema(pai, ActionType::WAG_TAIL()));
    //add(new AvatarActionSchema(pai, ActionType::MOVE_TAIL()));

    add(new AvatarActionSchema(pai, ActionType::BUILD_BLOCK()));
    add(new AvatarActionSchema(pai, ActionType::DESTROY_BLOCK()));

    // TODO: Create and add all other builtIn procedures here
}

BuiltInProcedureRepository::~BuiltInProcedureRepository()
{
    for (Name2ProcedureMap::iterator itr = _map.begin(); itr != _map.end(); ++itr) {
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

}} // ~namespace opencog::Procedure
