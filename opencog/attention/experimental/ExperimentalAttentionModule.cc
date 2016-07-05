/*
 * attention/ExperimentalAttentionModule.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Misgana Bayetta && Roman Treutlein
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

#include "ExperimentalAttentionModule.h"

#include "opencog/attention/atom_types.definitions"

using namespace opencog;

DECLARE_MODULE(ExperimentalAttentionModule)

ExperimentalAttentionModule::ExperimentalAttentionModule(CogServer& cs) :
    Module(cs)
{
    // New Thread based ECAN agents.
    _cogserver.registerAgent(AFImportanceDiffusionAgent::info().id, &afImportanceFactory);
    _cogserver.registerAgent(WAImportanceDiffusionAgent::info().id, &waImportanceFactory);

    _cogserver.registerAgent(AFRentCollectionAgent::info().id, &afRentFactory);
    _cogserver.registerAgent(WARentCollectionAgent::info().id, &waRentFactory);

    _afImportanceAgentPtr = _cogserver.createAgent(AFImportanceDiffusionAgent::info().id,false);
    _waImportanceAgentPtr = _cogserver.createAgent(WAImportanceDiffusionAgent::info().id,false);

    _afRentAgentPtr = _cogserver.createAgent(AFRentCollectionAgent::info().id, false);
    _waRentAgentPtr = _cogserver.createAgent(WARentCollectionAgent::info().id, false);
 }

ExperimentalAttentionModule::~ExperimentalAttentionModule()
{
    logger().debug("[ExperimentalAttentionModule] enter destructor");

    _cogserver.unregisterAgent(AFImportanceDiffusionAgent::info().id);
    _cogserver.unregisterAgent(WAImportanceDiffusionAgent::info().id);

     do_start_ecan_unregister();

    logger().debug("[ExperimentalAttentionModule] exit destructor");
}

void ExperimentalAttentionModule::init()
{
    do_start_ecan_register();
}

std::string ExperimentalAttentionModule::do_start_ecan(Request *req, std::list<std::string> args)
{
 std::string afImportance = AFImportanceDiffusionAgent::info().id;
 std::string waImportance = WAImportanceDiffusionAgent::info().id;

 std::string afRent = AFRentCollectionAgent::info().id;
 std::string waRent = WARentCollectionAgent::info().id;

 _cogserver.startAgent(_afImportanceAgentPtr, true, afImportance);
 _cogserver.startAgent(_waImportanceAgentPtr, true, waImportance);

 _cogserver.startAgent(_afRentAgentPtr, true, afRent);
 _cogserver.startAgent(_waRentAgentPtr, true, waRent);

 return ("Started the following agents:\n" + afImportance + "\n" + waImportance +
	 "\n" + afRent + "\n" + waRent + "\n");
}
