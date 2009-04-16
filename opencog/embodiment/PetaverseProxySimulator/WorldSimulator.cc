/*
 * opencog/embodiment/PetaverseProxySimulator/WorldSimulator.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#include "WorldSimulator.h"
#include <opencog/util/Logger.h>

using namespace PetaverseProxySimulator;
using namespace opencog;

WorldSimulator::~WorldSimulator()
{
}

WorldSimulator::WorldSimulator()
{
}

std::string WorldSimulator::createAgent(const std::string& name, const std::string& type, float x, float y, bool echoing)
{
    logger().log(opencog::Logger::INFO, "Created agent %s in the sim world", name.c_str());
    return name;
}

void WorldSimulator::timeTick()
{
    logger().log(opencog::Logger::INFO, "Time tick to sim world");
}

unsigned long WorldSimulator::executeAction(const std::string& agentName, const PerceptionActionInterface::PetAction &petAction)
{
    logger().log(opencog::Logger::WARN, "_PVPSimulator_UNIT_TEST_TAG_ Executing action for agent '%s': %s",
                 agentName.c_str(), petAction.stringRepresentation().c_str());
    //return 0; Cannot return 0 because this would require NetworkElement for sending action status for this...
    return ULONG_MAX - 1;
}

