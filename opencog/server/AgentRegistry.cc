/*
 * opencog/server/AgentRegistry.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#include "AgentRegistry.h"

#include <map>

#include <opencog/server/AgentFactory.h>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>

using namespace opencog;

bool AgentRegistry::registerAgent(std::string const& id, AbstractFactory<Agent> const* factory)
{
    fprintf(stderr, "Registering agent \"%s\"\n", id.c_str());
    return factories.insert(FactoryMap::value_type(id, factory)).second;
}

bool AgentRegistry::unregisterAgent(const std::string &id)
{
    logger().info("unregistering agent \"%s\"", id.c_str());
    return factories.erase(id) == 1;
}

Agent* AgentRegistry::createAgent(const std::string &id)
{
    FactoryMap::const_iterator it = factories.find(id);
    if (it == factories.end()) {
        // not found
        throw RuntimeException(TRACE_INFO, "unknown agent id: %s", id.c_str());
    }
    // invoke the creation function
    logger().info("creating agent \"%s\"", id.c_str());
    return it->second->create();
}
