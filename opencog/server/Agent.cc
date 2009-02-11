/*
 * opencog/server/Agent.cc
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

#include "Agent.h"

#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>

using namespace opencog;

void Agent::setParameters(const std::string* params) {
    PARAMETERS = params;

    for (unsigned int i = 0; params[i] != ""; i += 2) {
        if (!config().has(params[i])) {
           config().set(params[i], params[i + 1]);
        }
    }
}

// Some disused method from before
/*void Agent::init()
{
    server().plugInAgent(this, 1);    
}*/

std::string Agent::to_string() const
{
    std::ostringstream oss;
    oss << classinfo().id;
    oss << " {\"";
    for (unsigned int i = 0; PARAMETERS[i] != ""; i += 2) {
        if (i != 0) oss << "\", \"";
        oss << PARAMETERS[i] << "\" => \"" << config()[PARAMETERS[i]];
    }
    oss << "\"}";
    return oss.str();
}

void Agent::resetUtilizedHandleSets()
{
    for (size_t i = 0; i < _utilizedHandleSets.size(); i++)
        delete _utilizedHandleSets[i];
    _utilizedHandleSets.clear();
}
