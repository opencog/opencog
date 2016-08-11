/*
 * opencog/cogserver/server/Agent.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
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

#include <opencog/cogserver/server/CogServer.h>
#include <opencog/util/Config.h>

#define DEBUG

using namespace opencog;

Agent::Agent(CogServer& cs, const unsigned int f) :
    _log(nullptr), _cogserver(cs), _frequency(f)
{
    setParameters({""});

    _as = &cs.getAtomSpace();
}

Agent::~Agent()
{
    resetUtilizedHandleSets();

    if (_log) delete _log;
}

void Agent::setLogger(Logger* l)
{
    if (_log) delete _log;
    _log = l;
}

Logger* Agent::getLogger()
{
    return _log;
}


void Agent::setParameters(const std::vector<std::string>& params)
{
    _parameters = params;
    for (unsigned int i = 0; params[i] != ""; i += 2)
    {
        if (!config().has(params[i]))
           config().set(params[i], params[i + 1]);
    }
}

std::string Agent::to_string() const
{
    std::ostringstream oss;
    oss << classinfo().id;
    oss << " {\"";
    for (unsigned int i = 0; _parameters[i] != ""; i += 2) {
        if (i != 0) oss << "\", \"";
        oss << _parameters[i] << "\" => \"" << config()[_parameters[i]];
    }
    oss << "\"}";
    return oss.str();
}

void Agent::atomRemoved(const AtomPtr& atom)
{
    Handle h(atom->getHandle());
    {
        std::lock_guard<std::mutex> lock(_handleSetMutex);
        for (size_t i = 0; i < _utilizedHandleSets.size(); i++)
            _utilizedHandleSets[i].erase(h);
    }
}

void Agent::resetUtilizedHandleSets(void)
{
    std::lock_guard<std::mutex> lock(_handleSetMutex);
    for (size_t i = 0; i < _utilizedHandleSets.size(); i++)
        _utilizedHandleSets[i].clear();
    _utilizedHandleSets.clear();
}
