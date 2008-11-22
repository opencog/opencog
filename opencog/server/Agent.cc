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

using namespace opencog;

static void Agent::setDefaults() {
    Config& conf = Config.config();
     
    for (unsigned int i = 0; PARAMETERS()[i] != ""; i += 2) {
        if (!conf.has(PARAMETERS()[i])) {
           conf.set(PARAMETERS()[i], PARAMETERS()[i + 1]);
        }
    }
}

void Agent::init()
{
    server().plugInAgent(this, 1);
    setDefaults();
}

std::string Agent::to_string() const
{
    Config& conf = Config.config();

    std::ostringstream oss;
    oss << classinfo().id;
    oss << " {\"";
    for (unsigned int i = 0; PARAMETERS()[i] != ""; i += 2) {
        if (i != 0) oss << "\", \"";
        oss << PARAMETERS()[i] << "\" => \"" << conf[PARAMETERS()[i]];
    }
    oss << "\"}";
    return oss.str();
}
