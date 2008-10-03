/*
 * opencog/server/Agent.h
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

#ifndef _OPENCOG_AGENT_H
#define _OPENCOG_AGENT_H

#include <string>

#include <opencog/server/Factory.h>

namespace opencog
{

class CogServer;

class Agent
{

protected:

    int _frequency;

public:

    Agent(const unsigned int f = 1) : _frequency(f) {}
    virtual ~Agent() {}

    virtual void run(CogServer* server) = 0;
    virtual int frequency(void) const { return _frequency; }
    virtual const ClassInfo& classinfo() const = 0;

}; // class

}  // namespace

#endif // _OPENCOG_AGENT_H
