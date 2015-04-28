/*
 * examples/modules/ExampleAgent.h
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

#ifndef _OPENCOG_EXAMPLE_AGENT_H
#define _OPENCOG_EXAMPLE_AGENT_H

#include <string>

#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>

namespace opencog
{

class CogServer;

class ExampleAgent : public Agent
{
public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::ExampleAgent");
        return _ci;
    }

    ExampleAgent(CogServer&);
    virtual ~ExampleAgent();
    virtual void run();

private:
    int thing_a_ma_bob;

}; // class

class ExampleModule : public Module
{
private:

    Factory<ExampleAgent, Agent> factory;

public:

    ExampleModule(CogServer&);
    virtual ~ExampleModule();
    virtual void init();
    virtual const char* id();
};

} // namespace opencog

#endif // _OPENCOG_EXAMPLE_AGENT_H
