/*
 * examples/modules/AgentROS.h
 *
 * Copyright (C) 2015 opencog
 * All Rights Reserved
 *
 * Written by Mandeep Singh Bhatia
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

class AgentR : public Agent
{
public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::AgentROS");
        return _ci;
    }

    AgentR(CogServer&);
    //ros::NodeHandle n;
    ros::NodeHandle *n;
    ros::ServiceServer *service;
    virtual ~AgentR();
    virtual void run();

private:
    int thing_a_ma_bob;
    static bool add(AgentROS::string_atom::Request  &req,
            AgentROS::string_atom::Response &res);
    static AgentR* thisAgent;

}; // class

class rosModule : public Module
{
private:

    Factory<AgentR, Agent> factory;

public:

    rosModule(CogServer&);
    virtual ~rosModule();
    virtual void init();
    //virtual const char* id();
    static inline const char* id();
};

} // namespace opencog

#endif // _OPENCOG_EXAMPLE_AGENT_H
