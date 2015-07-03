/*
 * examples/modules/AgentROS.cc
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

#include <ros/ros.h>
#include <AgentROS/string_atom.h>

#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/util/Logger.h>

#include "AgentROS/AgentROS.h"

using namespace opencog;

// load/unload functions for the Module interface
DECLARE_MODULE(rosModule)

rosModule::rosModule(CogServer& cs) : Module(cs)
{
    logger().info("[rosModule] constructor");
}

rosModule::~rosModule()
{
    logger().info("[rosModule] destructor");
    _cogserver.destroyAllAgents(AgentR::info().id);
}

void rosModule::init()
{
    logger().info("[rosModule] init");
    _cogserver.registerAgent(AgentR::info().id, &factory);
    _cogserver.createAgent(AgentR::info().id, true);
    char* argv[]={"dummy"};
    int argc=1;
    ros::init(argc,argv,"string_atom_server");
}

AgentR* AgentR::thisAgent;
AgentR::AgentR(CogServer& cs) : Agent(cs, 100)
{
    logger().info("[AgentR] constructor");
    thing_a_ma_bob = 0;
    thisAgent=(this);
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("add_string_atom", add);
}

AgentR::~AgentR()
{
    logger().info("[AgentROS] destructor");
}

void AgentR::run()
{
    thing_a_ma_bob++;
    logger().info("[AgentR] run; thing amabob is now %d", thing_a_ma_bob);
    ros::spinOnce();
}
bool AgentR::add(AgentROS::string_atom::Request  &req,
            AgentROS::string_atom::Response &res)
{
	ROS_INFO("%s",req.name.c_str());
	AtomSpace& as=thisAgent->_cogserver.getAtomSpace();
	std::stringstream ss;
	ss << as;
	ROS_INFO("%s",ss.str().c_str());
	std::cout<<as;
	res.ok=true;
	return true;
}
