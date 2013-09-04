/*
 * opencog/embodiment/Control/OperationalAvatarController/HCTestAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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


#include "HCTestAgent.h"
#include "OAC.h"

#define IDLE_TIME 1
//replaced by learning_time1
//#define WAIT1_TIME 10 //time to wait for the first learning iteration
#define WAIT2_TIME 2
//replaced by learning_time2
//#define WAIT3_TIME 10000 //time to wait for the second learning iteration
#define WAIT4_TIME 2

using namespace opencog::oac;
using namespace opencog;

HCTestAgent::HCTestAgent(CogServer& cs) : Agent(cs)
{
}
void HCTestAgent::init(std::string sn, std::vector<std::string> schemaArgs,
                       std::string a, std::string b, AtomSpace* as,
                       MessageSender* s, unsigned int lt1,
                       unsigned int lt2, unsigned long mc) {
    mode = HCT_INIT;
    cycle = 0;
    schemaName = sn;
    schemaArguments = schemaArgs;
    avatarId = a;
    ownerId = b;
    atomSpace = as;
    sender = s;
    learning_time1 = lt1;
    learning_time2 = lt2;
    max_cycle = mc; 
}

HCTestAgent::~HCTestAgent()
{
}


void HCTestAgent::run()
{
    logger().fine("Executing HCTestAgent.");
    cycle++;
    if (cycle > max_cycle) { //timeout in case the test takes too long
        logger().error("Executing HCTestAgent.");
        exit(1);
    }
    // send the whole atomSpace to the LS
    switch (mode) {
    case HCT_IDLE:
        std::cout << "OAC IDLE" << std::endl;
        sleep(IDLE_TIME);
        break;
    case HCT_INIT:
        std::cout << "OAC INIT" << std::endl;
        sender->sendExemplar(schemaName, schemaArguments, ownerId, avatarId, *atomSpace);
        mode = HCT_WAIT1;
        break;
    case HCT_WAIT1:
        std::cout << "OAC WAIT1" << std::endl;
        sleep(learning_time1);
        std::cout << "OAC WAIT1 DONE" << std::endl;
        sender->sendCommand(config().get("TRY_SCHEMA_CMD"),
                            schemaName);
        mode = HCT_IDLE;
        break;

    case HCT_WAIT2:
        std::cout << "OAC WAIT2" << std::endl;
        sleep(WAIT2_TIME);
        std::cout << "OAC WAIT2 DONE" << std::endl;
        sender->sendExemplar(schemaName, schemaArguments, ownerId, avatarId, *atomSpace);
        mode = HCT_WAIT3;
        break;

    case HCT_WAIT3:
        std::cout << "OAC WAIT3" << std::endl;
        sleep(learning_time2);
        std::cout << "OAC WAIT3 DONE" << std::endl;
        sender->sendCommand(config().get("TRY_SCHEMA_CMD"),
                            schemaName);
        mode = HCT_IDLE;
        break;

    case HCT_WAIT4:
        std::cout << "OAC WAIT4" << std::endl;
        sleep(WAIT4_TIME);
        std::cout << "OAC WAIT4 DONE" << std::endl;
        sender->sendCommand(config().get("STOP_LEARNING_CMD"),
                            schemaName);
        mode = HCT_IDLE;
        break;

    default:
        break;
    }
}
