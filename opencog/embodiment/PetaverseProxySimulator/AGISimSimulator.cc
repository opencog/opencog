/*
 * opencog/embodiment/PetaverseProxySimulator/AGISimSimulator.cc
 *
 * Copyright (C) 2007-2008 Welter Luigi
 * All Rights Reserved
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

#include "AGISimSimulator.h"
#include <opencog/util/Logger.h>
#include <opencog/util/numeric.h>

using namespace PetaverseProxySimulator;
using namespace PerceptionActionInterface;
using namespace opencog;

AGISimSimulator::AGISimSimulator(const char* _host, AsynchronousPerceptionAndStatusHandler* handler) :
        host(_host), petPerceptionHandler(handler)
{
    nextPort = 40001;
    petSimProxy = NULL;
}

AGISimSimulator::~AGISimSimulator()
{
    for (AgentToSimProxyMap::iterator it = simProxyMap.begin(); it != simProxyMap.end(); it++) {
        delete it->second;
    }
}

std::string AGISimSimulator::createAgent(const std::string& name, const std::string& type, float x, float y, bool echoing)
{
    logger().log(opencog::Logger::DEBUG, "AGISimSimulator:createAgent(%s, %s, %d)", name.c_str(), type.c_str(), echoing);
    SimProxy* simProxy = new SimProxy(host, nextPort, type == "pet" ? petPerceptionHandler : new DefaultPerceptionAndStatusHandler(), echoing);
    std::string result;
    if (simProxy->connect()) {
        logger().log(opencog::Logger::DEBUG, "AGISimSimulator: connected to the server at host %s, port %d", host.c_str(), nextPort);
        // Create the agent
        result = simProxy->newAgent(x, 0, y, type.c_str(), "ball", type == "pet" ? 0.25f : 0.30f, name.c_str());
        if (result.length()) {
            if (result != name) {
                logger().log(opencog::Logger::WARN, "AGISimSimulator: expected to create an agent with name '%s', but got '%s'. This indicates there was already another object with the expected name in AGISimSim world!", name.c_str(),  result.c_str());
            } else {
                logger().log(opencog::Logger::DEBUG, "AGISimSimulator: Created agent with name = '%s' in AGISimSim server", result.c_str());
            }
            simProxyMap[result] = simProxy;
            if (type == "pet") petSimProxy = simProxy;
            nextPort++;
        } else {
            logger().log(opencog::Logger::ERROR, "AGISimSimulator: Failed to create agent in AGISimSim server");
            delete simProxy;
        }
    } else {
        logger().log(opencog::Logger::ERROR, "AGISimSimulator: Could not connect to AGISimSim server");
    }
    return result;
}

SimProxy* AGISimSimulator::getConnectedSimProxy(const std::string& agentName)
{
    logger().log(opencog::Logger::DEBUG, "AGISimSimulator::getConnectedSimProxy(%s)", agentName.c_str());
    SimProxy* result = NULL;
    AgentToSimProxyMap::iterator it = simProxyMap.find(agentName);
    if (it == simProxyMap.end()) {
        logger().log(opencog::Logger::ERROR, "AGISimSimulator: Found no SimProxy for the given agent name");
    } else {
        SimProxy* simProxy = it->second;
        if (!simProxy->IsConnected()) {
            logger().log(opencog::Logger::ERROR, "AGISimSimulator: SimProxy is not connected to AGISimSim server");
        } else {
            result = simProxy;
        }
    }
    return result;
}

void AGISimSimulator::timeTick()
{
    logger().log(opencog::Logger::DEBUG, "AGISimSimulator::timeTick()");
    if (petSimProxy) {
        if (!petSimProxy->getCurrentSense()) {
            logger().log(opencog::Logger::ERROR, "AGISimSimulator: Failed to send 'sense' command to AGISimSim server");
        }
    } else {
        logger().log(opencog::Logger::WARN, "AGISimSimulator: 'sense' command not sent to AGISimSim server, since pet's SimProxy is not set yet");
    }
}

unsigned long AGISimSimulator::executeAction(const std::string& agentName, const PetAction &petAction)
{
    unsigned long result = ULONG_MAX; // error
    logger().log(opencog::Logger::DEBUG, "AGISimSimulator:: executing action for agent '%s': %s",
                 agentName.c_str(), petAction.stringRepresentation().c_str());

    SimProxy* simProxy = getConnectedSimProxy(agentName);
    if (simProxy) {
        // For movement actions, build the agisim command, get a new ticket for it and send it to AGISimSim server
        // For all other known actions, just return 0
        // For unknown actions, also return 0, but generate a warning log message.
        switch (petAction.getType().getCode()) {
        case WALK_CODE: { // walk(Vector target, float speed[, Rotation rotate])
            const ActionParameter& targetParam = petAction.getParameters().front();
            Vector v = targetParam.getVectorValue();
            // NOTE: do not consider the other parameters by now
            result = simProxy->walkTowards(v.x, v.y);
            break;
        }
        case GRAB_CODE: { // grab(EntityID  id, [float range-radius])
            const ActionParameter& idParam = petAction.getParameters().front();
            // TODO: handle the optional "range-radius" parameter, if present
            result = simProxy->lift(idParam.getEntityValue().id);
            break;
        }
        case DROP_CODE: { // drop()
            result = simProxy->drop();
            break;
        }
        case TURN_CODE: { // turn(Rotation rotate)
            // For now, considers only yaw rotation
            const ActionParameter& rotParam = petAction.getParameters().front();
            result = simProxy->turnRight(rotParam.getRotationValue().yaw * PI / 360);
            break;
        }
        case JUMP_TOWARD_CODE: { // jumpToward(Vector position)
            const ActionParameter& targetParam = petAction.getParameters().front();
            Vector v = targetParam.getVectorValue();
            // NOTE: Actually this only works fine when collision detection is disabled.
            result = simProxy->walkTowards(v.x, v.y, 1.0);
            break;
        }
        case NUDGE_TO_CODE: { // nudgeTo(EntityID moveableObj, Vector target)
            // This function allows the pet to nudge or push an object to a specified target
            const ActionParameter& idParam = petAction.getParameters().front();
            const Vector& vectorParam = petAction.getParameters().back().getVectorValue();
            result = simProxy->nudgeTo(idParam.getEntityValue().id, vectorParam.x, vectorParam.y);
            break;
        }
        case BARE_TEETH_CODE: { // bareTeeth([EntityID id [, float duration]])
            // If the 2 optional parameters are specified, get the latest one and implement this as turnTo
            if (petAction.getParameters().size() == 2) {
                const ActionParameter& idParam = petAction.getParameters().front();
                result = simProxy->turnTo(idParam.getEntityValue().id);
            } else {
                result = 0;
            }
            break;
        }
        case GROWL_CODE: { // growl([EntityID id [, float duration]])
            // If the 2 optional parameters are specified, get the latest one and implement this as turnTo
            if (petAction.getParameters().size() == 2) {
                const ActionParameter& idParam = petAction.getParameters().front();
                result = simProxy->turnTo(idParam.getEntityValue().id);
            } else {
                result = 0;
            }
            break;
        }
        case EAT_CODE: // void eat(EntityID id [, short quantity])
        case DRINK_CODE: { // void drink(EntityID id [, short quantity])
            if (petAction.getParameters().size() > 0) {
                const ActionParameter& idParam = petAction.getParameters().front();
                float quantity = -1; // special flag that means "default quantity"
                if (petAction.getParameters().size() > 1) {
                    const ActionParameter& quantityParam = petAction.getParameters().back();
                    quantity = atof(quantityParam.getStringValue().c_str());
                }
                if (petAction.getType().getCode() == EAT_CODE) {
                    result = simProxy->eat(idParam.getEntityValue().id, quantity);
                } else {
                    result = simProxy->drink(idParam.getEntityValue().id, quantity);
                }
            } else {
                result = 0;
            }
            break;
        }

        case BARK_CODE:
        case TRICK_FOR_FOOD_CODE:
        case SIT_CODE:
        case LIE_DOWN_CODE:
        case FLY_CODE:
        case FLY_FOLLOW_CODE:
        case STRETCH_CODE:
        case SCRATCH_SELF_NOSE_CODE:
        case SCRATCH_SELF_RIGHT_EAR_CODE:
        case SCRATCH_SELF_LEFT_EAR_CODE:
        case SCRATCH_SELF_NECK_CODE:
        case SCRATCH_SELF_RIGHT_SHOULDER_CODE:
        case SCRATCH_SELF_LEFT_SHOULDER_CODE:
        case SCRATCH_GROUND_BACK_LEGS_CODE:
        case RUN_IN_CIRCLE_CODE:
        case ANTICIPATE_PLAY_CODE:
        case BEG_CODE:
        case HEEL_CODE:
        case HIDE_FACE_CODE:
        case PLAY_DEAD_CODE:
        case FOLLOW_CODE:
        case LICK_CODE:
        case TAP_DANCE_CODE:
        case LOOK_UP_TURN_HEAD_CODE:
        case WHINE_CODE:
        case SNIFF_CODE:
        case SNIFF_AT_CODE:
        case SNIFF_PET_PART_CODE:
        case SNIFF_AVATAR_PART_CODE:
        case SHAKE_HEAD_CODE:
        case EARS_BACK_CODE:
        case EARS_TWITCH_CODE:
        case MOVE_HEAD_CODE:
        case WAKE_CODE:
        case SLEEP_CODE:
        case PEE_CODE:
        case POO_CODE:
        case WAG_CODE:
        case TAIL_FLEX_CODE:
        case CHEW_CODE:
        case DREAM_CODE:
        case SCRATCH_OTHER_CODE:
        case EARS_PERK_CODE:
        case JUMP_UP_CODE:
        case PAY_ATTENTION_CODE:
        case VOMIT_CODE:
        case LEAN_ROCK_DANCE_CODE:
        case BACK_FLIP_CODE:
        case WIDEN_EYES_CODE:
        case FEARFUL_POSTURE_CODE:
        case CLEAN_CODE:
        case BELCH_CODE:
        case GREET_CODE:
        case DANCE1_CODE:
        case LOOK_RIGHT_CODE:
        case LOOK_LEFT_CODE:
        case KICK_LEFT_CODE:
        case KICK_RIGHT_CODE:
        case LEFT_EAR_PERK_CODE:
        case RIGHT_EAR_PERK_CODE:
        case LEFT_EAR_BACK_CODE:
        case RIGHT_EAR_BACK_CODE:
        case LEFT_EAR_TWITCH_CODE:
        case RIGHT_EAR_TWITCH_CODE:
        case ANGRY_EYES_CODE:
        case SAD_EYES_CODE:
        case HAPPY_EYES_CODE:
        case CLOSE_EYES_CODE:
        case BITE_CODE:
            logger().log(opencog::Logger::DEBUG, "AGISimSimulator will not execute the action in the simulated world, since it does not affect any spatial info!");
            result = 0;
            break;
        default:
            logger().log(opencog::Logger::WARN, "AGISimSimulator does not know this action!");
            break;
        }
    }
    return result;
}

