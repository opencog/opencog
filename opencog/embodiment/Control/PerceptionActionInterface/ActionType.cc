/*
 * opencog/embodiment/Control/PerceptionActionInterface/ActionType.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#include "ActionType.h"
#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/Logger.h>
#include <iostream>

using namespace opencog::pai;

/*====================================*
 * Static data members declaration    *
 * ===================================*/

// Arity 0
static ActionType::ParamTypes EMPTY;

// Arity 1
static ActionType::ParamTypes BOOLEAN;
static ActionType::ParamTypes INT;
static ActionType::ParamTypes FLOAT;
static ActionType::ParamTypes STRING;
static ActionType::ParamTypes VECTOR;
static ActionType::ParamTypes ROTATION;
static ActionType::ParamTypes ENTITY;

// Arity 2
static ActionType::ParamTypes FLOAT_and_ROTATION;
static ActionType::ParamTypes VECTOR_and_FLOAT;
static ActionType::ParamTypes ENTITY_and_INT;
static ActionType::ParamTypes ENTITY_and_FLOAT;
static ActionType::ParamTypes ENTITY_and_VECTOR;
static ActionType::ParamTypes STRING_and_STRING;
static ActionType::ParamTypes FLOAT_and_FLOAT;
static ActionType::ParamTypes FLOAT_and_STRING;
static ActionType::ParamTypes VECTOR_and_STRING;

// Arity 3
static ActionType::ParamTypes VECTOR_ROTATION_and_FLOAT;


/*====================================*
 * Static data members initialization *
 * ===================================*/
void ActionType::initParamTypes()
{
    static bool initialized = false;
    if (!initialized) {
        // Arity 1
        BOOLEAN.push_back(ActionParamType::BOOLEAN());
        INT.push_back(ActionParamType::INT());
        STRING.push_back(ActionParamType::STRING());
        ROTATION.push_back(ActionParamType::ROTATION());
        VECTOR.push_back(ActionParamType::VECTOR());
        ENTITY.push_back(ActionParamType::ENTITY());
        FLOAT.push_back(ActionParamType::FLOAT());

        // Arity 2
        FLOAT_and_ROTATION.push_back(ActionParamType::FLOAT());
        FLOAT_and_ROTATION.push_back(ActionParamType::ROTATION());
        VECTOR_and_FLOAT.push_back(ActionParamType::VECTOR());
        VECTOR_and_FLOAT.push_back(ActionParamType::FLOAT());
        ENTITY_and_INT.push_back(ActionParamType::ENTITY());
        ENTITY_and_INT.push_back(ActionParamType::INT());
        ENTITY_and_FLOAT.push_back(ActionParamType::ENTITY());
        ENTITY_and_FLOAT.push_back(ActionParamType::FLOAT());
        ENTITY_and_VECTOR.push_back(ActionParamType::ENTITY());
        ENTITY_and_VECTOR.push_back(ActionParamType::VECTOR());
        STRING_and_STRING.push_back(ActionParamType::STRING());
        STRING_and_STRING.push_back(ActionParamType::STRING());
        FLOAT_and_FLOAT.push_back(ActionParamType::FLOAT());
        FLOAT_and_FLOAT.push_back(ActionParamType::FLOAT());
        FLOAT_and_STRING.push_back(ActionParamType::FLOAT());
        FLOAT_and_STRING.push_back(ActionParamType::STRING());
        VECTOR_and_STRING.push_back(ActionParamType::VECTOR());
        VECTOR_and_STRING.push_back(ActionParamType::STRING());

        // Arity 3
        VECTOR_ROTATION_and_FLOAT.push_back(ActionParamType::VECTOR());
        VECTOR_ROTATION_and_FLOAT.push_back(ActionParamType::ROTATION());
        VECTOR_ROTATION_and_FLOAT.push_back(ActionParamType::FLOAT());

        initialized = true;
    }
}

// Internal maps
ActionType::Name2ActionTypeMap ActionType::nameMap;
ActionType::Code2ActionTypeMap ActionType::codeMap;

// Definition of all action type values
const ActionType& ActionType::EAT()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "quantity"};
    static ActionType* result = new ActionType(EAT_CODE, "eat", ENTITY, FLOAT, paramNames, "void eat(EntityID target [, float quantity])");
    return *result;
}
const ActionType& ActionType::WALK()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "speed", "frameOfReference"};
    static ActionType* result = new ActionType(WALK_CODE, "walk", VECTOR_and_FLOAT, ROTATION, paramNames, "void walk(Vector target, float speed[, Rotation frameOfReference])");
    return *result;
}

const ActionType& ActionType::MOVE_TO_OBJ()
{
    initParamTypes();
    static const char* paramNames[] = {"target"};
    static ActionType* result = new ActionType(MOVE_TO_OBJ_CODE, "move_to_obj", ENTITY, ROTATION, paramNames, "void move_to_obj(EntityID target)");
    return *result;
}

const ActionType& ActionType::GRAB()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "rangeRadius"};
    static ActionType* result = new ActionType(GRAB_CODE, "grab", ENTITY, FLOAT, paramNames, "Status grab(EntityID target [, float rangeRadius])");
    return *result;
}
const ActionType& ActionType::DROP()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(DROP_CODE, "drop", EMPTY, EMPTY, paramNames, "void drop()");
    return *result;
}
const ActionType& ActionType::SIT()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SIT_CODE, "sit", EMPTY, FLOAT, paramNames, "void sit([float duration])");
    return *result;
}
const ActionType& ActionType::FLY()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "speed", "frameOfReference"};
    static ActionType* result = new ActionType(FLY_CODE, "fly", VECTOR, FLOAT_and_ROTATION, paramNames, "void fly(Vector target [, float speed, Rotation frameOfReference])");
    return *result;
}
const ActionType& ActionType::FLY_FOLLOW()
{
    initParamTypes();
    static const char* paramNames[] = {"target"};
    static ActionType* result = new ActionType(FLY_FOLLOW_CODE, "fly_follow", ENTITY, EMPTY, paramNames, "void fly_follow(EntityID target)");
    return *result;
}
const ActionType& ActionType::FOLLOW()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "duration"};
    static ActionType* result = new ActionType(FOLLOW_CODE, "follow", ENTITY, FLOAT, paramNames, "void follow(EntityID target[, float duration])");
    return *result;
}
const ActionType& ActionType::NUDGE_TO()
{
    initParamTypes();
    static const char* paramNames[] = {"moveableObj", "target"};
    static ActionType* result = new ActionType(NUDGE_TO_CODE, "nudge_to", ENTITY_and_VECTOR, EMPTY, paramNames, "void nudge(EntityID moveableObj, Vector target)");
    return *result;
}
const ActionType& ActionType::MOVE_HEAD()
{
    initParamTypes();
    static const char* paramNames[] = {"toPosition", "rotate", "speed"};
    static ActionType* result = new ActionType(MOVE_HEAD_CODE, "move_head", VECTOR_ROTATION_and_FLOAT, EMPTY, paramNames, "void moveHead(Vector toPosition, Rotation rotate, float speed)");
    return *result;
}
const ActionType& ActionType::WAKE()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(WAKE_CODE, "wake", EMPTY, EMPTY, paramNames, "void wake()");
    return *result;
}
const ActionType& ActionType::SLEEP()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SLEEP_CODE, "sleep", EMPTY, FLOAT, paramNames, "void sleep([float duration])");
    return *result;
}
const ActionType& ActionType::DRINK()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "quantity"};
    static ActionType* result = new ActionType(DRINK_CODE, "drink", ENTITY, FLOAT, paramNames, "void drink(EntityID target [, float quantity])");
    return *result;
}
const ActionType& ActionType::TURN()
{
    initParamTypes();
    static const char* paramNames[] = {"direction"};
    static ActionType* result = new ActionType(TURN_CODE, "turn", ROTATION, EMPTY, paramNames, "void turn(Rotation direction)");
    return *result;
}
const ActionType& ActionType::ROTATE()
{
    initParamTypes();
    // To be convention, the angle is in the range [-180, 180], where negative
    // angle means rotating left, while positive one means rotating right.
    static const char* paramNames[] = {"angle"};
    static ActionType* result = new ActionType(ROTATE_CODE, "rotate", FLOAT, EMPTY, paramNames, "void rotate(float angle)");
    return *result;
}
const ActionType& ActionType::WIDEN_EYES()
{
    initParamTypes();
    static const char* paramNames[] = {"interval"};
    static ActionType* result = new ActionType(WIDEN_EYES_CODE, "widen_eyes", EMPTY, INT, paramNames, "void widenEyes([short interval])");
    return *result;
}
const ActionType& ActionType::JUMP_UP()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(JUMP_UP_CODE, "jump_up", EMPTY, EMPTY, paramNames, "void jump_up()");
    return *result;
}
const ActionType& ActionType::JUMP_TOWARD()
{
    initParamTypes();
    static const char* paramNames[] = {"direction"};
    static ActionType* result = new ActionType(JUMP_TOWARD_CODE, "jump_toward", VECTOR, EMPTY, paramNames, "void jumpToward(Vector direction)");
    return *result;
}
const ActionType& ActionType::JUMP_FORWARD()
{
    initParamTypes();
    static const char* paramNames[] = {"height"};
    static ActionType* result = new ActionType(JUMP_FORWARD_CODE, "jump_forward", FLOAT, EMPTY, paramNames, "void jumpForward(float height)");
    return *result;
}
const ActionType& ActionType::PAY_ATTENTION()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(PAY_ATTENTION_CODE, "pay_attention", EMPTY, EMPTY, paramNames, "void payAttention()");
    return *result;
}
const ActionType& ActionType::LOOK_RIGHT()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(LOOK_RIGHT_CODE, "look_right", EMPTY, FLOAT, paramNames, "void lookRight([float duration])");
    return *result;
}
const ActionType& ActionType::LOOK_LEFT()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(LOOK_LEFT_CODE, "look_left", EMPTY, FLOAT, paramNames, "void lookLeft([float duration])");
    return *result;
}
const ActionType& ActionType::KICK_LEFT()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(KICK_LEFT_CODE, "kick_left", EMPTY, FLOAT, paramNames, "void kickLeft([float duration])");
    return *result;
}
const ActionType& ActionType::KICK_RIGHT()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(KICK_RIGHT_CODE, "kick_right", EMPTY, FLOAT, paramNames, "void kickRight([float duration])");
    return *result;
}
const ActionType& ActionType::ANGRY_EYES()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(ANGRY_EYES_CODE, "angry_eyes", EMPTY, FLOAT, paramNames, "void angryEyes([float duration])");
    return *result;
}
const ActionType& ActionType::SAD_EYES()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SAD_EYES_CODE, "sad_eyes", EMPTY, FLOAT, paramNames, "void sad_eyes([float duration])");
    return *result;
}
const ActionType& ActionType::HAPPY_EYES()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(HAPPY_EYES_CODE, "happy_eyes", EMPTY, FLOAT, paramNames, "void happyEyes([float duration])");
    return *result;
}
const ActionType& ActionType::CLOSE_EYES()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(CLOSE_EYES_CODE, "close_eyes", EMPTY, FLOAT, paramNames, "void closeEyes([float duration])");
    return *result;
}

const ActionType& ActionType::KICK()
{
    initParamTypes();
    static const char* paramNames[] = {"target"};
    static ActionType* result = new ActionType(KICK_CODE, "kick", ENTITY, EMPTY, paramNames, "void kick(EntityID target)");
    return *result;
}

const ActionType& ActionType::GROUP_COMMAND()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "command", "parameters"};
    static ActionType* result = new ActionType(GROUP_COMMAND_CODE, "group_command", STRING, STRING_and_STRING, paramNames, "void group_command(string target, string command [, string parameters])");
    return *result;
}

const ActionType& ActionType::RECEIVE_LATEST_GROUP_COMMANDS()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(RECEIVE_LATEST_GROUP_COMMANDS_CODE, "receive_latest_group_commands", EMPTY, EMPTY, paramNames, "void receive_latest_group_commands()");
    return *result;
}

const ActionType& ActionType::LOOK_AT()
{
    initParamTypes();
    static const char* paramNames[] = {"target"};
    static ActionType* result = new ActionType(LOOK_AT_CODE, "look_at", ENTITY, EMPTY, paramNames, "void look_at( EntityID target )");
    return *result;
}

const ActionType& ActionType::SAY()
{
    initParamTypes();
    static const char* paramNames[] = {"message", "target"};
    static ActionType* result = new ActionType(SAY_CODE, "say", STRING, STRING, paramNames, "void say( string message, string target )");
    return *result;
}

const ActionType& ActionType::BUILD_BLOCK()
{
    initParamTypes();
    static const char* paramNames[] = {"position", "blockType"};
    static ActionType* result = new ActionType(BUILD_BLOCK_CODE, "build_block", VECTOR_and_STRING, EMPTY, paramNames, "void build_block(Vector position[, int direction])");
    return *result;
}

const ActionType& ActionType::DESTROY_BLOCK()
{
    initParamTypes();
    static const char* paramNames[] = {"position"};
    static ActionType* result = new ActionType(DESTROY_BLOCK_CODE, "destroy_block", VECTOR, EMPTY, paramNames, "void destroy_block(Vector position)");
    return *result;
}

const ActionType& ActionType::STEP_FORWARD()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(STEP_FORWARD_CODE, "step_forward", EMPTY, EMPTY, paramNames, "void step_forward() (C++ to help humans, WTF)");
    return *result;
}

const ActionType& ActionType::ROTATE_LEFT()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(ROTATE_LEFT_CODE, "rotate_left", EMPTY, EMPTY, paramNames, "rotate_left :: IO ()");
    return *result;
}

const ActionType& ActionType::ROTATE_RIGHT()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(ROTATE_RIGHT_CODE, "rotate_right", EMPTY, EMPTY, paramNames, "rotate_right :: IO ()");
    return *result;
}

const ActionType& ActionType::DO_NOTHING()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(DO_NOTHING_CODE, "do_nothing", EMPTY, EMPTY, paramNames, "void do_nothing()");
    return *result;
}

#if 0
// TEMPLATE FOR ADDING OTHER TYPES ABOVE
const ActionType& ActionType::ACTION()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(ACTION_CODE, "action", EMPTY, EMPTY, paramNames, "");
    return *result;
}
#endif

// Constructors

ActionType::ActionType() {};

ActionType::ActionType(ActionTypeCode _code,
                       const std::string& _name,
                       const ParamTypes& _mandatoryParamTypes,
                       const ParamTypes& _optionalParamTypes,
                       const char** _paramNames,
                       const std::string& _helpText):
        code(_code), name(_name),
        mandatoryParamTypes(_mandatoryParamTypes),
        optionalParamTypes(_optionalParamTypes),
        paramNames(_paramNames),
        helpText(_helpText)
{
    if (ActionType::existCode(code)) {
        OC_ASSERT(false, "ActionType - Duplicate action type code definition: %d.", _code);
    }
    if (ActionType::existName(name)) {
        OC_ASSERT(false, "ActionType - Duplicate action type name definition: %s.", _name.c_str());
    }
    nameMap[name] = this;
    codeMap[code] = this;
}

// GETTERS

const std::string& ActionType::getName() const
{
    return name;
}

ActionTypeCode ActionType::getCode() const
{
    return code;
}

const ActionType::ParamTypes& ActionType::getMandatoryParamTypes() const
{
    return mandatoryParamTypes;
}

const ActionType::ParamTypes& ActionType::getOptionalParamTypes() const
{
    return optionalParamTypes;
}

const unsigned int ActionType::getMandatoryParamSize() const
{
    return mandatoryParamTypes.size();
}

const unsigned int ActionType::getOptionalParamSize() const
{
    return optionalParamTypes.size();
}

const char** ActionType::getParamNames() const
{
    return paramNames;
}

std::string ActionType::getHelpText() const
{
    return helpText;
}

bool ActionType::operator==(const ActionType& other) const
{
    return (code == other.code);
}

bool ActionType::operator!=(const ActionType& other) const
{
    return (code != other.code);
}


// STATIC METHODS:

void ActionType::init()
{
    static bool initialized = false;
    if (!initialized) {
        EAT();
        WALK();
        MOVE_TO_OBJ();
        GRAB();
        DROP();
        SIT();
        FLY();
        FLY_FOLLOW();
        FOLLOW();
        NUDGE_TO();
        MOVE_HEAD();
        WAKE();
        SLEEP();
        DRINK();
        TURN();
        ROTATE();
        JUMP_UP();
        JUMP_TOWARD();
        JUMP_FORWARD();
        PAY_ATTENTION();
        WIDEN_EYES();
        LOOK_RIGHT();
        LOOK_LEFT();
        KICK_LEFT();
        KICK_RIGHT();
        ANGRY_EYES();
        SAD_EYES();
        HAPPY_EYES();
        CLOSE_EYES();
        KICK();
        GROUP_COMMAND();
        RECEIVE_LATEST_GROUP_COMMANDS();
        LOOK_AT();
        SAY();

        BUILD_BLOCK();
        DESTROY_BLOCK();

        // For Santa Fe Trail problem
        STEP_FORWARD();
        ROTATE_LEFT();
        ROTATE_RIGHT();

        DO_NOTHING();

        initialized = true;
    }
}

const ActionType& ActionType::getFromName(const std::string& name)
{
    init();
    Name2ActionTypeMap::const_iterator itr = nameMap.find(name);
    if (itr == nameMap.end()) {
        OC_ASSERT(false, "ActionType - Trying to get an ActionType with an invalid/unknown name: %s.", name.c_str());
    }
    return *(itr->second);
}

const ActionType& ActionType::getFromCode(ActionTypeCode code)
{
    init();
    Code2ActionTypeMap::const_iterator itr = codeMap.find(code);
    if (itr == codeMap.end()) {
        OC_ASSERT(false, "ActionType - Trying to get an ActionType with an invalid/unknown code: %d.", code);
    }
    return *(itr->second);
}

bool ActionType::existName(const std::string& name)
{
    Name2ActionTypeMap::const_iterator itr = nameMap.find(name);
    return (itr != nameMap.end());
}

bool ActionType::existCode(ActionTypeCode code)
{
    Code2ActionTypeMap::const_iterator itr = codeMap.find(code);
    return (itr != codeMap.end());
}

void ActionType::printHelp()
{
    opencog::logger().info(
                          "==================================== HELP OF PET ACTION TYPES ========================================");
    for (Name2ActionTypeMap::const_iterator itr = nameMap.begin(); itr != nameMap.end(); ++itr) {
        const ActionType& t = *(itr->second);
        opencog::logger().info("    %s => %s", t.getName().c_str(), t.getHelpText().c_str());
    }
    opencog::logger().info(
                          "======================================================================================================");
}

std::ostream& operator<<(std::ostream& out, const ActionType& arg)
{
    return (out << arg.getName());
}
