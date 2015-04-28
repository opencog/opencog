/*
 * opencog/embodiment/Control/PerceptionActionInterface/ActionType.h
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

#ifndef _ACTION_TYPE_H_
#define _ACTION_TYPE_H_
/**
 * ActionType.h
 *
 * Define all types of pet actions
 *
 */

#include <string>
#include <map>
#include <sstream>
#include <iostream>
#include <vector>

#include "ActionParamType.h"

namespace opencog { namespace pai {

/**
 * Numeric codes for the action types
 */
enum ActionTypeCode {
    EAT_CODE,
    WALK_CODE,
    MOVE_TO_OBJ_CODE,
    GRAB_CODE,
    DROP_CODE,
    SIT_CODE,
    FLY_CODE,
    FLY_FOLLOW_CODE,
    FOLLOW_CODE,
    NUDGE_TO_CODE,
    MOVE_HEAD_CODE,
    WAKE_CODE,
    SLEEP_CODE,
    DRINK_CODE,
    TURN_CODE,
    ROTATE_CODE, // only turn around along the vertical axis.
    JUMP_UP_CODE,
    JUMP_TOWARD_CODE,
    JUMP_FORWARD_CODE,
    PAY_ATTENTION_CODE,
    WIDEN_EYES_CODE,
    LOOK_RIGHT_CODE,
    LOOK_LEFT_CODE,
    KICK_LEFT_CODE,
    KICK_RIGHT_CODE,
    ANGRY_EYES_CODE,
    SAD_EYES_CODE,
    HAPPY_EYES_CODE,
    CLOSE_EYES_CODE,
    KICK_CODE,

    GROUP_COMMAND_CODE,
    RECEIVE_LATEST_GROUP_COMMANDS_CODE,
    LOOK_AT_CODE,
    SAY_CODE,

    BUILD_BLOCK_CODE,
    DESTROY_BLOCK_CODE,
    DO_NOTHING_CODE,

    // For Santa Fe Trail problem
    STEP_FORWARD_CODE,
    ROTATE_LEFT_CODE,
    ROTATE_RIGHT_CODE,

    // This is not really a type and must be at the end of that enumeration in order to count the number of action types.
    NUMBER_OF_ACTION_TYPES
};


/**
 * ActionType class. Wrappers both primitive's code and name in an object
 */
class ActionType
{

public:

    typedef std::vector<ActionParamType> ParamTypes;

    // Object methods
    const std::string& getName() const;
    ActionTypeCode getCode() const;

    const ParamTypes& getMandatoryParamTypes() const;
    const ParamTypes& getOptionalParamTypes() const;

    const unsigned int getMandatoryParamSize() const;
    const unsigned int getOptionalParamSize() const;

    const char** getParamNames() const;
    std::string getHelpText() const;

    bool operator==(const ActionType& other) const;
    bool operator!=(const ActionType& other) const;

    // Class methods
    static void init();
    static const ActionType& getFromName(const std::string& name);
    static const ActionType& getFromCode(ActionTypeCode code);
    static void printHelp();

    static const ActionType& EAT();
    /**
     * void walk(Vector target, float speed[, Rotation frameOfReference])
     */
    static const ActionType& WALK();
    /**
     * void move_to_obj(EntityID target)
     */
    static const ActionType& MOVE_TO_OBJ();
    /**
     * Status grab(EntityID target [, float rangeRadius])
     */
    static const ActionType& GRAB();
    /**
     * void drop()
     */
    static const ActionType& DROP();
    /**
     * void sit([float duration])
     */
    static const ActionType& SIT();
    /**
     * void lieDown([float duration])
     */
    static const ActionType& FLY();
    /**
     * void flyFollow(EntityID target)
     */
    static const ActionType& FLY_FOLLOW();
    /**
     * void stretch()
     */
    static const ActionType& FOLLOW();
    /**
     * void lick([EntityID target])
     */
    static const ActionType& NUDGE_TO();
    /**
     * void move_head(Vector toPosition, Rotation rotate, float speed)
     */
    static const ActionType& MOVE_HEAD();
    /**
     * void wake()
     */
    static const ActionType& WAKE();
    /**
     * void sleep([float duration])
     */
    static const ActionType& SLEEP();
    /**
     * void drink(EntityID target [, float quantity])
     */
    static const ActionType& DRINK();
    /**
     * void pee()
     */
    static const ActionType& TURN();
    /**
     * void rotate(Rotation direction)
     */
    static const ActionType& ROTATE();
    /**
     * void scratchOther(EntityID target [, float duration])
     */
    static const ActionType& JUMP_UP();
    /**
     * void jumpToward(Vector direction)
     */
    static const ActionType& JUMP_TOWARD();
    /**
     * void jumpForward(Vector direction)
     */
    static const ActionType& JUMP_FORWARD();

    /**
     * void PayAttention()
     */
    static const ActionType& PAY_ATTENTION();
    /**
     * void widen_eyes()
     */
    static const ActionType& WIDEN_EYES();
    /**
     * void lookRight([float duration])
     */
    static const ActionType& LOOK_RIGHT();
    /**
     * void lookLeft([float duration])
     */
    static const ActionType& LOOK_LEFT();
    /**
     * void kickLeft([float duration])
     */
    static const ActionType& KICK_LEFT();
    /**
     * void kickRight([float duration])
     */
    static const ActionType& KICK_RIGHT();
    /**
     * void leftEarPerk([float duration])
     */
    static const ActionType& LEFT_EAR_PERK();
    /**
     * void rightEarPerk([float duration])
     */
    static const ActionType& RIGHT_EAR_PERK();
    /**
     * void angryEyes([float duration])
     */
    static const ActionType& ANGRY_EYES();
    /**
     * void sadEyes([float duration])
     */
    static const ActionType& SAD_EYES();
    /**
     * void happyEyes([float duration])
     */
    static const ActionType& HAPPY_EYES();
    /**
     * void closeEyes([float duration])
     */
    static const ActionType& CLOSE_EYES();
    /**
     * void kick(EntityID target)
     */
    static const ActionType& KICK();
    /**
     * void group_command(string command [, EntityID target])
     */
    static const ActionType& GROUP_COMMAND();
    /**
     * void receive_latest_group_commands()
     */
    static const ActionType& RECEIVE_LATEST_GROUP_COMMANDS();
    /**
     * void look_at( EntityID target )
     */
    static const ActionType& LOOK_AT();
    /**
     * void say( string message, string target )
     */
    static const ActionType& SAY();
    /**
     * void build_block( float offset, string texture )
     */
    static const ActionType& BUILD_BLOCK();
    /**
     * void destroy_block( Vector position )
     */
    static const ActionType& DESTROY_BLOCK();

    /**
     * void step_forward()
     */
    static const ActionType& STEP_FORWARD();

    /**
     * void rotate_right()
     */
    static const ActionType& ROTATE_RIGHT();

    /**
     * void rotate_left()
     */
    static const ActionType& ROTATE_LEFT();

    /**
     * void do_nothing( )  // this is using in planning
     */
    static const ActionType& DO_NOTHING();

private:
    // Attributes
    ActionTypeCode code;
    std::string name;
    ParamTypes mandatoryParamTypes;
    ParamTypes optionalParamTypes;
    const char** paramNames;
    std::string helpText;

    /**
     * Empty constructor
     */
    ActionType();

    /**
     * Constructor
     */
    ActionType(ActionTypeCode _code,
               const std::string& _name,
               const ParamTypes& _mandatoryParamTypes,
               const ParamTypes& _optionalParamTypes,
               const char** paramNames,
               const std::string& _helpText);

    // Static data structure
    typedef std::map<std::string, ActionType*> Name2ActionTypeMap;
    typedef std::map<ActionTypeCode, ActionType*> Code2ActionTypeMap;
    static Name2ActionTypeMap nameMap;
    static Code2ActionTypeMap codeMap;

    // private static methods
    static bool existName(const std::string& name);
    static bool existCode(ActionTypeCode code);
    static void initParamTypes();

};

std::ostream& operator<<(std::ostream& out, const ActionType& arg);

} } // namespace opencog::pai


#endif //_ACTION_TYPE_H_
