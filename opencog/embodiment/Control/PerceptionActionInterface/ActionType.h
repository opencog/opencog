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
    BARK_CODE,
    TRICK_FOR_FOOD_CODE,
    EAT_CODE,
    WALK_CODE,
    MOVE_TO_OBJ_CODE,
    GRAB_CODE,
    DROP_CODE,
    SIT_CODE,
    LIE_DOWN_CODE,
    FLY_CODE,
    FLY_FOLLOW_CODE,
    STRETCH_CODE,
    SCRATCH_SELF_NOSE_CODE,
    SCRATCH_SELF_RIGHT_EAR_CODE,
    SCRATCH_SELF_LEFT_EAR_CODE,
    SCRATCH_SELF_NECK_CODE,
    SCRATCH_SELF_RIGHT_SHOULDER_CODE,
    SCRATCH_SELF_LEFT_SHOULDER_CODE,
    SCRATCH_GROUND_BACK_LEGS_CODE,
    RUN_IN_CIRCLE_CODE,
    ANTICIPATE_PLAY_CODE,
    BEG_CODE,
    HEEL_CODE,
    HIDE_FACE_CODE,
    PLAY_DEAD_CODE,
    FOLLOW_CODE,
    LICK_CODE,
    NUDGE_TO_CODE,
    TAP_DANCE_CODE,
    BARE_TEETH_CODE,
    GROWL_CODE,
    LOOK_UP_TURN_HEAD_CODE,
    WHINE_CODE,
    SNIFF_CODE,
    SNIFF_AT_CODE,
    SNIFF_PET_PART_CODE,
    SNIFF_AVATAR_PART_CODE,
    SHAKE_HEAD_CODE,
    EARS_BACK_CODE,
    EARS_TWITCH_CODE,
    MOVE_HEAD_CODE,
    WAKE_CODE,
    SLEEP_CODE,
    DRINK_CODE,
    PEE_CODE,
    POO_CODE,
    WAG_CODE,
    TAIL_FLEX_CODE,
    CHEW_CODE,
    DREAM_CODE,
    TURN_CODE,
    ROTATE_CODE, // only turn around along the vertical axis.
    SCRATCH_OTHER_CODE,
    EARS_PERK_CODE,
    JUMP_UP_CODE,
    JUMP_TOWARD_CODE,
    JUMP_FORWARD_CODE,
    PAY_ATTENTION_CODE,
    VOMIT_CODE,
    LEAN_ROCK_DANCE_CODE,
    BACK_FLIP_CODE,
    WIDEN_EYES_CODE,
    FEARFUL_POSTURE_CODE,
    CLEAN_CODE,
    BELCH_CODE,
    GREET_CODE,
    DANCE1_CODE,
    LOOK_RIGHT_CODE,
    LOOK_LEFT_CODE,
    KICK_LEFT_CODE,
    KICK_RIGHT_CODE,
    LEFT_EAR_PERK_CODE,
    RIGHT_EAR_PERK_CODE,
    LEFT_EAR_BACK_CODE,
    RIGHT_EAR_BACK_CODE,
    LEFT_EAR_TWITCH_CODE,
    RIGHT_EAR_TWITCH_CODE,
    ANGRY_EYES_CODE,
    SAD_EYES_CODE,
    HAPPY_EYES_CODE,
    CLOSE_EYES_CODE,
    BITE_CODE,
    PET_CODE,
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

    /**
     * void bark([EntityID target [, float duration]])
     */
    static const ActionType& BARK();
    /**
     * void trickForFood()
     */
    static const ActionType& TRICK_FOR_FOOD();
    /**
     * void eat(EntityID target [, float quantity])
     */
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
    static const ActionType& LIE_DOWN();
    /**
     * void fly(Vector target, float speed[, Rotation frameOfReference])
     */
    static const ActionType& FLY();
    /**
     * void flyFollow(EntityID target)
     */
    static const ActionType& FLY_FOLLOW();
    /**
     * void stretch()
     */
    static const ActionType& STRETCH();
    /**
     * void scratchSelfNose([float duration])
     */
    static const ActionType& SCRATCH_SELF_NOSE();
    /**
     * void scratchSelfRightEar([float duration])
     */
    static const ActionType& SCRATCH_SELF_RIGHT_EAR();
    /**
     * void scratchSelfLeftEar([float duration])
     */
    static const ActionType& SCRATCH_SELF_LEFT_EAR();
    /**
     * void scratchSelfNeck([float duration])
     */
    static const ActionType& SCRATCH_SELF_NECK();
    /**
     * void scratchSelfRightShoulder([float duration])
     */
    static const ActionType& SCRATCH_SELF_RIGHT_SHOULDER();
    /**
     * void scratchSelfLeftShoulder([float duration])
     */
    static const ActionType& SCRATCH_SELF_LEFT_SHOULDER();
    /**
     * void scratchGroundBackLegs([float duration])
     */
    static const ActionType& SCRATCH_GROUND_BACK_LEGS();
    /**
     * void runInCircle([float duration])
     */
    static const ActionType& RUN_IN_CIRCLE();
    /**
     * void anticipatePlay()
     */
    static const ActionType& ANTICIPATE_PLAY();
    /**
     * void beg()
     */
    static const ActionType& BEG();
    /**
     * void heel([float duration])
     */
    static const ActionType& HEEL();
    /**
     * void hideFace([float duratio])
     */
    static const ActionType& HIDE_FACE();
    /**
     * void playDead()
     */
    static const ActionType& PLAY_DEAD();
    /**
     * void follow(EntityID target[, float duration])
     */
    static const ActionType& FOLLOW();
    /**
     * void lick([EntityID target])
     */
    static const ActionType& LICK();
    /**
    * void nudgeTo(EntityID moveableObj, Vector target)
     */
    static const ActionType& NUDGE_TO();
    /**
     * void tapDance([float duration])
     */
    static const ActionType& TAP_DANCE();
    /**
     * void bareTeeth([EntityID target [, float duration]])
     */
    static const ActionType& BARE_TEETH();
    /**
     * void growl([EntityID target [, float duration]])
     */
    static const ActionType& GROWL();
    /**
     * void lookUpTurnHead([float duration])
     */
    static const ActionType& LOOK_UP_TURN_HEAD();
    /**
     * void whine([EntityID target [, float duration]])
     */
    static const ActionType& WHINE();
    /**
     * void sniff([EntityID target])
     */
    static const ActionType& SNIFF();
    /**
     * void sniffAt(EntityID target)
     */
    static const ActionType& SNIFF_AT();
    /**
     * void sniffPetPart(EntityID pet, integer part)
     */
    static const ActionType& SNIFF_PET_PART();
    /**
     * void sniffAvatarPart(EntityID avatar, integer part)
     */
    static const ActionType& SNIFF_AVATAR_PART();
    /**
     * void shakeHead([float duration])
     */
    static const ActionType& SHAKE_HEAD();
    /**
     * void earsBack([float duration])
     */
    static const ActionType& EARS_BACK();
    /**
     * void earTwitch([float duration])
     */
    static const ActionType& EARS_TWITCH();
    /**
     * void moveHead(Vector toPosition, Rotation rotate, float speed)
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
    static const ActionType& PEE();
    /**
     * void poo()
     */
    static const ActionType& POO();
    /**
     * void wag([float duration])
     */
    static const ActionType& WAG();
    /**
     * void tailFlex(Vector position [, float duration])
     */
    static const ActionType& TAIL_FLEX();
    /**
     * void chew([EntityID id])
     */
    static const ActionType& CHEW();
    /**
     * void dream(EntityID id)
     */
    static const ActionType& DREAM();
    /**
     * void turn(Rotation direction)
     */
    static const ActionType& TURN();
    /**
     * void rotate(Rotation direction)
     */
    static const ActionType& ROTATE();
    /**
     * void scratchOther(EntityID target [, float duration])
     */
    static const ActionType& SCRATCH_OTHER();
    /**
     * void earsPerk([float duration])
     */
    static const ActionType& EARS_PERK();
    /**
     * void jump_up()
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
     * void vomit()
     */
    static const ActionType& VOMIT();
    /**
     * void leanRockDance()
     */
    static const ActionType& LEAN_ROCK_DANCE();
    /**
     * void backFlip()
     */
    static const ActionType& BACK_FLIP();
    /**
     * void widenEyes([short interval])
     */
    static const ActionType& WIDEN_EYES();
    /**
     * void fearfulPosture()
     */
    static const ActionType& FEARFUL_POSTURE();
    /**
     * void clean([float duration])
     */
    static const ActionType& CLEAN();
    /**
     * void belch()
     */
    static const ActionType& BELCH();
    /**
     * void greet(EntityID target)
     */
    static const ActionType& GREET();
    /**
     * void dance1([float duration])
     */
    static const ActionType& DANCE1();
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
     * void leftEarBack([float duration])
     */
    static const ActionType& LEFT_EAR_BACK();
    /**
     * void rightEarBack([float duration])
     */
    static const ActionType& RIGHT_EAR_BACK();
    /**
     * void leftEarTwitch([float duration])
     */
    static const ActionType& LEFT_EAR_TWITCH();
    /**
     * void rightEarTwitch([float duration])
     */
    static const ActionType& RIGHT_EAR_TWITCH();
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
     * void bite(EntityID target [, float intensity])
     */
    static const ActionType& BITE();
    /**
     * void pet(EntityID target)
     */
    static const ActionType& PET();
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
