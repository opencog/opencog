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
const ActionType& ActionType::BARK()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "duration"};
    static ActionType* result = new ActionType(BARK_CODE, "bark", EMPTY, ENTITY_and_FLOAT, paramNames, "void bark([EntityID target [, float duration]])");
    return *result;
}
const ActionType& ActionType::TRICK_FOR_FOOD()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(TRICK_FOR_FOOD_CODE, "trick_for_food", EMPTY, EMPTY, paramNames, "void trick_for_food()");
    return *result;
}
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
const ActionType& ActionType::LIE_DOWN()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(LIE_DOWN_CODE, "lie_down", EMPTY, FLOAT, paramNames, "void lie_down([float duration])");
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
const ActionType& ActionType::STRETCH()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(STRETCH_CODE, "stretch", EMPTY, EMPTY, paramNames, "void stretch()");
    return *result;
}
const ActionType& ActionType::SCRATCH_SELF_NOSE()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SCRATCH_SELF_NOSE_CODE, "scratch_self_nose", EMPTY, FLOAT, paramNames, "void scratch_self_nose([float duration])");
    return *result;
}
const ActionType& ActionType::SCRATCH_SELF_RIGHT_EAR()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SCRATCH_SELF_RIGHT_EAR_CODE, "scratch_self_right_ear", EMPTY, FLOAT, paramNames, "void scratch_self_right_ear([float duration])");
    return *result;
}
const ActionType& ActionType::SCRATCH_SELF_LEFT_EAR()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SCRATCH_SELF_LEFT_EAR_CODE, "scratch_self_left_ear", EMPTY, FLOAT, paramNames, "void scratchSelfLeftEar([float duration])");
    return *result;
}
const ActionType& ActionType::SCRATCH_SELF_NECK()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SCRATCH_SELF_NECK_CODE, "scratch_self_neck", EMPTY, FLOAT, paramNames, "void scratchSelfNeck([float duration])");
    return *result;
}
const ActionType& ActionType::SCRATCH_SELF_RIGHT_SHOULDER()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SCRATCH_SELF_RIGHT_SHOULDER_CODE, "scratch_self_right_shoulder", EMPTY, FLOAT, paramNames, "void scratchSelfRightShoulder([float duration])");
    return *result;
}
const ActionType& ActionType::SCRATCH_SELF_LEFT_SHOULDER()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SCRATCH_SELF_LEFT_SHOULDER_CODE, "scratch_self_left_shoulder", EMPTY, FLOAT, paramNames, "void scratchSelfLeftShoulder([float duration])");
    return *result;
}
const ActionType& ActionType::SCRATCH_GROUND_BACK_LEGS()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SCRATCH_GROUND_BACK_LEGS_CODE, "scratch_ground_back_legs", EMPTY, FLOAT, paramNames, "void scratchGroundBackLegs([float duration])");
    return *result;
}
const ActionType& ActionType::RUN_IN_CIRCLE()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(RUN_IN_CIRCLE_CODE, "run_in_circle", EMPTY, FLOAT, paramNames, "void runInCircle([float duration])");
    return *result;
}
const ActionType& ActionType::ANTICIPATE_PLAY()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(ANTICIPATE_PLAY_CODE, "anticipate_play", EMPTY, EMPTY, paramNames, "void anticipatePlay()");
    return *result;
}
const ActionType& ActionType::BEG()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(BEG_CODE, "beg", EMPTY, EMPTY, paramNames, "void beg()");
    return *result;
}
const ActionType& ActionType::HEEL()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(HEEL_CODE, "heel", EMPTY, FLOAT, paramNames, "void heel([float duration])");
    return *result;
}
const ActionType& ActionType::HIDE_FACE()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(HIDE_FACE_CODE, "hide_face", EMPTY, FLOAT, paramNames, "void hideFace([float duration])");
    return *result;
}
const ActionType& ActionType::PLAY_DEAD()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(PLAY_DEAD_CODE, "play_dead", EMPTY, EMPTY, paramNames, "void playDead()");
    return *result;
}
const ActionType& ActionType::FOLLOW()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "duration"};
    static ActionType* result = new ActionType(FOLLOW_CODE, "follow", ENTITY, FLOAT, paramNames, "void follow(EntityID target[, float duration])");
    return *result;
}
const ActionType& ActionType::LICK()
{
    initParamTypes();
    static const char* paramNames[] = {"target"};
    static ActionType* result = new ActionType(LICK_CODE, "lick", EMPTY, ENTITY, paramNames, "void lick([EntityID target])");
    return *result;
}
const ActionType& ActionType::NUDGE_TO()
{
    initParamTypes();
    static const char* paramNames[] = {"moveableObj", "target"};
    static ActionType* result = new ActionType(NUDGE_TO_CODE, "nudge_to", ENTITY_and_VECTOR, EMPTY, paramNames, "void nudge(EntityID moveableObj, Vector target)");
    return *result;
}
const ActionType& ActionType::TAP_DANCE()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(TAP_DANCE_CODE, "tap_dance", EMPTY, FLOAT, paramNames, "void tapDance([float duration])");
    return *result;
}
const ActionType& ActionType::BARE_TEETH()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "duration"};
    static ActionType* result = new ActionType(BARE_TEETH_CODE, "bare_teeth", EMPTY, ENTITY_and_FLOAT, paramNames, "void bareTeeth([EntityID target [, float duration]])");
    return *result;
}
const ActionType& ActionType::GROWL()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "duration"};
    static ActionType* result = new ActionType(GROWL_CODE, "growl", EMPTY, ENTITY_and_FLOAT, paramNames, "void growl([EntityID target [, float duration]])");
    return *result;
}
const ActionType& ActionType::LOOK_UP_TURN_HEAD()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(LOOK_UP_TURN_HEAD_CODE, "look_up_turn_head", EMPTY, FLOAT, paramNames, "void lookUpTurnHead([float duration])");
    return *result;
}
const ActionType& ActionType::WHINE()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "duration"};
    static ActionType* result = new ActionType(WHINE_CODE, "whine", EMPTY, ENTITY_and_FLOAT, paramNames, "void whine([EntityID target [, float duration]])");
    return *result;
}
const ActionType& ActionType::SNIFF()
{
    initParamTypes();
    static const char* paramNames[] = {"target"};
    static ActionType* result = new ActionType(SNIFF_CODE, "sniff", EMPTY, ENTITY, paramNames, "void sniff([EntityID target])");
    return *result;
}
const ActionType& ActionType::SNIFF_AT()
{
    initParamTypes();
    static const char* paramNames[] = {"target"};
    static ActionType* result = new ActionType(SNIFF_AT_CODE, "sniff_at", ENTITY, EMPTY, paramNames, "void sniffAt(EntityID target)");
    return *result;
}
const ActionType& ActionType::SNIFF_PET_PART()
{
    initParamTypes();
    static const char* paramNames[] = {"pet part"};
    static ActionType* result = new ActionType(SNIFF_PET_PART_CODE, "sniff_pet_part", ENTITY_and_INT, EMPTY, paramNames, "void sniffPetPart(EntityID pet, integer part)");
    return *result;
}
const ActionType& ActionType::SNIFF_AVATAR_PART()
{
    initParamTypes();
    static const char* paramNames[] = {"avatar part"};
    static ActionType* result = new ActionType(SNIFF_AVATAR_PART_CODE, "sniff_avatar_part", ENTITY_and_INT, EMPTY, paramNames, "void sniffAvatarPart(EntityID avatar, integer part)");
    return *result;
}
const ActionType& ActionType::SHAKE_HEAD()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(SHAKE_HEAD_CODE, "shake_head", EMPTY, FLOAT, paramNames, "void shakeHead([float duration])");
    return *result;
}
const ActionType& ActionType::EARS_BACK()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(EARS_BACK_CODE, "ears_back", EMPTY, FLOAT, paramNames, "void earsBack([float duration])");
    return *result;
}
const ActionType& ActionType::EARS_TWITCH()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(EARS_TWITCH_CODE, "ears_twitch", EMPTY, FLOAT, paramNames, "void earTwitch([float duration])");
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
const ActionType& ActionType::PEE()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(PEE_CODE, "pee", EMPTY, EMPTY, paramNames, "void pee()");
    return *result;
}
const ActionType& ActionType::POO()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(POO_CODE, "poo", EMPTY, EMPTY, paramNames, "void poo()");
    return *result;
}
const ActionType& ActionType::WAG()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(WAG_CODE, "wag", EMPTY, FLOAT, paramNames, "void wag([float duration])");
    return *result;
}
const ActionType& ActionType::TAIL_FLEX()
{
    initParamTypes();
    static const char* paramNames[] = {"position", "duration"};
    static ActionType* result = new ActionType(TAIL_FLEX_CODE, "tail_flex", VECTOR, FLOAT, paramNames, "void tailFlex(Vector position [, float duration])");
    return *result;
}
const ActionType& ActionType::CHEW()
{
    initParamTypes();
    static const char* paramNames[] = {"id"};
    static ActionType* result = new ActionType(CHEW_CODE, "chew", EMPTY, ENTITY, paramNames, "void chew([EntityID id])");
    return *result;
}
const ActionType& ActionType::DREAM()
{
    initParamTypes();
    static const char* paramNames[] = {"id"};
    static ActionType* result = new ActionType(DREAM_CODE, "dream", EMPTY, ENTITY, paramNames, "void dream(EntityID id)");
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
const ActionType& ActionType::SCRATCH_OTHER()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "duration"};
    static ActionType* result = new ActionType(SCRATCH_OTHER_CODE, "scratch_other", ENTITY, FLOAT, paramNames, "void scratchOther(EntityID target [, float duration])");
    return *result;
}
const ActionType& ActionType::EARS_PERK()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(EARS_PERK_CODE, "perk_ear", EMPTY, FLOAT, paramNames, "void earsPerk([float duration])");
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
const ActionType& ActionType::VOMIT()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(VOMIT_CODE, "vomit", EMPTY, EMPTY, paramNames, "void vomit()");
    return *result;
}
const ActionType& ActionType::LEAN_ROCK_DANCE()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(LEAN_ROCK_DANCE_CODE, "lean_rock_dance", EMPTY, EMPTY, paramNames, "void leanRockDance()");
    return *result;
}
const ActionType& ActionType::BACK_FLIP()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(BACK_FLIP_CODE, "back_flip", EMPTY, EMPTY, paramNames, "void backFlip()");
    return *result;
}
const ActionType& ActionType::WIDEN_EYES()
{
    initParamTypes();
    static const char* paramNames[] = {"interval"};
    static ActionType* result = new ActionType(WIDEN_EYES_CODE, "widen_eyes", EMPTY, INT, paramNames, "void widenEyes([short interval])");
    return *result;
}
const ActionType& ActionType::FEARFUL_POSTURE()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(FEARFUL_POSTURE_CODE, "fearful_posture", EMPTY, EMPTY, paramNames, "void fearfulPosture()");
    return *result;
}
const ActionType& ActionType::CLEAN()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(CLEAN_CODE, "clean", EMPTY, FLOAT, paramNames, "void clean([float duration])");
    return *result;
}
const ActionType& ActionType::BELCH()
{
    initParamTypes();
    static const char* paramNames[] = {};
    static ActionType* result = new ActionType(BELCH_CODE, "belch", EMPTY, EMPTY, paramNames, "void belch()");
    return *result;
}
const ActionType& ActionType::GREET()
{
    initParamTypes();
    static const char* paramNames[] = {"target"};
    static ActionType* result = new ActionType(GREET_CODE, "greet", ENTITY, EMPTY, paramNames, "void greet(EntityID target)");
    return *result;
}
const ActionType& ActionType::DANCE1()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(DANCE1_CODE, "dance1", EMPTY, FLOAT, paramNames, "void dance1([float duration])");
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
const ActionType& ActionType::LEFT_EAR_PERK()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(LEFT_EAR_PERK_CODE, "left_ear_perk", EMPTY, FLOAT, paramNames, "void leftEarPerk([float duration])");
    return *result;
}
const ActionType& ActionType::RIGHT_EAR_PERK()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(RIGHT_EAR_PERK_CODE, "right_ear_perk", EMPTY, FLOAT, paramNames, "void rightEarPerk([float duration])");
    return *result;
}
const ActionType& ActionType::LEFT_EAR_BACK()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(LEFT_EAR_BACK_CODE, "left_ear_back", EMPTY, FLOAT, paramNames, "void leftEarBack([float duration])");
    return *result;
}
const ActionType& ActionType::RIGHT_EAR_BACK()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(RIGHT_EAR_BACK_CODE, "right_ear_back", EMPTY, FLOAT, paramNames, "void rightEarBack([float duration])");
    return *result;
}
const ActionType& ActionType::LEFT_EAR_TWITCH()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(LEFT_EAR_TWITCH_CODE, "left_ear_twitch", EMPTY, FLOAT, paramNames, "void leftEarTwitch([float duration])");
    return *result;
}
const ActionType& ActionType::RIGHT_EAR_TWITCH()
{
    initParamTypes();
    static const char* paramNames[] = {"duration"};
    static ActionType* result = new ActionType(RIGHT_EAR_TWITCH_CODE, "right_ear_twitch", EMPTY, FLOAT, paramNames, "void rightEarTwitch([float duration])");
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
const ActionType& ActionType::BITE()
{
    initParamTypes();
    static const char* paramNames[] = {"target", "intensity"};
    static ActionType* result = new ActionType(BITE_CODE, "bite", ENTITY, FLOAT, paramNames, "void bite(EntityID target [, float intensity])");
    return *result;
}

const ActionType& ActionType::PET()
{
    initParamTypes();
    static const char* paramNames[] = {"target"};
    static ActionType* result = new ActionType(PET_CODE, "pet", ENTITY, EMPTY, paramNames, "void pet(EntityID target)");
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
        BARK();
        TRICK_FOR_FOOD();
        EAT();
        WALK();
        MOVE_TO_OBJ();
        GRAB();
        DROP();
        SIT();
        LIE_DOWN();
        FLY();
        FLY_FOLLOW();
        STRETCH();
        SCRATCH_SELF_NOSE();
        SCRATCH_SELF_RIGHT_EAR();
        SCRATCH_SELF_LEFT_EAR();
        SCRATCH_SELF_NECK();
        SCRATCH_SELF_RIGHT_SHOULDER();
        SCRATCH_SELF_LEFT_SHOULDER();
        SCRATCH_GROUND_BACK_LEGS();
        RUN_IN_CIRCLE();
        ANTICIPATE_PLAY();
        BEG();
        HEEL();
        HIDE_FACE();
        PLAY_DEAD();
        FOLLOW();
        LICK();
        NUDGE_TO();
        TAP_DANCE();
        BARE_TEETH();
        GROWL();
        LOOK_UP_TURN_HEAD();
        WHINE();
        SNIFF();
        SNIFF_AT();
        SNIFF_PET_PART();
        SNIFF_AVATAR_PART();
        SHAKE_HEAD();
        EARS_BACK();
        EARS_TWITCH();
        MOVE_HEAD();
        WAKE();
        SLEEP();
        DRINK();
        PEE();
        POO();
        WAG();
        TAIL_FLEX();
        CHEW();
        DREAM();
        TURN();
        ROTATE();
        SCRATCH_OTHER();
        EARS_PERK();
        JUMP_UP();
        JUMP_TOWARD();
        JUMP_FORWARD();
        PAY_ATTENTION();
        VOMIT();
        LEAN_ROCK_DANCE();
        BACK_FLIP();
        WIDEN_EYES();
        FEARFUL_POSTURE();
        CLEAN();
        BELCH();
        GREET();
        DANCE1();
        LOOK_RIGHT();
        LOOK_LEFT();
        KICK_LEFT();
        KICK_RIGHT();
        LEFT_EAR_PERK();
        RIGHT_EAR_PERK();
        LEFT_EAR_BACK();
        RIGHT_EAR_BACK();
        LEFT_EAR_TWITCH();
        RIGHT_EAR_TWITCH();
        ANGRY_EYES();
        SAD_EYES();
        HAPPY_EYES();
        CLOSE_EYES();
        BITE();
        PET();
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
    for (Name2ActionTypeMap::const_iterator itr = nameMap.begin(); itr != nameMap.end(); itr++) {
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
