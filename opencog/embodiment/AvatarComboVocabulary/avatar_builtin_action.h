/*
 * opencog/embodiment/AvatarComboVocabulary/avatar_builtin_action.h
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

#ifndef _AVATAR_BUILTIN_ACTION_H
#define _AVATAR_BUILTIN_ACTION_H

#include <opencog/util/numeric.h>

#include <opencog/comboreduct/combo/builtin_action.h>

#include "avatar_operator.h"

namespace opencog { namespace combo {

namespace id {

enum avatar_builtin_action_enum {

    //Note that all following comments are based on experimentation
    //with the MV-Proxy. It does not necessarily apply for another proxy

    //==== goto_obj(destination speed) ====
    // There isn't a fixed maximum and minimum value for speed.
    // It depends on the proxy implementation.
    // Some tests on MV-Proxy for speed:
    // < 1.2: very slow walk (not recommended)
    // >= 1.2 and < 3.1: walk (recommended: 2)
    // >= 3.1 and < 5: moderate run (recommended: 4)
    // >= 5 and < 7: fast run (recommended: 6)
    // >= 7: causes flickering on the animation during the agent motion
    goto_obj,

    //==== gonear_obj(destination speed) ====
    // like goto_obj but get near the destination instead of at it
    gonear_obj,

    //==== gobehind_obj(object speed) ====
    // The agent is positioned at a given side of that object.
    // This action  will compute a position situated immediately
    // on the other side of the target object, relative to the current
    // agent's position, besides building a walk plan to make the agent
    // go there.
    gobehind_obj,

    //==== follow(object duration speed) ====
    // not tested, apparently duration is not taken into acount yet
    follow,

    //==== step_towards(object direction) ====
    // direction can be TOWARDS or AWAY
    // Note: seems way too far (more than 10 meters)
    step_towards,

    //==== step_backward ====
    // Note: seems way too far (more than 10 meters)
    step_backward,

    //==== step_forward ====
    // Note: seems way too far (more than 10 meters)
    step_forward,

    //==== random_step ====
    // not implemented or buggy
    random_step,

    //==== rotate_left ====
    // seems buggy (it turns of an inch)
    rotate_left,

    //==== rotate_right ====
    // not implemented or buggy
    rotate_right,

    //==== rotate ====
    // A generic rotate function that accepts an rotating angle
    rotate,

    //==== jump_up ====
    jump_up,

    //==== jump_towards(object) ====
    // not implemented on MV-Proxy
    jump_towards,

    jump_forward,

    //==== turn_to_face(object) ====
    // does not turn in the right direction
    turn_to_face,

    //==== go_behind(object1 object2 speed) ====
    // seems buggy or not implemented
    go_behind,

    //==== grab(object) ====
    grab,

    //==== nudge_to(object1 object2) ====
    // seems not to nudge object only to execute goto_obj actions
    nudge_to,

    //==== drop ====
    drop,

    //==== eat(object) ====
    eat,

    //==== drink(object) ====
    drink,

    //==== sit ====
    sit,

    //==== move_head(contin contin contin)
    // not implemented in MV-Proxy yet
    move_head,

    //==== widen_eyes ====
    widen_eyes,

    //==== shake_head ====
    shake_head,

    //==== kick(object) ====
    // not implemented in MV-Proxy yet
    kick,

    //==== kick_left ====
    kick_left,

    //==== kick_right ====
    kick_right,

    group_command,
    receive_latest_group_commands,

    //==== look_at(object) ====
    // look at object for a few seconds
    look_at,

    //=== say a sentence ===
    say,

    //==== build_block(offset, block_type) ====
    // build a block in front of avatar, used in unity minecraft-like world only.
    build_block,

    //==== destroy_block(offset) ====
    // destroy a block at given vertical offset, used in unity minecraft-like world only.
    destroy_block,

    avatar_builtin_action_count //to give the number of actions
};

} // ~namespace id

typedef id::avatar_builtin_action_enum avatar_builtin_action_enum;

/*********************************************************************
 *         Arrays containing action name type and properties         *
 *                 to be edited by the developer                     *
 *********************************************************************/

namespace avatar_builtin_action_properties {

//struct for description of name and type
typedef avatar_operator<avatar_builtin_action_enum, id::avatar_builtin_action_count>::basic_description action_basic_description;

//struct for decription of action properties
struct action_property_description {
    avatar_builtin_action_enum action;
    bool compound; // Compound is not in the set of required action
                   // but is needed for the Operational Avatar
                   // Controller (OAC), loosely saying that the action
                   // requires possibly several cycles to be completed
    bool idempotent;
    bool reversible;
    bool always_succeeds; //true iff the action_result is always action_succeed
    avatar_builtin_action_enum reversal;
};

//struct for description of action argument
struct action_argument_property_description {
    avatar_builtin_action_enum action;
    unsigned char argument_index;
    bool additive;
    bool zero_neutral;
    bool modular;
    double min_value;
    double max_value;
};

struct action_precedence {
    avatar_builtin_action_enum action1;
    avatar_builtin_action_enum action2;
};

static const action_basic_description abd[] = {
    //builtin action             name                 type
    { id::goto_obj,          "goto_obj",          "->(union(definite_object indefinite_object wild_card) contin action_result)" },
    { id::gonear_obj,        "gonear_obj",        "->(union(definite_object indefinite_object wild_card) contin action_result)" },
    { id::gobehind_obj,      "gobehind_obj",      "->(union(definite_object indefinite_object wild_card) contin action_result)" },
    { id::go_behind,         "go_behind",         "->(union(definite_object indefinite_object wild_card) union(definite_object indefinite_object wild_card) contin action_result)" },
    { id::follow,            "follow",            "->(union(definite_object indefinite_object wild_card) contin contin action_result)" },
    { id::step_towards,      "step_towards",      "->(union(definite_object indefinite_object wild_card) action_symbol action_result)" },
    { id::step_backward,     "step_backward",     "action_result" },
    { id::step_forward,      "step_forward",      "action_result" },
    { id::random_step,       "random_step",       "action_result" },
    { id::rotate_left,       "rotate_left",       "action_result" },
    { id::rotate_right,      "rotate_right",      "action_result" },
    { id::rotate,            "rotate",            "->(contin action_result)" },
    { id::jump_up,           "jump_up",           "action_result" },
    { id::jump_towards,      "jump_towards",      "->(union(definite_object indefinite_object wild_card) action_result)" },
    { id::jump_forward,      "jump_forward",      "->(contin action_result)" },
    { id::turn_to_face,      "turn_to_face",      "->(union(definite_object indefinite_object wild_card) action_result)" },
    { id::grab,              "grab",              "->(union(definite_object indefinite_object wild_card) action_result)" },
    { id::nudge_to,          "nudge_to",          "->(union(definite_object indefinite_object wild_card) union(definite_object indefinite_object wild_card) action_result)" },
    { id::drop,              "drop",              "action_result" },
    { id::eat,               "eat",               "->(union(definite_object indefinite_object wild_card) action_result)" },
    { id::drink,             "drink",             "->(union(definite_object indefinite_object wild_card) action_result)" },
    { id::sit,               "sit",               "action_result" },
    { id::move_head,         "move_head",         "->(contin contin contin action_result)" },
    { id::widen_eyes,        "widen_eyes",        "action_result" },
    { id::shake_head,        "shake_head",        "action_result" },
    { id::kick,              "kick",              "->(union(definite_object indefinite_object wild_card) action_result)" },
    { id::kick_left,         "kick_left",         "action_result" },
    { id::kick_right,        "kick_right",        "action_result" },
    { id::group_command,     "group_command",     "->(union(definite_object indefinite_object wild_card) definite_object arg_list(union(definite_object contin boolean)) action_result)" },
    //{ id::group_command,     "group_command",     "->(union(definite_object indefinite_object wild_card) definite_object union(definite_object contin boolean) union(definite_object contin boolean) action_result)" },
    { id::receive_latest_group_commands,          "receive_latest_group_commands",     "action_result" },
    { id::look_at,           "look_at",           "->(union(definite_object indefinite_object wild_card) action_result)" },
    { id::say,               "say",               "->(definite_object union(definite_object indefinite_object wild_card) action_result)" },
    { id::build_block,       "build_block",       "->(contin definite_object action_result)" },
    //{ id::destroy_block_at,  "destroy_block_at",  "->(union(definite_object indefinite_object wild_card) action_result)" },
    { id::destroy_block,     "destroy_block",     "->(contin action_result)" },

};

// compound: an action is compound whether it is composed by two or more steps
// and only after the oac receive the last step execution status, the action will be considered done
static const action_property_description apd[] = {
    // builtin action     compound  idempotent reversible  always_succeed  reversal
    { id::goto_obj,       true,     false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::gonear_obj,     true,     false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::gobehind_obj,   true,     false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::go_behind,      true,     false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::follow,         true,     false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::step_towards,   false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::step_backward,  false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::step_forward,   false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::random_step,    false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::rotate_left,    false,    false,     true,       true,           id::rotate_right },
    { id::rotate_right,   false,    false,     true,       true,           id::rotate_left },
    { id::rotate,         false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::jump_up,        false,    false,     false,      true,           (avatar_builtin_action_enum)0 },
    { id::jump_towards,   false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::jump_forward,   false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::turn_to_face,   false,    false,     false,      true,           (avatar_builtin_action_enum)0 },
    { id::grab,           false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::nudge_to,       true,     false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::drop,           false,    true,      false,      true,           (avatar_builtin_action_enum)0 },
    { id::eat,            false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::drink,          false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::sit,            false,    true,      false,      true,           (avatar_builtin_action_enum)0 },
    { id::widen_eyes,     false,    false,     false,      true,           (avatar_builtin_action_enum)0 },
    { id::shake_head,     false,    false,     false,      true,           (avatar_builtin_action_enum)0 },
    { id::kick,           false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::kick_left,      false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::kick_right,     false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::group_command,  false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::receive_latest_group_commands,
                          false,    false,     false,      true,           (avatar_builtin_action_enum)0 },
    { id::look_at,        false,    false,     false,      true,           (avatar_builtin_action_enum)0 },
    { id::say,            false,    false,     false,      true,           (avatar_builtin_action_enum)0 },

    { id::build_block,    false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
    { id::destroy_block,  false,    false,     false,      false,          (avatar_builtin_action_enum)0 },
};


//in the case of builtin action with arg_list argument
//(like group_command) the semantics is that the property at arg index
//corresponding to the arg_list will be assigned to all arguments
//of the arg_list
static const action_argument_property_description aapd[] = {
    // builtin action           arg index addit. z.neut. modular min max
    { id::goto_obj,                 0,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::goto_obj,                 1,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::gonear_obj,               0,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::gonear_obj,               1,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::gobehind_obj,             0,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::gobehind_obj,             1,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::go_behind,                0,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::go_behind,                1,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::go_behind,                2,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::follow,                   0,    false, false,  false,  0,  0 },
    { id::follow,                   1,    false, false,  false,  0,  0 },
    { id::follow,                   2,    false, false,  false,  0,  0 },

    { id::step_towards,             0,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::step_towards,             1,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::jump_towards,             0,    false, false,  false,  0,  0 },
    { id::jump_forward,             0,    false, false,  false,  0,  0 },
    { id::turn_to_face,             0,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::rotate,                   0,    false, false,  false,  0,  0 },
    { id::grab,                     0,    false, false,  false,  0,  0 },
    { id::nudge_to,                 0,    false, false,  false,  0,  0 },
    { id::nudge_to,                 1,    false, false,  false,  0,  0 },
    { id::eat,                      0,    false, false,  false,  0,  0 },
    { id::drink,                    0,    false, false,  false,  0,  0 },
    { id::move_head,                0,    true,  false,  true,  -PI,  PI },
    { id::move_head,                1,    true,  false,  true,  -PI,  PI },
    { id::move_head,                2,    true,  false,  true,  -PI,  PI },
    { id::kick,                     0,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::group_command,            0,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::group_command,            1,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::group_command,            2,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::group_command,            3,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::look_at,                  0,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::say,                      0,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1
    { id::say,                      1,    false, false,  false,  0,  0 }, // not specified in Pet_Action_Spec_1.1

    { id::build_block,              0,    false, false,  false,  0,  0 }, 
    { id::build_block,              1,    false, false,  false,  0,  0 }, 
    { id::destroy_block,            0,    false, false,  false,  0,  0 }, 
};

static const action_precedence must_precede[] = {
    { id::grab, id::drop  }
};

}//~namespace avatar_builtin_action_properties

class avatar_builtin_action : public builtin_action_base, public avatar_operator<avatar_builtin_action_enum, id::avatar_builtin_action_count>
{

private:

    //standard properties
    bool _reversible;
    bool _always_succeeds;
    const avatar_builtin_action* _reversal;
    bool _idempotent;
    std::vector<bool> _arg_additive;
    bool _exists_additive_argument;
    std::vector<bool> _arg_zero_neutral;
    bool _exists_zero_neutral_argument;
    std::vector<bool> _arg_modulo;
    std::vector<double> _arg_modulo_min;
    std::vector<double> _arg_modulo_max;
    std::set<builtin_action> _preconditions;

    //PetBrain property
    bool _compound;

    //private methods

    //ctor
    avatar_builtin_action();

    const basic_description* get_basic_description_array() const;
    unsigned int get_basic_description_array_count() const;

    static const avatar_builtin_action* init_actions();
    //set an action with all its name, type and properties
    //action_array is used to refer to other actions
    //as _reversal needs for instance
    void set_action(avatar_builtin_action_enum,
                    avatar_builtin_action* action_array);

public:
    //return a pointer of the static builtin_action corresponding
    //to a given name string
    //if no such builtin_action exists then return NULL pointer
    static builtin_action get_instance(const std::string& name);

    //return a pointer of the static builtin_action corresponding
    //to a given avatar_builtin_action_enum
    static builtin_action get_instance(avatar_builtin_action_enum);

    //basic access methods
    const std::string& get_name() const;
    const type_tree& get_type_tree() const;
    arity_t arity() const;
    type_tree get_output_type_tree() const;

    //return the type tree of the input argument of index i
    //if the operator has arg_list(T) as last input argument
    //then it returns always T past that index
    const type_tree& get_input_type_tree(arity_t i) const;

    //action property methods
    bool is_reversible() const;
    bool always_succeeds() const;
    builtin_action get_reversal() const;
    bool is_idempotent() const;
    bool is_additive(arity_t index) const;
    bool exists_additive_argument() const;
    bool is_zero_neutral(arity_t index) const;
    bool exists_zero_neutral_argument() const;
    bool is_modulo(arity_t index) const;
    double modulo_min(arity_t index) const;
    double modulo_max(arity_t index) const;
    const std::set<builtin_action> preconditions() const;
    //PetBrain specific property method
    bool is_compound() const;
};

}} // ~namespaces combo opencog

#endif
