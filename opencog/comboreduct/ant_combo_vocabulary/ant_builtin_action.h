/*
 * opencog/comboreduct/ant_combo_vocabulary/ant_builtin_action.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _ANT_BUILTIN_ACTION_H
#define _ANT_BUILTIN_ACTION_H

#include <opencog/util/numeric.h>

#include <opencog/comboreduct/combo/builtin_action.h>
#include "ant_operator.h"

namespace opencog { namespace combo {
  
//later to be replaced by id
namespace id {
  enum ant_builtin_action_enum {
    move_forward,
    turn_left,
    turn_right,
    
    ant_builtin_action_count //to give the number of actions
  };
}

typedef id::ant_builtin_action_enum ant_builtin_action_enum;

/*********************************************************************
 *         Arrays containing action name type and properties         *
 *                 to be edited by the developer                     *
 *********************************************************************/

namespace ant_builtin_action_properties {

  //struct for description of name and type
  typedef opencog::combo::ant_operator<ant_builtin_action_enum, id::ant_builtin_action_count>::basic_description action_basic_description;

  //struct for decription of action properties
  struct action_property_description {
    ant_builtin_action_enum action;
    bool idempotent;
    bool always_succeeds; //true iff the action_result is always action_succeed
    bool reversible;
    ant_builtin_action_enum reversal;
  };


  //the following structure can be used to stock action argument properties
  //although in the ant problem no action takes argument, that structure
  //is kept in example to be reused elsewhere.

  //struct for description of action argument
  struct action_argument_property_description {
    ant_builtin_action_enum action;
    unsigned char argument_index;
    bool additive;
    bool zero_neutral;
    bool modular;
    double min_value;
    double max_value;
  };

  static const action_basic_description abd[] = {
    //builtin action         name                 type  
    { id::move_forward,      "move_forward",      "action_result" },
    { id::turn_left,         "turn_left",         "action_result" },
    { id::turn_right,        "turn_right",        "action_result" }
  };
  
  static const action_property_description apd[] = {
    // builtin action     idempotent always_succeed  reversal        reversed action
    { id::move_forward,   false,     true,           false,          (ant_builtin_action_enum)0 },
    { id::turn_left,      false,     true,           true,           id::turn_right },
    { id::turn_right,     false,     true,           true,           id::turn_left }
  };
  
  //this array is empty because no action has arguments
  static const action_argument_property_description aapd[] = {
    // builtin action       argument index    addit. z.neut. modular min max
  };
  
}//~namespace ant_builtin_action_properties

class ant_builtin_action : public builtin_action_base, public ant_operator<ant_builtin_action_enum, id::ant_builtin_action_count> {

private:

  //standard properties
  bool _always_succeeds;
  bool _reversible;
  const ant_builtin_action* _reversal;
  bool _idempotent;
  std::vector<bool> _arg_additive;
  bool _exists_additive_argument;
  std::vector<bool> _arg_zero_neutral;
  bool _exists_zero_neutral_argument;
  std::vector<bool> _arg_modulo;
  std::vector<double> _arg_modulo_min;
  std::vector<double> _arg_modulo_max;

  //private methods

  //ctor
  ant_builtin_action();

  const basic_description* get_basic_description_array() const;
  unsigned int get_basic_description_array_count() const;

  static const ant_builtin_action* init_actions();
  //set an action with all its name, type and properties
  //action_array is used to refer to other actions
  //as _reversal needs for instance
  void set_action(ant_builtin_action_enum, ant_builtin_action*);

public:
  //return a pointer of the static builtin_action corresponding
  //to a given name string
  //if no such builtin_action exists then return NULL pointer
  static builtin_action instance(const std::string& name);

  //return a pointer of the static builtin_action corresponding
  //to a given ant_builtin_action_enum
  static builtin_action instance(ant_builtin_action_enum);

  //basic access methods
  const std::string& get_name() const;
  const type_tree& get_type_tree() const;
  arity_t arity() const;
  type_tree get_output_type_tree() const;

  //return the type tree of the argument of index i
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
};

}} // ~namespaces combo opencog

#endif
