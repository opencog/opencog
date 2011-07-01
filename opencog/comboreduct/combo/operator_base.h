/*
 * opencog/comboreduct/combo/operator_base.h
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
#ifndef _COMBO_OPERATOR_BASE_H
#define _COMBO_OPERATOR_BASE_H

#include "common_def.h"
#include "type_tree_def.h"

namespace opencog { namespace combo {

  //that abstract class contains common methods
  //of builtin_action_base and perception_base
  class operator_base {
  public:
    virtual ~operator_base() {}

    //get_name
    virtual const std::string& get_name() const = 0;

    //type_tree
    virtual const type_tree& get_type_tree() const = 0;

    //helper methods for fast access type properties
    //number of arguments that takes the operator
    virtual arity_t arity() const = 0;
    //return the type node of the operator
    virtual type_tree get_output_type_tree() const = 0;
    //return the type tree of the input argument of index i
    virtual const type_tree& get_input_type_tree(arity_t i) const = 0;
  };

}} // ~namespace combo opencog

#endif
