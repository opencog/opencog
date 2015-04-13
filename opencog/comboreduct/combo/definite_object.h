/*
 * opencog/comboreduct/combo/definite_object.h
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
#ifndef _COMBO_DEFINITE_OBJECT_H
#define _COMBO_DEFINITE_OBJECT_H

#include <string>
#include <set>

namespace opencog { namespace combo {

typedef std::string definite_object;

typedef std::set<definite_object> definite_object_set;
typedef definite_object_set::iterator definite_object_set_it;
typedef definite_object_set::const_iterator definite_object_set_const_it;

//a definite object with suffix _action is a special definite_object
//used to reflect a self or other agent action
//the following methods allows to check wether a definite_object
//is of action type and get the action name (without the suffix "_action")
bool is_action_definite_object(const definite_object& d);
std::string get_action_name(const definite_object& d);
definite_object get_action_definite_object(const std::string& action_name);

}} // ~namespaces combo opencog

#endif
