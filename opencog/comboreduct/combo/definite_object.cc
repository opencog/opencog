/*
 * opencog/comboreduct/combo/definite_object.cc
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
#include "definite_object.h"

#define ACTION_NAME_POSTFIX "_action"

namespace opencog { namespace combo {

bool is_action_definite_object(const definite_object& d) {
    return std::string::npos != d.rfind(ACTION_NAME_POSTFIX);
}

std::string get_action_name(const definite_object& d) {
    return d.substr(0, d.rfind(ACTION_NAME_POSTFIX));
}

definite_object get_action_definite_object(const std::string& action_name) {
    return definite_object(action_name + std::string(ACTION_NAME_POSTFIX));
}

}} // ~namespaces combo opencog
