/*
 * opencog/learning/moses/moses/knobs.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#include "knobs.h"

namespace opencog { namespace moses {

std::string logical_subtree_knob::toStr() const
{
    std::stringstream ss;
    ss << "[";
    for(int i = 0; i < multiplicity(); ++i)
        ss << posStr(map_idx(i)) << (i < multiplicity()-1? " " : "");
    ss << "]";
    return ss.str();
}

// return << *_loc or << *_loc.begin() if it is null_vertex
// if *_loc is a negative literal returns !$n
// if negated is true a copy of the literal is negated before being printed
std::string logical_subtree_knob::locStr(bool negated) const
{
    OC_ASSERT(*_loc != id::null_vertex || _loc.has_one_child(),
              "if _loc is null_vertex then it must have only one child");
    std::stringstream ss;
    combo_tree::iterator it;
    if (*_loc == id::null_vertex)
        it = _loc.begin();
    else it = _loc;
    if (is_argument(*it)) {
        argument arg = get_argument(*it);
        if (negated) arg.negate();
        ostream_abbreviate_literal(ss, arg);
    } else {
        ss << (negated? "!" : "") << *it;
    }
    return ss.str();
}

// Return the name of the position, if it is the current one and
// tag_current is true then the name is put in parenthesis.
std::string logical_subtree_knob::posStr(int pos, bool tag_current) const
{
    std::stringstream ss;
    switch (pos) {
    case absent:
        ss << "nil";
        break;
    case present:
        ss << locStr();
        break;
    case negated:
        ss << locStr(true);
        break;
    default:
        ss << "INVALID SETTING";
    }
    return pos == _current && tag_current?
        std::string("(") + ss.str() + ")" : ss.str();
}

} //~namespace moses
} //~namespace opencog
