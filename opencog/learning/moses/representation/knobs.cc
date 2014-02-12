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

//////////////////////////
// logical_subtree_knob //
//////////////////////////

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

/////////////////////////
// action_subtree_knob //
/////////////////////////

action_subtree_knob::action_subtree_knob(combo_tree& tr, combo_tree::iterator tgt,
                                         const vector<combo_tree>& perms)
    : discrete_knob<MAX_PERM_ACTIONS>(tr), _perms(perms) {

    OC_ASSERT((int)_perms.size() < MAX_PERM_ACTIONS, "Too many perms.");

    for (int i = _perms.size() + 1;i < MAX_PERM_ACTIONS;++i)
        disallow(i);

    _default = 0;
    _current = _default;
    _loc = _tr.append_child(tgt, id::null_vertex);
}

complexity_t action_subtree_knob::complexity_bound() const {
    return tree_complexity(_loc);
}

void action_subtree_knob::clear_exemplar() {
    if (in_exemplar())
        turn(0);
    else
        _tr.erase(_loc);
}

void action_subtree_knob::turn(int idx)
{
    idx = map_idx(idx);
    OC_ASSERT(idx <= (int)_perms.size(), "Index too big.");
    
    if (idx == _current) //already set, nothing to
        return;
    
    if (idx == 0) {
        if (_current != 0) {
            combo_tree t(id::null_vertex);
            _loc = _tr.replace(_loc, t.begin());
        }
    } else {
        pre_it ite = (_perms[idx-1]).begin();
        _loc = _tr.replace(_loc, ite);
    }
    _current = idx;
}


combo_tree::iterator action_subtree_knob::append_to(combo_tree& candidate,
                                                    combo_tree::iterator& parent_dst,
                                                    int idx) const
{
    OC_ASSERT(false, "Not implemented yet");
    return combo_tree::iterator();
}

field_set::disc_spec action_subtree_knob::spec() const {
    return field_set::disc_spec(multiplicity());
}

std::string action_subtree_knob::toStr() const {
    std::stringstream ss;
    ss << "[";
    for(int i = 0; i < multiplicity(); ++i) {
        int idx = map_idx(i);
        OC_ASSERT(idx <= (int)_perms.size(), "Index too big.");
        if (idx == 0)
            ss << "nil ";
        else
            ss << _perms[idx-1];
    }
    ss << "]";
    return ss.str();
}

} //~namespace moses
} //~namespace opencog
