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

/////////////////
// contin_knob //
/////////////////

contin_knob::contin_knob(combo_tree& tr, combo_tree::iterator tgt,
                         contin_t step_size, contin_t expansion,
                         field_set::width_t depth)
    : knob_base(tr, tgt), _spec(combo::get_contin(*tgt),
                                step_size, expansion, depth) { }

bool contin_knob::in_exemplar() const
{
    return true;
}

void contin_knob::clear_exemplar() { }

void contin_knob::turn(contin_t x)
{
    *_loc = x;
}

void contin_knob::append_to(combo_tree& candidate, combo_tree::iterator parent_dst,
                            contin_t c) const
{
    if (candidate.empty())
        candidate.set_head(c);
    else
        candidate.append_child(parent_dst, c);
}

const field_set::contin_spec& contin_knob::spec() const
{
    return _spec;
}

std::string contin_knob::toStr() const
{
    std::stringstream ss;
    ss << "[" << *_loc << "]";
    return ss.str();
}

//////////////////////////
// logical_subtree_knob //
//////////////////////////

logical_subtree_knob::logical_subtree_knob(combo_tree& tr, combo_tree::iterator tgt,
                                           const logical_subtree_knob& lsk)
    : discrete_knob<3>(tr)
{
    // logger().debug("lsk = %s", lsk.toStr().c_str());
    // stringstream ss;
    // ss << "*tgt = " << *tgt;
    // logger().debug(ss.str());

    if (lsk.in_exemplar())
        _loc = _tr.child(tgt, lsk._tr.sibling_index(lsk._loc));
    else
        _loc = _tr.append_child(tgt, lsk._loc);
    _disallowed = lsk._disallowed;
    _default = lsk._default;
    _current = lsk._current;
}

logical_subtree_knob::logical_subtree_knob(combo_tree& tr, combo_tree::iterator tgt,
                                           combo_tree::iterator subtree)
    : discrete_knob<3>(tr)
{
    typedef combo_tree::sibling_iterator sib_it;
    typedef combo_tree::pre_order_iterator pre_it;

    // compute the negation of the subtree
    combo_tree negated_subtree(subtree);
    negated_subtree.insert_above(negated_subtree.begin(), id::logical_not);

    reduct::logical_reduction r;
    r(1)(negated_subtree);

    for (sib_it sib = tgt.begin(); sib != tgt.end(); ++sib) {
        if (_tr.equal_subtree(pre_it(sib), subtree) ||
            _tr.equal_subtree(pre_it(sib), negated_subtree.begin())) {
            _loc = sib;
            _current = present;
            _default = present;
            return;
        }
    }

    _loc = _tr.append_child(tgt, id::null_vertex);
    _tr.append_child(_loc, subtree);
}

complexity_t logical_subtree_knob::complexity_bound() const
{
    return (_current == absent ? 0 : tree_complexity(_loc));
}

void logical_subtree_knob::clear_exemplar()
{
    if (in_exemplar())
        turn(0);
    else
        _tr.erase(_loc);
}

void logical_subtree_knob::turn(int idx)
{
    idx = map_idx(idx);
    OC_ASSERT((idx < 3), "INVALID SETTING: Index greater than 2.");

    if (idx == _current) // already set, nothing to do
        return;

    switch (idx) {
    case absent:
        // flag subtree to be ignored with a null_vertex, replace
        // negation if present
        if (_current == negated)
            *_loc = id::null_vertex;
        else
            _loc = _tr.insert_above(_loc, id::null_vertex);
        break;
    case present:
        _loc = _tr.erase(_tr.flatten(_loc));
        break;
    case negated:
        if (_current == present)
            _loc = _tr.insert_above(_loc, id::logical_not);
        else
            *_loc = id::logical_not;
        break;
    }

    _current = idx;
}

combo_tree::iterator logical_subtree_knob::append_to(combo_tree& candidate,
                                                     combo_tree::iterator& parent_dst,
                                                     int idx) const
{
    typedef combo_tree::iterator pre_it;

    idx = map_idx(idx);
    OC_ASSERT((idx < 3), "INVALID SETTING: Index greater than 2.");

    // append v to parent_dst's children. If candidate is empty
    // then set it as head. Return the iterator pointing to the
    // new content.
    auto append_child = [&candidate](pre_it parent_dst, const vertex& v)
    {
        return candidate.empty()? candidate.set_head(v)
        : candidate.append_child(parent_dst, v);
    };

    pre_it new_src = parent_dst.end();
    if (idx == negated)
        parent_dst = append_child(parent_dst, id::logical_not);
    if (idx != absent) {
        new_src = _default == present ? _loc : (pre_it)_loc.begin();
        parent_dst = append_child(parent_dst, *new_src);
    }
    return new_src;
}

field_set::disc_spec logical_subtree_knob::spec() const {
    return field_set::disc_spec(multiplicity());
}

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
    idx = map_idx(idx);
    OC_ASSERT(idx <= (int)_perms.size(), "Index too big.");

    if (idx != 0)
        if (candidate.empty())
            candidate = _perms[idx-1];
        else
            candidate.append_child(parent_dst, _perms[idx-1].begin());

    return parent_dst.end();    // there is no child knobs
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
            ss << "nil";
        else
            ss << _perms[idx-1];
        if (i+1 < multiplicity())
            ss << "|";
    }
    ss << "]";
    return ss.str();
}

////////////////////////////////
// simple_action_subtree_knob //
////////////////////////////////

simple_action_subtree_knob::simple_action_subtree_knob(combo_tree& tr,
                                                       combo_tree::iterator tgt)
    : discrete_knob<2>(tr, tgt)
{
    _current = present;
    _default = present;
}

complexity_t csimple_action_subtree_knob::omplexity_bound() const {
    return (_current == absent ? 0 : tree_complexity(_loc));
}

void simple_action_subtree_knob::clear_exemplar() {
    //      if (in_exemplar())
    turn(0);
    //      else
    // _tr.erase(_loc);
}

void simple_action_subtree_knob::turn(int idx)
{
    idx = map_idx(idx);
    OC_ASSERT((idx < 2), "Index greater than 1.");

    if (idx == _current) //already set, nothing to
        return;

    switch (idx) {
    case present:
        _loc = _tr.erase(_tr.flatten(_loc));
        break;
    case absent:
        _loc = _tr.insert_above(_loc, id::null_vertex);
        break;
    }

    _current = idx;
}

combo_tree::iterator simple_action_subtree_knob::append_to(combo_tree& candidate,
                                                           combo_tree::iterator& parent_dst,
                                                           int idx) const
{
    OC_ASSERT(false, "Not implemented yet");
    return combo_tree::iterator();
}

field_set::disc_spec simple_action_subtree_knob::spec() const {
    return field_set::disc_spec(multiplicity());
}

std::string simple_action_subtree_knob::toStr() const {
    std::stringstream ss;
    ss << "[" << *_loc << " TODO ]";
    return ss.str();
}

} //~namespace moses
} //~namespace opencog
