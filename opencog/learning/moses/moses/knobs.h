/*
 * opencog/learning/moses/moses/knobs.h
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
#ifndef _MOSES_KNOBS_H
#define _MOSES_KNOBS_H

#include <bitset>

#include <opencog/util/tree.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/numeric.h>
#include <opencog/util/based_variant.h>

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>

#include "using.h"
#include "../eda/field_set.h"
#include "complexity.h"

namespace opencog { namespace moses {

using namespace ant_combo;

//a knob represents a single dimension of variation relative to an exemplar
//program tree. This may be discrete or continuous. In the discrete case, the
//various settings are accessible via turn(0),turn(1),...turn(arity()-1). In
//the continuous case, turn(contin_t) is used.
//
//for example, given the program tree fragment or(0<(*(#1,0.5)),#2), a
//continuous knob might be used to vary the numerical constant. So setting
//this knob to 0.7 would transform the tree fragment to
//or(0<(*(#1,0.7)),#2). A discrete knob with arity()==3 might be used to
//transform the boolean input #2. So setting this knob to 1 might transform
//the tree to or(0<(*(#1,0.7)),not(#2)), and setting it to 2 might remove it
//from the tree (while setting it to 0 would return to the original tree).

struct knob_base {
    knob_base(combo_tree& tr, combo_tree::iterator log)
        : _tr(&tr), _loc(log) {}
    knob_base(combo_tree& tr) : _tr(&tr), _loc(tr.end()) {}
    virtual ~knob_base() { }

    //is the feature nonzero by default? i.e., is it present in the exemplar?
    virtual bool in_exemplar() const = 0;

    //return the exemplar to its state before the knob was created (deleting
    //any null vertices if present)
    virtual void clear_exemplar() = 0;

    combo_tree::iterator get_loc() const {
        return _loc; 
    }
protected:
    combo_tree* _tr;
    combo_tree::iterator _loc; // location of the knob in the combo_tree
};

struct disc_knob_base : public knob_base {
    disc_knob_base(combo_tree& tr, combo_tree::iterator tgt) 
        : knob_base(tr, tgt) {}
    disc_knob_base(combo_tree& tr) 
        : knob_base(tr) {}
    virtual ~disc_knob_base() {}

    virtual void turn(int) = 0;
    virtual void disallow(int) = 0;
    virtual void allow(int) = 0;

    //create a spec describing the space spanned by the knob
    virtual eda::field_set::disc_spec spec() const = 0;

    //arity based on whatever knobs are currently allowed
    virtual int arity() const = 0;

    //expected complexity based on whatever the knob is currently turned to
    virtual int complexity_bound() const = 0;

    virtual std::string toStr() const = 0;
};

struct contin_knob : public knob_base {
    contin_knob(combo_tree& tr, combo_tree::iterator tgt,
                contin_t step_size, contin_t expansion,
                eda::field_set::arity_t depth)
        : knob_base(tr, tgt), _spec(combo::get_contin(*tgt),
                                    step_size, expansion, depth) { }

    bool in_exemplar() const {
        return true;
    }

    // @todo: it does not go back to the initiale state
    void clear_exemplar() { }

    void turn(eda::contin_t x) {
        *_loc = x;
    }

    //create a spec describing the space spanned by the knob
    const eda::field_set::contin_spec& spec() const {
        return _spec;
    }

    std::string toStr() const {
        stringstream ss;
        ss << "[" << *_loc << "]";
        return ss.str();
    }
protected:
    eda::field_set::contin_spec _spec;
};

template<int MaxArity>
struct knob_with_arity : public disc_knob_base {
    knob_with_arity(combo_tree& tr, combo_tree::iterator tgt) 
        : disc_knob_base(tr, tgt), _default(0) {}
    knob_with_arity(combo_tree& tr) 
        : disc_knob_base(tr), _default(0) {}

    void disallow(int idx) {
        _disallowed[idx] = true;
    }
    void allow(int idx) {
        _disallowed[idx] = false;
    }

    int arity() const {
        return MaxArity -_disallowed.count();
    }

    bool in_exemplar() const {
        return (_default != 0);
    }
protected:
    std::bitset<MaxArity> _disallowed;
    int _default;

    int map_idx(int idx) const {
        if (idx == _default)
            idx = 0;
        else if (idx == 0)
            idx = _default;
        return idx + (_disallowed << (MaxArity - idx)).count();
    }
};

//note - children aren't cannonized when parents are called
struct logical_subtree_knob : public knob_with_arity<3> {
    static const int absent = 0;
    static const int present = 1;
    static const int negated = 2;
    static const std::map<int, string> pos_str;

    logical_subtree_knob(combo_tree& tr, combo_tree::iterator tgt,
                         combo_tree::iterator subtree)
        : knob_with_arity<3>(tr, tr.end()), _current(absent) {
        typedef combo_tree::sibling_iterator sib_it;
        typedef combo_tree::pre_order_iterator pre_it;
        //compute the negation of the subtree
        combo_tree negated_subtree(subtree);
        negated_subtree.insert_above(negated_subtree.begin(), id::logical_not);
        reduct::logical_reduce(1, negated_subtree);

        for (sib_it sib = tgt.begin(); sib != tgt.end();++sib) {
            if (_tr->equal_subtree(pre_it(sib), subtree) ||
                _tr->equal_subtree(pre_it(sib), negated_subtree.begin())) {
                _loc = sib;
                _current = present;
                _default = present;
                return;
            }
        }

        _loc = _tr->append_child(tgt, id::null_vertex);
        _tr->append_child(_loc, subtree);
    }

    int complexity_bound() const {
        return (_current == absent ? 0 : complexity(_loc));
    }

    void clear_exemplar() {
        if (in_exemplar())
            turn(0);
        else
            _tr->erase(_loc);
    }

    void turn(int idx) {
        idx = map_idx(idx);
        OC_ASSERT((idx < 3), "Index greater than 3.");

        if (idx == _current) //already set, nothing to
            return;

        switch (idx) {
        case absent:
            // flag subtree to be ignored with a null_vertex, replace
            // negation if present
            if (_current == negated)
                *_loc = id::null_vertex;
            else
                _loc = _tr->insert_above(_loc, id::null_vertex);
            break;
        case present:
            _loc = _tr->erase(_tr->flatten(_loc));
            break;
        case negated:
            if (_current == present)
                _loc = _tr->insert_above(_loc, id::logical_not);
            else
                *_loc = id::logical_not;
            break;
        }

        _current = idx;
    }

    eda::field_set::disc_spec spec() const {
        return eda::field_set::disc_spec(arity());
    }
    
    string toStr() const {
        stringstream ss;
        ss << "[";
        for(int i = 0; i < arity(); ++i)
            ss << posStr(map_idx(i)) << (i < arity()-1? " " : "");
        ss << "]";
        return ss.str();
    }
protected:
    int _current;
private:
    // return << *_loc or << *_loc.begin() if it is null_vertex
    // if *_loc is a negative literal returns !#n
    // if negated is true a copy of the literal is negated before being printed
    string locStr(bool negated = false) const {
        OC_ASSERT(*_loc != id::null_vertex || _loc.has_one_child(),
                  "if _loc is null_vertex then it must have only one child");
        stringstream ss;
        combo_tree::iterator it;
        if(*_loc == id::null_vertex)
            it = _loc.begin();
        else it = _loc;
        if(is_argument(*it)) {
            argument arg = get_argument(*it);
            if(negated) arg.negate();
            ostream_abbreviate_literal(ss, arg);
        } else {
            ss << (negated? "!" : "") << *it;
        }
        return ss.str();
    }
    // return the name of the position, if it is the current one and
    // tag_current is true then the name is put in parenthesis
    string posStr(int pos, bool tag_current = false) const {
        stringstream ss;
        switch(pos) {
        case absent:
            ss << "nil";
            break;
        case present:
            ss << locStr();
            break;
        case negated:
            ss << locStr(true);
            break;
        }
        return pos == _current && tag_current?
            string("(") + ss.str() + ")" : ss.str();
    }
};

#define MAX_PERM_ACTIONS 128

//note - children aren't cannonized when parents are called
struct action_subtree_knob : public knob_with_arity<MAX_PERM_ACTIONS> {

    typedef combo_tree::pre_order_iterator pre_it;

    action_subtree_knob(combo_tree& tr, combo_tree::iterator tgt,
                        vector<combo_tree>& perms)
        : knob_with_arity<MAX_PERM_ACTIONS>(tr, tr.end()),
          _current(0),  _perms(perms) {

        OC_ASSERT((int)_perms.size() < MAX_PERM_ACTIONS, "Too many perms.");

        for (int i = _perms.size() + 1;i < MAX_PERM_ACTIONS;++i)
            disallow(i);

        _default = 0;
        _current = _default;
        _loc = _tr->append_child(tgt, id::null_vertex);
    }


    int complexity_bound() const {
        return complexity(_loc);
    }

    void clear_exemplar() {
        if (in_exemplar())
            turn(0);
        else
            _tr->erase(_loc);
    }


    void turn(int idx) {
        idx = map_idx(idx);
        OC_ASSERT(idx <= (int)_perms.size(), "Index too big.");

        if (idx == _current) //already set, nothing to
            return;

        if (idx == 0) {
            if (_current != 0) {
                combo_tree t(id::null_vertex);
                _loc = _tr->replace(_loc, t.begin());
            }
        } else {
            pre_it ite = (_perms[idx-1]).begin();
            _loc = _tr->replace(_loc, ite);
        }
        _current = idx;
    }


    eda::field_set::disc_spec spec() const {
        return eda::field_set::disc_spec(arity());
    }

    std::string toStr() const {
        stringstream ss;
        ss << "[" << *_loc << " TODO ]";
        return ss.str();
    }
protected:
    int _current;
    const vector<combo_tree> _perms;
};



//note - children aren't cannonized when parents are called
struct ant_action_subtree_knob : public knob_with_arity<4> {
    static const int none    = 0;
    static const int forward = 1;
    static const int rleft   = 2;
    static const int rright  = 3;

    ant_action_subtree_knob(combo_tree& tr, combo_tree::iterator tgt,
                            combo_tree::iterator subtree)
        : knob_with_arity<4>(tr, tr.end()), _current(none) {

        _default = none;
        _current = _default;

        _loc = _tr->append_child(tgt, id::null_vertex);
        _tr->append_child(_loc, subtree);
    }

    int complexity_bound() const {
        return complexity(_loc);
    }

    void clear_exemplar() {
        if (in_exemplar())
            turn(0);
        else
            _tr->erase(_loc);
    }

    void turn(int idx) {
        idx = map_idx(idx);
        OC_ASSERT((idx < 4), "Index greater than 3.");

        if (idx == _current) //already set, nothing to
            return;

        if (idx == none) {
            if (_current != none)
                _loc = _tr->insert_above(_loc, id::null_vertex);
        } else {
            if (_current == none)
                _tr->erase_children(_loc);

            switch (idx) {
            case forward:
                *_loc = instance(id::move_forward);
                break;

            case rleft:
                *_loc = instance(id::turn_left);
                break;

            case rright:
                *_loc = instance(id::turn_right);
                break;

            default:
                break;
            }
        }

        _current = idx;
    }

    eda::field_set::disc_spec spec() const {
        return eda::field_set::disc_spec(arity());
    }

    std::string toStr() const {
        stringstream ss;
        ss << "[" << *_loc << " TODO ]";
        return ss.str();
    }
protected:
    int _current;
};



struct simple_action_subtree_knob : public knob_with_arity<2> {
    static const int present = 0;
    static const int absent = 1;

    simple_action_subtree_knob(combo_tree& tr, combo_tree::iterator tgt)
        : knob_with_arity<2>(tr, tgt) {

        _current = present;
        _default = present;
    }

    int complexity_bound() const {
        return (_current == absent ? 0 : complexity(_loc));
    }

    void clear_exemplar() {
//      if (in_exemplar())
        turn(0);
//      else
// _tr->erase(_loc);
    }

    void turn(int idx) {
        idx = map_idx(idx);
        OC_ASSERT((idx < 2), "Index greater than 2.");

        if (idx == _current) //already set, nothing to
            return;

        switch (idx) {
        case present:
            _loc = _tr->erase(_tr->flatten(_loc));
            break;
        case absent:
            _loc = _tr->insert_above(_loc, id::null_vertex);
            break;
        }

        _current = idx;
    }

    eda::field_set::disc_spec spec() const {
        return eda::field_set::disc_spec(arity());
    }

    std::string toStr() const {
        stringstream ss;
        ss << "[" << *_loc << " TODO ]";
        return ss.str();
    }
protected:
    int _current;
};

typedef based_variant < boost::variant<logical_subtree_knob,
                                                action_subtree_knob,
                                                simple_action_subtree_knob>,
                                 disc_knob_base > disc_knob;
} //~namespace moses
} //~namespace opencog

#endif

