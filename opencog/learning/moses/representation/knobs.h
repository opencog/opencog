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
#include <string>

#include <opencog/util/based_variant.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/numeric.h>
#include <opencog/util/tree.h>

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/combo/iostream_combo.h>

#include "../moses/complexity.h"
#include "field_set.h"

namespace opencog { namespace moses {

using namespace combo;

// A knob represents a single dimension of variation relative to an exemplar
// program tree. This may be discrete or continuous. In the discrete case, the
// various settings are accessible via turn(0), turn(1),...turn(multiplicity()-1).
// In the continuous case, turn(contin_t) is used.
//
// For example, given the program tree fragment or(0<(*($1,0.5)),$2), a
// continuous knob might be used to vary the numerical constant. So setting
// this knob to 0.7 would transform the tree fragment to
// or(0<(*($1,0.7)),$2). A discrete knob with multiplicity()==3 might be used to
// transform the boolean input $2. So setting this knob to 1 might transform
// the tree to or(0<(*($1,0.7)),not($2)), and setting it to 2 might remove it
// from the tree (while setting it to 0 would return to the original tree).

struct knob_base
{
    knob_base(combo_tree& tr, combo_tree::iterator loc)
        : _tr(tr), _loc(loc) {}
    knob_base(combo_tree& tr) : _tr(tr), _loc(tr.end()) {}
    virtual ~knob_base() { }

    // Is the feature nonzero by default? i.e., is it present in the exemplar?
    virtual bool in_exemplar() const = 0;

    // Return the exemplar to its state before the knob was created
    // (deleting any null vertices if present).
    virtual void clear_exemplar() = 0;

    combo_tree::iterator get_loc() const
    {
        return _loc;
    }

    virtual std::string toStr() const = 0;

protected:
    combo_tree& _tr;
    combo_tree::iterator _loc; // location of the knob in the combo_tree
};

struct disc_knob_base : public knob_base
{
    disc_knob_base(combo_tree& tr, combo_tree::iterator tgt)
        : knob_base(tr, tgt) {}
    disc_knob_base(combo_tree& tr)
        : knob_base(tr) {}
    virtual ~disc_knob_base() {}

    virtual void turn(int) = 0;
    virtual void disallow(int) = 0;
    virtual void allow(int) = 0;

    /**
     * Append the right content (determined by idx) to parent_dst. If
     * candidate is empty then it sets the content as head. If
     * anything is appended then parent_dst will be overwritten with
     * the iterator pointing to that new content.
     *
     * @param candidate  candidate destination
     * @param parent_dst iterator pointing to the parent of the future
     *                   node to be appended. Then the iterator
     *                   pointing to that new content (if any) is
     *                   copied in parent_dst
     * @param idx        disc idx
     *
     * @return iterator pointing to the next node of the exemplar to
     * copy.
     */
    virtual combo_tree::iterator append_to(combo_tree& candidate,
                                           combo_tree::iterator& parent_dst,
                                           int idx) const = 0;

    // Create a spec describing the space spanned by the knob.
    virtual field_set::disc_spec spec() const = 0;

    // Arity based on whatever knobs are currently allowed.
    virtual int multiplicity() const = 0;

    /// Expected complexity based on whatever the knob is currently
    /// turned to.   XXX This method is never called by anyone, at this
    /// time ...
    virtual complexity_t complexity_bound() const = 0;
};

struct contin_knob : public knob_base
{
    contin_knob(combo_tree& tr, combo_tree::iterator tgt,
                contin_t step_size, contin_t expansion,
                field_set::width_t depth);

    bool in_exemplar() const;

    // @todo: it does not go back to the initiale state
    void clear_exemplar();

    void turn(contin_t x);

    /**
     * Append the content (a contin value) as child of parent_dst. If
     * candidate is empty then it sets the content as head.
     *
     * @param candidate  candidate destination
     * @param parent_dst iterator pointing to the parent of the future
     *                   node to be appended.
     * @param c          contin constant to be append
     */
    void append_to(combo_tree& candidate, combo_tree::iterator parent_dst,
                   contin_t c) const;

    // Return the spec describing the space spanned by the knob
    // Note that this spec is *not* a part of the field set that is
    // being used by the representation!
    const field_set::contin_spec& spec() const;

    std::string toStr() const;

protected:
    field_set::contin_spec _spec;
};

//* A discrete_knob is a knob with a finite number of different,
// discrete settings. The total number of possible settings is called
// the "Multiplicity" of the knob.  Zero is always a valid knob setting.
//
// Some knob settings can be disalowed at runtime, and thus, the
// effective multiplicity can be less than that declared at compile-time.
//
template<int Multiplicity>
struct discrete_knob : public disc_knob_base
{
    discrete_knob(combo_tree& tr, combo_tree::iterator tgt)
        : disc_knob_base(tr, tgt), _default(0), _current(0) {}
    discrete_knob(combo_tree& tr)
        : disc_knob_base(tr), _default(0), _current(0) {}

    /// Do not allow setting to be set.
    void disallow(int setting) {
        _disallowed[setting] = true;
    }
    void allow(int setting) {
        _disallowed[setting] = false;
    }

    int multiplicity() const {
        return Multiplicity -_disallowed.count();
    }

    bool in_exemplar() const {
        return (_default != 0);
    }

protected:
    std::bitset<Multiplicity> _disallowed;
    int _default;
    int _current;   // The current knob setting.

    // XXX Huh?? what does this do?? Why does shifting matter,
    // if the only thing done is to count the number of bits set ??
    // WTF ??  I think the shift is n the  wrong direction, yeah?
    // If the goal is to skip over index values that are disallowed, then
    // the shift is definitely in the wrong direction!! FIXME.
    int map_idx(int idx) const
    {
        if (idx == _default)
            idx = 0;  // interpreted as "absent", as used below. XXX
        else if (idx == 0)
            idx = _default;
        return idx + (_disallowed << (Multiplicity - idx)).count();
    }
};

// A unary function knob: this knob negates (or not) a boolean-valued
// tree underneath it.
//
// XXX This uses reduct::logical_reduction rules; it is not clear if those
// rules tolerate predicates.
//
// XXX what is the difference between "present" and "absent" ??? A
// knob that is "absent" from a logical "or" is the same as "present
// and false".  while one that is absent from a logical "and" is the
// same as "present and true" So I think this is a bit confusing ...
// I think that a better implementation might have four settings:
// "invert", "identity", "always true" and "always false".  So,
// overall, this is confusing without some sort of additional
// justification.
//
// XXX Also -- I think I want to rename this to "logical unary knob",
// or something like that, as it is a unary logical function ... err...
// well, I guess all combo opers are unary, due to Currying.
//
// note - children aren't canonized when parents are called (??? huh ???)
struct logical_subtree_knob : public discrete_knob<3>
{
    static const int absent = 0;
    static const int present = 1;
    static const int negated = 2;
    static const std::map<int, std::string> pos_str;

    // copy lsk on tr at position tgt
    logical_subtree_knob(combo_tree& tr, combo_tree::iterator tgt,
                         const logical_subtree_knob& lsk);

    logical_subtree_knob(combo_tree& tr, combo_tree::iterator tgt,
                         combo_tree::iterator subtree);

    complexity_t complexity_bound() const;

    void clear_exemplar();

    void turn(int idx);

    combo_tree::iterator append_to(combo_tree& candidate,
                                   combo_tree::iterator& parent_dst,
                                   int idx) const;

    field_set::disc_spec spec() const;

    std::string toStr() const;
private:
    // return << *_loc or << *_loc.begin() if it is null_vertex
    // if *_loc is a negative literal returns !$n
    // if negated is true a copy of the literal is negated before being printed
    std::string locStr(bool negated = false) const;

    // Return the name of the position, if it is the current one and
    // tag_current is true then the name is put in parenthesis.
    std::string posStr(int pos, bool tag_current = false) const;
};

#define MAX_PERM_ACTIONS 128

// Note - children aren't canonized when parents are called.
// XXX what does the above comment mean ???
struct action_subtree_knob : public discrete_knob<MAX_PERM_ACTIONS>
{
    typedef combo_tree::pre_order_iterator pre_it;

    action_subtree_knob(combo_tree& tr, combo_tree::iterator tgt,
                        const vector<combo_tree>& perms);

    complexity_t complexity_bound() const;

    void clear_exemplar();

    void turn(int idx);

    combo_tree::iterator append_to(combo_tree& candidate,
                                   combo_tree::iterator& parent_dst,
                                   int idx) const;

    field_set::disc_spec spec() const;

    std::string toStr() const;
protected:
    const vector<combo_tree> _perms;
};

// Binary knob for action subtree, present or absent
struct simple_action_subtree_knob : public discrete_knob<2>
{
    static const int present = 0;
    static const int absent = 1;

    simple_action_subtree_knob(combo_tree& tr, combo_tree::iterator tgt);

    complexity_t complexity_bound() const;

    void clear_exemplar();

    void turn(int idx);

    combo_tree::iterator append_to(combo_tree& candidate,
                                   combo_tree::iterator& parent_dst,
                                   int idx) const;

    field_set::disc_spec spec() const;

    std::string toStr() const;

private:
    // return << *_loc or << *_loc.begin() if it is null_vertex
    std::string locStr() const;
};

// The disc_knob may be any one of a number of different discrete
// knob types.
typedef based_variant <boost::variant<logical_subtree_knob,
                                      action_subtree_knob,
                                      simple_action_subtree_knob>,
                       disc_knob_base> disc_knob;

// Without this helper, the compiler gets confused and tries to cast
// knob_base to something strange.
inline std::ostream& operator<<(std::ostream& out,
                                const opencog::moses::knob_base& s)
{
	return out << s.toStr();
}

} //~namespace moses
} //~namespace opencog

#endif

