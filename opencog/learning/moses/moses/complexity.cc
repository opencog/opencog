/*
 * opencog/comboreduct/combo/complexity.cc
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
#include <opencog/util/exceptions.h>
#include <opencog/comboreduct/combo/combo.h>

#include "complexity.h"

namespace opencog { namespace moses {

using namespace opencog::combo;

// For a Boolean formula, the complexity is the neg(# of literals)
// That is, a Boolean formula can always be reduced to CNF or DNF,
// and, in that case, the complexity is "almost" a linear function
// of the number of arguments -- well, at least, its sub-quadratic,
// if we did decide to count the operators.  (XXX But why  not count
// the operators?  For ENF (elegant normal form) there should be even
// fewer operators than either CNF or DNF, so counting operators
// doesn't seem wrong to me ...)
//
// XXX What is the complexity of contin expressions?
// Expressions containining greater_than_zero, impulse, cond?  I'm
// somwhat confused about how thigs are being measured.   Note that
// when we calculate the complexity of a field_set, then contins
// count for more than one, depending on the number of "digits" in them.
// And, in a field set, operators are disc_knobs, and so they are
// counted there ...
//
// Soooo... Is this complexity supposed to mirror that of the field
// set -- i.e. lots-of-knobs==higher-complexity?  That probably means
// we should count logical and, logical_or, below ..!?!? TODO, clarify.
// (grep for "information_theoretic_bits" in optimization.h)
//
// Note Bene: this function returns a POSITIVE number!
complexity_t tree_complexity(combo_tree::iterator it,
                             bool (*stopper)(const combo_tree::iterator&))
{
    // base cases
    // null_vertex marks the location of a logical knob.  Halt 
    // recursion past logical knobs.
    if (*it == id::logical_true
        || *it == id::logical_false
        || *it == id::null_vertex)
        return 0;

    // If the stopper function is defined, and it returns true, we halt
    // recursion.  This is needed for knob-probing across boundaries
    // between logical and contin expressions.
    if (stopper && stopper(it)) return 0;

    // *(0 stuff) marks the location of a contin knob.  Halt recursion
    // past contin knobs.  This is for knob-probing.
    if ((*it==id::times) && is_contin(*it.begin()) && 
        (0 == get_contin(*it.begin())))
        return 0;

    // Contins get a complexity of 1. But perhaps, contins should
    // get a complexity of 2 or more, if they are very large, or 
    // require many digits of precision.
    if (is_argument(*it)
        || is_contin(*it)
        || is_builtin_action(*it)
        || is_ann_type(*it)
        || is_action_result(*it))
        return 1;

    // recursive cases
    if (*it == id::logical_not)
        return tree_complexity(it.begin(), stopper);

    // If an operator is not listed below, it has a complexity of zero.
    // Note that logical_and, logical_or are not listed, these have a
    // complexity of zero.
    //
    // div and trigonometric functions have complexity one.
    // But greatere_than_zero, impulse, plus, times are all treated
    // with complexity zero.  Why?  I dunno; maybe because impulse is
    // like an unavoidable type conversion?  Kind of like id::not above ???
    int c = int(*it==id::div
                 || *it==id::exp
                 || *it==id::log
                 || *it==id::sin
                 || *it==id::rand
                 || *it==id::equ
                 || *it==id::cond);

    for (combo_tree::sibling_iterator sib = it.begin(); sib != it.end(); ++sib)
        c += tree_complexity(sib, stopper);
    return c;
}

complexity_t tree_complexity(const combo_tree& tr,
                             bool (*stopper)(const combo_tree::iterator&))
{
    combo_tree::iterator it = tr.begin();
    if (it == tr.end()) return 0;

    return tree_complexity(tr.begin(), stopper);
}

} // ~namespace moses
} // ~namespace opencog
