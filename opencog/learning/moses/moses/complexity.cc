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

#include "complexity.h"
#include "using.h"

namespace opencog { namespace moses {

using namespace opencog::combo;

// For a Boolean formula, the complexity is the neg(# of literals)
// XXX  .. but why?  What is the complexity of contin expressions?
// Expressions containining greater_than_zero, impulse, cond?  I'm
// somwhere confused about how thigs are being measured.   Note that
// when we calculate the complexity of a field_set, then contins
// count for more than one, depending on the number of "digits" in them.
//
// Soooo... Is this complexity supposed to mirror that of the field
// set -- i.e. lots-of-knobs==higher-complexity?  That probably means
// we should count logical and, logical_or, below ..!?!? TODO, clarify.
complexity_t tree_complexity(combo_tree::iterator it)
{
    // base cases
    if (*it == id::logical_true
        || *it == id::logical_false
        || *it == id::null_vertex)
        return 0;

    if (is_argument(*it)
        || is_builtin_action(*it)
        || is_ann_type(*it)
        || is_constant(*it))
        return -1;

    // recursive cases
    if (*it == id::logical_not)
        return tree_complexity(it.begin());

    // div and trigonometric functions have complexity -1
    // But greatere_than_zero, impulse, plus, times are all treated
    // with complexity zero.  Why?  I dunno; maybe because impulse is
    // like an unavoidable type conversion?  Kind of like id::not above ???
    int c = -int(*it==id::div
                 || *it==id::exp
                 || *it==id::log
                 || *it==id::sin
                 || *it==id::rand
                 || *it==id::equ
                 || *it==id::cond);

    for (combo_tree::sibling_iterator sib = it.begin(); sib != it.end(); ++sib)
        c += tree_complexity(sib);
    return c;
}

complexity_t tree_complexity(const combo_tree& tr)
{
    combo_tree::iterator it = tr.begin();
    if (it == tr.end()) return 0;

    return tree_complexity(tr.begin());
}

} // ~namespace moses
} // ~namespace opencog
