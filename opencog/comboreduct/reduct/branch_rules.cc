/*
 * opencog/comboreduct/reduct/branch_rules.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings
 * All Rights Reserved
 *
 * Written by Nil Geisweiller, Linas Vepstas
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
#include "branch_rules.h"
#include <opencog/comboreduct/combo/type_tree.h>
#include <opencog/util/exceptions.h>

namespace opencog { namespace reduct {

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;


// Apply boolean reduction to the argument of impulse.
void reduce_cond_arg::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it != id::cond) return;

    // Every other one is a condition, except for the last one.
    size_t nc = tr.number_of_children(it);
    for (sib_it sib = it.begin(); 3 <= nc; nc -= 2) {
        logical_reduce(reduct_effort, tr, sib, ignore_ops);
        sib++;
        sib++;
    }
}


// cond(v) -> v
// cond(p1 x1 ... pn xn p x x) -> cond(p1 x1 ... pn xn x)
//
// This obsoletes reduce_contin_if_equal_branch
void reduce_cond_else::operator()(combo_tree& tr,
                                  combo_tree::iterator it) const
{
    if ((*it != id::cond) && (*it != id::contin_if)) return;

    while (1) {
        size_t last = tr.number_of_children(it);

        // cond(v) -> v
        if (1 == last) {
            *it = *(it.begin());
            tr.erase(tr.flatten(it.begin()));
            return;
        }

        // Look at the last two consequents.
        // cond(p1 x1 ... pn xn p x x) -> cond(p1 x1 ... pn xn x)
        last -= 3;
        pre_it cond = tr.child(it, last);
        pre_it c1 = tr.child(it, last+1);
        pre_it c2 = tr.child(it, last+2);
        if (tr.equal_subtree(c1, c2)) {
            tr.erase(cond);
            *cond = *c1;
            tr.erase(c2);
        } else
            break;
    }
}


} // ~namespace reduct
} // ~namespace opencog
