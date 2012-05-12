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
            pre_it val = it.begin();
            *it = *val;
            tr.erase(tr.flatten(val));
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

// cond(... p x q x ...) -> cond (... or(p q) x ...)
void reduce_cond_adjacent::operator()(combo_tree& tr,
                                      combo_tree::iterator it) const
{
    if (*it != id::cond) return;

    // Two clauses, plus the else cklause means at least 5 terms for
    // this to even make sense.
    size_t sz = tr.number_of_children(it);
    sib_it sib = it.begin();
    for (; 5 <= sz; sz -= 2) {

        pre_it c1 = sib;
        sib_it q1 = ++sib;
        pre_it c2 = ++sib;
        sib_it q2 = c2; q2++;

        // If the two consequents aren't equal, try the next pair.
        if (!tr.equal_subtree(q1, q2)) continue;
        
        pre_it por = tr.insert(c1, id::logical_or);

        pre_it new1 = tr.append_child(por, *c1);
        tr.reparent(new1, c1.begin(), c1.end());
        tr.erase(c1);
        
        pre_it new2 = tr.append_child(por, *c2);
        tr.reparent(new2, c2.begin(), c2.end());
        tr.erase(c2);
        
        tr.erase(q2);

        // Lost track of things; recurse to find more, if any. 
        operator()(tr, it);
    }
}

#if UNDER_CONSTRUCTION
// cond(true x1 ... pn xn y) -> x1
// cond(false x1 ... pn xn y) -> cond(p2 x2 .. pn xn y)
void reduce_cond_const::operator()(combo_tree& tr,
                                   combo_tree::iterator it) const
{
    if ((*it != id::cond) && (*it != id::contin_if)) return;
    while (1) {
        pre_it cond = tr.child(it, 0);
        pre_it quent = tr.child(it, 1);
        if (*cond == id::logical_true) {
            *it = *quent;
            tr.flatten(quent);
            tr.erase(cond);

            // Erase everything else that follows.
            for (sib_it sib = ++quent; sib != it.end(); sib++) {
                tr.erase(sib);
            }
        }
    }
}
#endif

} // ~namespace reduct
} // ~namespace opencog
