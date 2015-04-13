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
#include "../type_checker/type_tree.h"
#include <opencog/util/exceptions.h>

namespace opencog { namespace reduct {

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;


// Apply boolean reduction to the predicates in a cond
void reduce_cond_arg::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it != id::cond) return;

    // Every other one is a predicate, except for the last one.
    size_t nc = tr.number_of_children(it);
    for (sib_it sib = it.begin(); 3 <= nc; nc -= 2) {
        logical_reduce(reduct_effort, tr, sib, ignore_ops);
        ++sib;
        ++sib;
    }
}

// cond(v) -> v
static inline void zap_one(combo_tree& tr, combo_tree::iterator it)
{
    pre_it top = it;
    *it = *it.begin();  // it may be head of the tree...
    tr.erase(++top);
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
            zap_one(tr, it);
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

    // Two clauses, plus the else clause means at least 5 terms for
    // this to even make sense.
    size_t sz = tr.number_of_children(it);
    sib_it sib = it.begin();
    for (; 5 <= sz; sz -= 2) {

        sib_it p = sib;
        sib_it x1 = ++sib;
        sib_it q = ++sib;
        sib_it x2 = q;  x2++;

        // If the two consequents aren't equal, try the next pair.
        if (!tr.equal_subtree(x1, x2)) continue;
        
        pre_it por = tr.insert_above(p, id::logical_or);
        tr.move_after(por.begin(), q);

        sib = x2;
        ++sib;
        sz -= 2;

        tr.erase(x2);
    }
}

// cond(true x1 ... pn xn y) -> x1
// cond(p1 x1 ... true xk ... pn xn y) -> cond(p1 x1 ... p{k-1} x{k-1} xk)
//
// cond(false x1 ... pn xn y) -> cond(p2 x2 ... pn xn y)
// cond(p1 x1 ... false xk ... pn xn y) -> 
//                 cond(p1 x1 ... p{k-1} x{k-1} p{k+1} x{k+1} ... pn xn y)
void reduce_cond_const::operator()(combo_tree& tr,
                                   combo_tree::iterator it) const
{
    if ((*it != id::cond) && (*it != id::contin_if)) return;

    sib_it sib = it.begin();
    size_t num = tr.number_of_children(it);
    while (1) {

        // cond(v) -> v
        if (1 == num) {
            zap_one(tr, it);
            return;
        }

        pre_it pred = sib;
        pre_it quent = ++sib;
        if (sib == it.end()) return;
        ++sib;

        // Truncate everything after the true predicate
        if (*pred == id::logical_true) {
            tr.erase(pred);
            num --;

            // Erase everything else that follows.
            for (; sib != it.end(); ++sib) {
                tr.erase(sib);
                num --;
            }

            if (num == 1) zap_one(tr, it);
            return;
        }

        // Eliminate the false clause
        if (*pred == id::logical_false) {
            tr.erase(pred);
            tr.erase(quent);
            num -= 2;
        }
    }
}

} // ~namespace reduct
} // ~namespace opencog
