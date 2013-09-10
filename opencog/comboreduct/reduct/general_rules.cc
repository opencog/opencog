/*
 * opencog/comboreduct/reduct/general_rules.cc
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
#include "general_rules.h"
#include "../interpreter/eval.h"
#include <opencog/comboreduct/combo/assumption.h>

namespace opencog { namespace reduct {
typedef combo_tree::sibling_iterator sib_it;

/// Flattens all associative functions: f(a,f(b,c)) -> f(a,b,c)
/// Note that level is recursive that is f(a,f(b,f(c,d))) -> f(a,b,c,d)
void level::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (is_associative(*it)) {
        for (sib_it sib = it.begin(); sib != it.end(); )
            if (*sib == *it)
                sib = tr.erase(tr.flatten(sib));
            else
                ++sib;
    }
}

/// Evaluates sub-expressions when possible.
/// If an operator is commutative, op(const,var,const) will become
/// op(op(const,const),var), e.g., +(2,x,1)->+(3,x)
void eval_constants::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (it.is_childless()) {
        if (is_indefinite_object(*it)) {
            //not sure we want that when indefinite_object is random
            vertex_seq empty;
            *it = eval_throws_binding(empty, it);
        }
        return;
    }

    sib_it to;
    if (is_associative(*it)) {
        if (is_commutative(*it)) {
            to = tr.partition(it.begin(), it.end(), is_constant<vertex>);
            int n_consts = distance(it.begin(), to);
            if (n_consts < 2 && (!(n_consts == 1 && it.has_one_child())))
                return;
            if (to != it.end()) {
                tr.reparent(tr.append_child(it, *it), it.begin(), to);
                it = it.last_child();
            }
        }
        else {
            OC_ASSERT(false, "Not implemented yet");
        }
    }
    else {
        for (sib_it sib = it.begin(); sib != it.end(); ++sib)
            if (!is_constant(*sib))
                return;	
    }

    // We pass an empty vertex sequence, as there should be no
    // arguments in the tree below 'it'.  Viz, the only things we
    // expect to evaluate are things like 'not(true)', '*(-1.75 1)'
    // '+(2 0 6)' and so on.  If there are args, then hopefully an
    // exception will be thrown :-)
    //
    // Err, well, its common for knob-building to generate things
    // log(0) or /(1 0) and so we want to catch that inf, and 
    // propagate it along.  (later on, there may be a divde-by-inf
    // or maybe a 0<(-inf) predicate, etc. all of which can be legally
    // evaluated to return valid results.
    vertex_seq empty;
    try {
        *it = eval_throws_binding(empty, it);
    } catch (EvalException e) {
        *it = e.get_vertex();
    };
    tr.erase_children(it);
}

// Reorder children of commutative operators (should be applied upwards)
void reorder_commutative::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (is_commutative(*it))
        tr.sort_on_subtrees(it.begin(), it.end(),
                            opencog::lexicographic_subtree_order<vertex>(),false);
}

// Get rid of subtrees marked with a null_vertex in their roots
void remove_null_vertices::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    // Most nodes take simple lists; but not cond. Cond takes clauses,
    // which are pairs. If we remove the condition, we must also remove
    // the consequent.
    if (*it != id::cond) {
        for (sib_it sib = it.begin(); sib != it.end(); )
            if (*sib == id::null_vertex)
                sib = tr.erase(sib);
            else
                ++sib;
    } else {
        for (sib_it sib = it.begin(); sib != it.end(); )
            if (*sib == id::null_vertex) {
                sib = tr.erase(sib);
                sib = tr.erase(sib);
            }
            else {
                ++sib;
                ++sib;
            }
    }
}

void remove_all_assumptions::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    delete_all_assumptions(tr);
}

} // ~namespace reduct
} // ~namespace opencog

