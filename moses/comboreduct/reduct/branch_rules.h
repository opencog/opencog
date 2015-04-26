/*
 * opencog/comboreduct/reduct/branch_rules.h
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
#ifndef _REDUCT_BRANCH_RULES_H
#define _REDUCT_BRANCH_RULES_H

#include "reduct.h"

namespace opencog { namespace reduct {

/// Apply boolean reduction to each predicate
/// (same class structure as reduce_impulse_arg)
struct reduce_cond_arg : public crule<reduce_cond_arg>
{
    int reduct_effort;
    const vertex_set &ignore_ops;
    reduce_cond_arg(int effort, const vertex_set &igop)
        : crule<reduce_cond_arg>::crule("reduce_cond_arg"),
          reduct_effort(effort), ignore_ops(igop) {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
}; 


/// If the last two consequents are identical, omit the last clause.
/// cond(p1 x1 ... pn xn p x x) -> cond(p1 x1 ... pn xn x)
/// cond(v) -> v
struct reduce_cond_else : public crule<reduce_cond_else> 
{
    reduce_cond_else() 
        : crule<reduce_cond_else>::crule("reduce_cond_else") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};


/// If two neighboring consequents are identical, merge them.
/// cond(... p x q x ...) -> cond (... or(p q) x ...)
struct reduce_cond_adjacent : public crule<reduce_cond_adjacent> 
{
    reduce_cond_adjacent() 
        : crule<reduce_cond_adjacent>::crule("reduce_cond_adjacent") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

/// Reduce cond's with constant predicates.
///
/// Any predicate that is true truncates the rest of the condition.
/// cond(true x1 ... pn xn y) -> x1
/// cond(p1 x1 ... true xk ... pn xn y) -> cond(p1 x1 ... p{k-1} x{k-1}
/// xk)
///
/// Any predicate that is false causes the whole clause to fall out.
/// cond(false x1 ... pn xn y) -> cond(p2 x2 ... pn xn y)
/// cond(p1 x1 ... false xk ... pn xn y) -> 
///                 cond(p1 x1 ... p{k-1} x{k-1} p{k+1} x{k+1} ... pn xn
///                 y)
///
/// This can happen along the way.
/// cond(v) -> v
///
/// This partially obsoletes reduce_contin_if
struct reduce_cond_const : public crule<reduce_cond_const> 
{
    reduce_cond_const() 
        : crule<reduce_cond_const>::crule("reduce_cond_const") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};


} // ~namespace reduct
} // ~namespace opencog

#endif
