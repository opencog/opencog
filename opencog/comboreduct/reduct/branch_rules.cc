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


// cond(p1 x1 ... pn xn p x x) -> cond(p1 x1 ... pn xn x)
//
// This obsoletes reduce_contin_if_equal_branch
void reduce_cond_else::operator()(combo_tree& tr,
                                  combo_tree::iterator it) const
{
    if (*it == id::cond ||
        *it == id::contin_if)
    {
        size_t last = tr.number_of_children(it);
        if (last < 3) return;
        last -= 3;
        pre_it cond = tr.child(it, last);
        pre_it b1 = tr.child(it, last+1);
        pre_it b2 = tr.child(it, last+2);
        if (tr.equal_subtree(b1, b2)) {
            *it = *b1;
            tr.erase(tr.flatten(b1));
            tr.erase(cond);
            tr.erase(b2);
        }
    }
}


} // ~namespace reduct
} // ~namespace opencog
