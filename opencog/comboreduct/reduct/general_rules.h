/*
 * opencog/comboreduct/reduct/general_rules.h
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
#ifndef _REDUCT_GENERAL_RULES_H
#define _REDUCT_GENERAL_RULES_H

#include <opencog/util/RandGen.h>

#include "reduct.h"
#include <opencog/comboreduct/combo/eval.h>

namespace opencog { namespace reduct {

//flattens all associative functions: f(a,f(b,c)) -> f(a,b,c)
//note that level is recursive that is f(a,f(b,f(c,d))) -> f(a,b,c,d)
struct level : public crule<level> { 
    level() : crule<level>::crule("level") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//evaluates sub-expressions when possible
//if an operator is commutative, op(const,var,const) will become
//op(op(const,const),var), e.g., +(2,x,1)->+(3,x)
struct eval_constants : public crule<eval_constants> { 
    opencog::RandGen& rng;
    Evaluator* evaluator;
    eval_constants(opencog::RandGen& _rng, Evaluator* e = NULL)
        : crule<eval_constants>::crule("eval_constants"),
          rng(_rng), evaluator(e) {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//Reorder children of commutative operators (should be applied upwards)
struct reorder_commutative : public crule<reorder_commutative> {
    reorder_commutative() : 
        crule<reorder_commutative>::crule("reorder_commutative") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//Get rid of subtrees marked with a null_vertex in their roots
struct remove_null_vertices : public crule<remove_null_vertices> {
    remove_null_vertices() : crule<remove_null_vertices>::crule("remove_null_vertices") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//Remaove all assumptions
struct remove_all_assumptions : public crule<remove_all_assumptions> {
    remove_all_assumptions() : crule<remove_all_assumptions>::crule("remove_all_assumptions") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

} // ~namespace reduct
} // ~namespace opencog

#endif
