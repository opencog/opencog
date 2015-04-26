/*
 * opencog/comboreduct/reduct/perception_rules.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _REDUCT_PERCEPTION_RULES_H
#define _REDUCT_PERCEPTION_RULES_H

#include "reduct.h"
#include <opencog/comboreduct/type_checker/type_tree.h>

namespace opencog { namespace reduct {
  
//add in the set of assumptions such knowledge
//f(x, z) <= max{d(x, y), d(y, z)}
//when encounting f(x, y) and f(y, z) and d is ultrametric
struct reduce_ultrametric : public crule<reduce_ultrametric> {
    reduce_ultrametric() : crule<reduce_ultrametric>::crule("reduce_ultrametric") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};
 
//add in the set of assumptions such knowledge
//f(x, z)
//when encounting f(x, y) and f(y, z)
struct reduce_transitive : public crule<reduce_transitive> {
    reduce_transitive() : crule<reduce_transitive>::crule("reduce_transitive") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//reduce f(x,x) -> true iff f is reflexive
struct reduce_reflexive : public crule<reduce_reflexive> {
    reduce_reflexive() : crule<reduce_reflexive>::crule("reduce_reflexive") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//reduce f(x,x) -> false iff f is irreflexive
struct reduce_irreflexive : public crule<reduce_irreflexive> {
    reduce_irreflexive() : crule<reduce_irreflexive>::crule("reduce_irreflexive") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};


//add f(y,x) in the set of assumptions if f(x,y)
//Note that although it seems that since a symmetric
//operator is commutative there is no need of this rule
//it is not true because this rule permits to combines
//different properties of a relation, as for instance
//if the relation is both symmetric and transitive
//then the combination of reduce_transitive and
//reduce_symmetric can permit more reductions
//than with associative reduction.
struct reduce_symmetric : public crule<reduce_symmetric> {
    reduce_symmetric() : crule<reduce_symmetric>::crule("reduce_symmetric") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};


//reduce f(x,y) -> 0 iff f verifies the property of identity of indiscernibles
struct reduce_identity_of_indiscernibles : public crule<reduce_identity_of_indiscernibles> {
    reduce_identity_of_indiscernibles() : crule<reduce_identity_of_indiscernibles>::crule("reduce_identity_of_indiscernibles") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

} // ~namespace reduct
} // ~namespace opencog

#endif
