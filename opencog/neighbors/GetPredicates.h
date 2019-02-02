/*
 * GetPredicates.h
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: William Ma <https://github.com/williampma>
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

#ifndef _OPENCOG_ATOM_UTILS_H
#define _OPENCOG_ATOM_UTILS_H

#include <opencog/atoms/atom_types/atom_types.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/atom_types/types.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * Returns a list of all the EvaluationLinks with Predicates of the given type
 * that the atom specified by 'target' participates in. 
 * (C++ implementation of cog-get-pred)
 *
 * For example, given existence of the following:
 *
 *     EvaluationLink
 *        PredicateNode "IsA"
 *        ListLink
 *           ConceptNode "dog"
 *           ConceptNode "mammal"
 *
 *     EvaluationLink
 *        PredicateNode "IsA"
 *        ListLink
 *           ConceptNode "dog"
 *           ConceptNode "vertebrate"
 *
 *     EvaluationLink
 *        PredicateNode "eats"
 *        ListLink
 *           ConceptNode "dog"
 *           ConceptNode "meat"
 *
 * and, given a get_predicates target of:
 *
 *     ConceptNode "dog"
 *
 * and predicateType PREDICATE_NODE (or "PredicateNode"), then get_predicates
 * returns evaluation links for:
 *  
 *     dog IsA mammal
 *     dog IsA vertebrate
 *     dog eats meat
 *
 * @param target Search for predicates that target.
 * @param predicateType Search only for predicates of this type.
 * @param subClasses Follow subtypes of linkType too.
 */
HandleSeq get_predicates(const Handle& target, 
                         Type predicateType=PREDICATE_NODE,
                         bool subClasses=true);

/**
 * Returns a list of all the EvaluationLinks with the specified predicate
 * that the atom specified by 'target' participates in. 
 *
 * For example, given the following:
 *
 *     EvaluationLink
 *        PredicateNode "IsA"
 *        ListLink
 *           ConceptNode "dog"
 *           ConceptNode "mammal"
 *
 *     EvaluationLink
 *        PredicateNode "IsA"
 *        ListLink
 *           ConceptNode "dog"
 *           ConceptNode "vertebrate"
 *
 *     EvaluationLink
 *        PredicateNode "eats"
 *        ListLink
 *           ConceptNode "dog"
 *           ConceptNode "meat"
 *
 * then, given a target of:
 *
 *     ConceptNode "dog"
 *
 * and predicate:
 *
 *     PredicateNode "IsA"
 *
 * then get_predicates_for returns a list of all of the EvaluationLinks in 
 * which the dog has the "IsA" predicate, but not the one for "eats". So:
 * 
 *     dog IsA mammal 
 *     dog IsA vertebrate
 *
 * but NOT:
 *
 *     dog eats meat
 *
 * @param target Search for predicates that target.
 * @param predicateType Search only for predicates of this type.
 * @param subClasses Follow subtypes of linkType too.
 */
HandleSeq get_predicates_for(const Handle& target, 
                             const Handle& predicate);

/** @}*/
}


#endif // _OPENCOG_ATOM_UTILS_H
