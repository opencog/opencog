/*
 * opencog/comboreduct/combo/assumptions.h
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
#ifndef _COMBO_ASSUMPTION_H
#define _COMBO_ASSUMPTION_H

#include "vertex.h"

namespace opencog { namespace combo {

//This file contains all function required to treat assumptions in combo_tree

//The assumption set of a combo_tree are all children (subtrees) of head
//except the first child designing the "conclusion" tree or main tree.
//It is assumed that the assumption set is lexicographically ordered
//this assumption is verified because insert_assumption is the only way
//to add an element in that set and respect that order.

//insert_assumption
//inserts a subtree in the assumption set of a combo_tree
//Note it is a set so identical subtree are represented only once
//tr designs the tree where to insert the assumption
//assum_tr designs the tree where the assumption is taken from
//assum_it the root of the assumption
//It is assumed that the tree is not empty
void insert_assumption(combo_tree& tr, combo_tree::iterator assum_it);

//put in res all interators that have v as content in all expressions
//if nothing was found return false
//otherwise return true
//of all assumptions
//it is assumed that the tree is not empty
bool find_vertices_in_assumptions(const combo_tree& tr, vertex v,
                                  std::vector<combo_tree::iterator>& res);

//check if all assumptions are the same in tr1 and tr2
//assumed that tr1 and tr2 are not empty
bool equal_assumptions(const combo_tree& tr1, const combo_tree& tr2);

//delete all assumptions
//it is assumed that the tree is not empty
void delete_all_assumptions(combo_tree& tr);

}} // ~namespaces combo opencog

#endif
