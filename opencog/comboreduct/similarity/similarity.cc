/*
 * opencog/comboreduct/similarity/similarity.cc
 *
 * Copyright (C) 2014 Aidyia Limited
 * All Rights Reserved
 *
 * Written by Linas Vepstas
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

#include <opencog/comboreduct/combo/iostream_combo.h>

#include "similarity.h"

namespace opencog { namespace combo {

using namespace std;
/**
 * This file implements the tree similarity measure described by:
 * Rui Yang, Panos Kalnis, Anthony K. H. Tung
 * "Similarity Evaluation on Tree-structured Data"
 * appearning in SIGMOD 2005.
 * http://www.comp.nus.edu.sg/~atung/renmin/treematch.pdf
 *
 * There are various changes and departures form the algo described
 * above:
 * 1) We never need the binary tree version of a tree, and so never
 *    generate that.  Instead, we generate the branch-tree vector
 *    directly.
 * 2) Empty nodes are never generated; they're not needed.
 * 3) The code here assumes that any N-ary operator for N>2 can
 *    be decomposed into a binary tree in a fully associative way.
 *    For the combo arithmetic and boolean operators, this is
 *    appropriate, since, of course, +(a b c) == +(a +(b c))
 *    At time of writing, all combo N-ary operators are associative
 *    in this way.
 * 4) We don't actually need a C++ vector to hold the vector. An
 *    std::map is easier.
 */

/**
 * tree_flatten -- approximate combo tree by a flattened vector
 *
 * Implicitly convert the tree to binary form, and then count the 
 * number of vertexes of each type in that binary tree.
 *
 * (The binary tree equivalent of a combo tree is a tree where every
 * vertex has at most two children, but is otherwise semantically
 * equivalent).
 */
static void tree_flatten_rec(tree_branch_vector& ctr, combo_tree::iterator root)
{
	size_t numch = root.number_of_children();
	if (0 == numch)
	{
		stringstream ss;
		ss << *root;
		ctr[ss.str()] += 1;
		return;
	}
	if (1 == numch)
	{
		combo_tree::sibling_iterator first = root.begin();
		stringstream ss;
		ss << *root << "(" << *first << ")";
		ctr[ss.str()] += 1;

		if (0 != first.number_of_children())
			tree_flatten_rec(ctr, first);

		return;
	}

	combo_tree::sibling_iterator first = root.begin();
	size_t i = 0;
	for (; i < numch-2; first++, i++)
	{
		stringstream ss;
		ss << *root << "(" << *first << " " << *root << ")";
		ctr[ss.str()] += 1;

		if (0 != first.number_of_children())
			tree_flatten_rec(ctr, first);
	}

	combo_tree::sibling_iterator second = first;
	second ++;

	stringstream ss;
	ss << *root << "(" << *first << " " << *second << ")";
	ctr[ss.str()] += 1;

	if (0 != first.number_of_children())
		tree_flatten_rec(ctr, first);

	if (0 != second.number_of_children())
		tree_flatten_rec(ctr, second);
}

tree_branch_vector tree_flatten(const combo_tree& tree)
{
	tree_branch_vector counter;
	combo_tree::iterator root = tree.begin();
	tree_flatten_rec(counter, root);
	return counter;
}

tree_branch_vector tree_flatten(const std::string& str)
{
	stringstream ss;
	combo_tree tr;
	ss << str;
	ss >> tr;
	return tree_flatten(tr);
}


std::ostream& operator<<(std::ostream& os, const tree_branch_vector& btv)
{
	bool nf = false;
	foreach(auto pr, btv) {
		if (nf) os << ", "; else nf = true;
		os << pr.first << " : " << pr.second;
	}
	return os;
}


tree_similarity::tree_similarity(void)
{

}

} // ~namespace combo
} // ~namespace opencog
