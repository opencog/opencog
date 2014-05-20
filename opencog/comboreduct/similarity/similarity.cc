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
	cout << "duuude root " << *root << endl;

	int numch = root.number_of_children();
	if (0 == numch)
	{
		stringstream ss;
		ss << combo_tree(root);
		ctr[ss.str()] += 1;
		return;
	}
	else if (1 == numch)
	{
		combo_tree::sibling_iterator first = root.begin();
		if (0 == first.number_of_children())
		{
			stringstream ss;
			ss << combo_tree(root);
			ctr[ss.str()] += 1;
			return;
		}
	}
	else if (2 == numch)
	{
		combo_tree::sibling_iterator first = root.begin();
		if (0 == first.number_of_children() and
		    0 == (++first).number_of_children())
		{
			stringstream ss;
			ss << combo_tree(root);
			ctr[ss.str()] += 1;
			return;
		}
	}

	combo_tree::sibling_iterator it = root.begin();
	combo_tree::sibling_iterator last = root.end();
	for (; it !=last; it++)
	{
		cout << "duuude child " << *it << endl;
	}
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
		os << pr.first << ": " << pr.second;
	}
	return os;
}


tree_similarity::tree_similarity(void)
{

}

} // ~namespace combo
} // ~namespace opencog
