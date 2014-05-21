/*
 * opencog/comboreduct/combo/similarity.cc
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

/**
 * This file implements a combo tree similarity measure, obtained by
 * directly comparing combo trees.  It differs strongly from a
 * behavioral-score based similarity measure, in the two xausual ways:
 * two very different trees could produce very similar or even identical
 * behavioural scores, and two very similar trees could have very
 * wildly different scores.  Thus, the similarity measured here provides
 * a different viewpoint to a behavioral-score similarity. Roughly
 * speaking, its like comparing genotypes instead of pheontypes.
 *
 * The tree similarity measure implemented here is described by:
 * Rui Yang, Panos Kalnis, Anthony K. H. Tung
 * "Similarity Evaluation on Tree-structured Data"
 * appearning in SIGMOD 2005.
 * http://www.comp.nus.edu.sg/~atung/renmin/treematch.pdf
 *
 * Roughly, the algo words as follows:
 *
 * A) Given a combo expression such as "+(a b c d)", convert it to
 *    a binary tree: e.g. "+(a +(b +(c d)))"  This proceedure is
 *    applied recursively to subtrees.
 *
 * B) Define a "tree branch" as a node and its two immediate children.
 *    The above example has the following tree branches: "+(a +)"
 *    and "+(b +)" and "+(c d)". For this example, each of thes occurs
 *    only once.
 *
 * C) Count how many times each kind of tree branch occurs in the binary
 *    tree. This dictionary of branch-type vs. count can be thought of
 *    as a vector if it was sorted, e.g. lexicographically.
 *
 * D) The similarity of two trees is *defined* as the Manhattan or lp_1
 *    distance between the two dictionaries (or vectors).
 *
 * Commentary:
 *
 * The pdf above provides a proof that this similarity measure is a
 * lower bound for a certain kind of tree-edit distance. However, it
 * is not at all clear that "tree-edit distance" is the correct way
 * to compare combo trees.  There are multiple reasons for this:
 *
 * a) The combo trees +(a b) and +(b a) are functionally equivalent,
 *    but are separated by a non-zero tree-edit distance.  They are
 *    also judged different by the code below.  Thus, the code below
 *    will work best if both trees have been passed through reduct,
 *    thus normalizing them for such differences.
 *
 * b) The combo trees +($a $b) and +($a $c) seem obviously different,
 *    but what if $b and $c are features that are highly correlated?
 *    Thus, the "true" distance probably has to be adjusted by such
 *    correlations.
 *
 * c) The tree-edit distance (and the distnace used here) between the
 *    combo trees "fold(f [] a b c d e)" and  "fold(g [] a b c d e)
 *    would seem to be small: differing only by the applied function
 *    f vs g.  But these can be very wildly different functions, so
 *    what is the "true" diffference between these trees?
 *
 * Thus, the similarity defined here seems to be no better and no worse
 * than comparing combo trees via some tree-edit distance.  its an
 * imperfect measure but, lacking a clearer defintiion of the "genotype"
 * of a combo tree, there does not seem to be any way of much improving
 * in this measure.
 *
 * Note: There are various changes and departures from the algo
 * described in the PDF:
 *
 * 1) We never need the binary tree version of a tree, and so never
 *    generate that.  Instead, we generate the branch-tree vector
 *    directly.
 * 2) Empty nodes are never generated; they're not needed.
 * 2) Leaf nodes are not generated. This is a fairly significant
 *    departure from the original algorithm.  It does alter the
 *    distance measure for tree similarity; our trees are measured
 *    'closer' than the original algo, by about a factor of 2.
 *    I beleive that this difference won't affect results by much;
 *    it will save cpu time, though.
 * 4) We don't actually need a C++ vector to hold the vector. An
 *    std::map (a dictionary) is easier.
 */

/**
 * tree_flatten -- approximate combo tree by a flattened vector
 *
 * Implicitly convert the tree to binary form, and then count the
 * number of vertexes of each type in that binary tree.
 *
 * (The binary tree equivalent of a combo tree is a tree where every
 * vertex has at most two children, but is otherwise semantically
 * equivalent. See notes above for details.).
 */
static void tree_flatten_rec(tree_branch_vector& ctr, combo_tree::iterator root)
{
	size_t numch = root.number_of_children();
	if (0 == numch)
	{
		std::stringstream ss;
		ss << *root;
		ctr[ss.str()] += 1;
		return;
	}
	if (1 == numch)
	{
		combo_tree::sibling_iterator first = root.begin();
		std::stringstream ss;
		ss << *root << "(" << *first << ")";
		ctr[ss.str()] += 1;

		if (0 != first.number_of_children())
			tree_flatten_rec(ctr, first);

		return;
	}

	// For numch>2, we just distribute associarively over its arguments.
	// For example, if the root f, then the tree "f(a b c)" is converted
	// to f(a f(b c)), and so the first tree-branch is f(a f).  Note that
	// f *does not* have to actually be an associative operator (like +
	// or * or boolean_and or boolean_or).  The reason that f does not
	// have to be associative is that we never evaluate the binary tree;
	// rather, this is merely an accounting trick to generate a vector
	// that approximately captures the general structure of the tree.
	combo_tree::sibling_iterator first = root.begin();
	size_t i = 0;
	for (; i < numch-2; first++, i++)
	{
		std::stringstream ss;
		ss << *root << "(" << *first << " " << *root << ")";
		ctr[ss.str()] += 1;

		if (0 != first.number_of_children())
			tree_flatten_rec(ctr, first);
	}

	combo_tree::sibling_iterator second = first;
	second ++;

	std::stringstream ss;
	ss << *root << "(" << *first << " " << *second << ")";
	ctr[ss.str()] += 1;

	if (0 != first.number_of_children())
		tree_flatten_rec(ctr, first);

	if (0 != second.number_of_children())
		tree_flatten_rec(ctr, second);
}

/** Same as above, wrapper */
tree_branch_vector tree_flatten(const combo_tree& tree)
{
	tree_branch_vector counter;
	combo_tree::iterator root = tree.begin();
	tree_flatten_rec(counter, root);
	return counter;
}

/** Same as above, utility version for combo trees as strings */
tree_branch_vector tree_flatten(const std::string& str)
{
	std::stringstream ss;
	combo_tree tr;
	ss << str;
	ss >> tr;
	return tree_flatten(tr);
}

/**
 * tree_similarity -- compute the manhattan distance between trees.
 *
 * The manhattan distance is the same as the lp_1 distance.
 */
size_t tree_similarity(const tree_branch_vector& av,
                       const tree_branch_vector& bv)
{
	size_t dist = 0;
	// At the end, diff will hold anything in bv that is not in av
	tree_branch_vector diff = bv;
	foreach (auto pr, av)
	{
		size_t acnt = pr.second;
		size_t bcnt = 0;
		diff.erase(pr.first);
		try {
			bcnt = bv.at(pr.first);
		}
		catch (const std::out_of_range& oor)
		{}

		dist += abs(acnt - bcnt);
	}

	// Add anything in bv that was not in av.
	foreach (auto p, diff)
	{
		dist += p.second;
	}
	return dist;
}

size_t tree_similarity(const combo_tree& a, const combo_tree& b)
{
	tree_branch_vector av = tree_flatten(a);
	tree_branch_vector bv = tree_flatten(b);
	return tree_similarity(av, bv);
}

size_t tree_similarity(const std::string& a, const std::string& b)
{
	tree_branch_vector av = tree_flatten(a);
	tree_branch_vector bv = tree_flatten(b);
	return tree_similarity(av, bv);
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


} // ~namespace combo
} // ~namespace opencog
