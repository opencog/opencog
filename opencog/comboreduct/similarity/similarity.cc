/*
 * opencog/comboreduct/similarity/similarity.cc
 *
 * Copyright (C) 2014 Aidyia
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
#include "similarity.h"

namespace opencog { namespace combo {

/**
 * to_binary_tree -- convert combo tree to binary tree
 *
 * Create a a tree that is functionally the same as the given tree,
 * but such that it is a binary tree: every vertex has at most two
 * children.
 */
combo_tree to_binary_tree(const combo_tree& tree)
{
	combo_tree btree(tree);

	return btree;
}


tree_similarity::tree_similarity(void)
{

}

} // ~namespace combo
} // ~namespace opencog
