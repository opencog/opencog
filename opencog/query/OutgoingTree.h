/**
 * OutgoingTree.h
 *
 * Utilities for navigating a tree of outgoing edges.
 *
 * Copyright (C) 2009 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
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
 *
 * Created by Linas Vepstas February 2008
 */

#ifndef _OPENCOG_OUTGOING_TREE_H
#define _OPENCOG_OUTGOING_TREE_H

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/Foreach.h>

namespace opencog {

class OutgoingTree
{
	public:
		/**
		 * Return true if the indicated node occurs somewhere in the 
		 * tree spanned by the outgoing set.
		 */
		inline bool is_node_in_tree(Handle& tree, Handle& node)
		{
			tgt = node;
			return in_tree(tree);
		}

	private:
		Handle tgt;
		inline bool in_tree(Handle tree)
		{
			if (tree == Handle::UNDEFINED) return false;
			if (tree == tgt) return true;
			LinkPtr ltree(LinkCast(tree));
			if (NULL == ltree) return false;
			return opencog::foreach_outgoing_handle(ltree, &OutgoingTree::in_tree, this);
		}
};

} // namespace opencog

#endif // _OPENCOG_OUTGOING_TREE_H
