/**
 * OutgoingTree.h
 *
 * Utilities for navigating a tree of outgoing edges.
 *
 * Linas Vepstas February 2008
 */

#include "types.h"

namespace opencog {

class OutgoingTree
{
	public:
		/**
		 * Return true if the indicated node occurs somwhere in the 
		 * tree spanned by the outgoing set.
		 */
		inline bool is_node_in_tree(Handle tree, Handle node)
		{
			tgt = node;
			return in_tree(tree);
		}

	private:
		Handle tgt;
		inline bool in_tree(Handle tree)
		{
			if (tree == NULL) return false;
			if (tree == tgt) return true;
			foreach_outgoing_handle(tree, &OutgoingTree::in_tree, this);
		}
};

}

