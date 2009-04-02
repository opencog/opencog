/**
 * OutgoingTree.h
 *
 * Utilities for navigating a tree of outgoing edges.
 *
 * Linas Vepstas February 2008
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
		inline bool is_node_in_tree(Handle tree, Handle node)
		{
			tgt = node;
			return in_tree(tree);
		}

	private:
		Handle tgt;
		inline bool in_tree(Handle tree)
		{
			if (TLB::isInvalidHandle(tree)) return false;
			if (tree == tgt) return true;
			return opencog::foreach_outgoing_handle(tree, &OutgoingTree::in_tree, this);
		}
};

} // namespace opencog

#endif // _OPENCOG_OUTGOING_TREE_H
