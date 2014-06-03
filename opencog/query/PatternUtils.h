/**
 * PatternUtils.h
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

#ifndef _OPENCOG_PATTERN_UTILS_H
#define _OPENCOG_PATTERN_UTILS_H

#include <opencog/util/foreach.h>
#include <opencog/atomspace/types.h>
#include <opencog/atomspace/Foreach.h>

namespace opencog {

///  Find all of the variable nodes occuring in a clause.
class FindVariables
{
   public:
      std::vector<Handle> varlist;

      /**
       * Create a list of all of the VariableNodes that lie in the
       * outgoing set of the handle (recursively).
       */
      inline bool find_vars(Handle h)
      {
         Type t = h->getType();
         if (classserver().isNode(t))
         {
            if (t == VARIABLE_NODE)
            {
               varlist.push_back(h);
            }
            return false;
         }

         LinkPtr l(LinkCast(h));
         return foreach_outgoing_handle(l, &FindVariables::find_vars, this);
      }

      inline void find_vars(std::vector<Handle> hlist)
      {
			foreach(Handle h, hlist) find_vars(h);
		}
};


class OutgoingTree
{
	public:
		/**
		 * Return true if any of the indicated nodes occurs somewhere in 
		 * the tree spanned by the outgoing set.
		 */
		inline bool any_node_in_tree(Handle& tree, std::vector<Handle>& nodes)
		{
			foreach(Handle n, nodes)
			{
				if (is_node_in_tree(tree, n)) return true;
			}
			return false;
		}

		/**
		 * Return true if any of the indicated node occurs somewhere in the 
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

#endif // _OPENCOG_PATTERN_UTILS_H
