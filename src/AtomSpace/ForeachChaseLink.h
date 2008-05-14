/**
 * ForeachChaseLink.h
 *
 * This template class implements a foreach iterator that iterates over
 * all atoms associated with a given atom via a certain link type.
 * It does this by iterating over all links containing the given atom,
 * and then invoking a callback for a corresponding atom in that link.
 * Two utility methods are provided: follow a binary link in the foreard
 * direction, and in the reverse direction.  
 *
 * If there is only one link of the given type, for the given atom, then
 * the FollowLink clas provides an easier-to-use interface.
 *
 * Example usage:
 *
 * class MyClass
 * {
 *    bool my_callback(Handle h)
 *    {
 *       printf("Hello world, found %ul\n", (unsigned long) h);
 *    }
 *
 *    void do_stuff_with_handle(Handle h)
 *    {
 *       ForeachChaseLink<MyClass> my_iter;
 *
 *       my_iter.follow_binary_link(h, INHERITANCE_LINK, 
 *                                  MyClass::my_callback, this);
 *    }
 * };
 *
 * The above example invokes the callback "my_callback" on every handle
 * that is at the far end of an inheritence link containing the input 
 * handle h.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_LINK_CHASE_H_
#define OPENCOG_LINK_CHASE_H_

#include "Atom.h"
#include "Link.h"
#include "Foreach.h"
#include "TLB.h"

namespace opencog {

template <class T>
class ForeachChaseLink
{
	public:

		/**
		 * follow_binary_link -- follow an ordered, binary link.
		 *
		 * Look at the incoming set of the specified atom.
		 * Find all links of type link_type,
		 * then follow this link to see where its going.
		 * Call the callback for each endpoint found.
		 *
		 * The callback should return false to search for 
		 * more matches, or return true to halt the search.
		 */
		inline bool follow_binary_link(Handle h, Type ltype, bool (T::*cb)(Handle), T *data)
		{
			return follow_link(h, ltype, 0, 1, cb, data);
		}

		/**
		 * Same as above, except that the link is followed in the
		 * reverse direction.
		 */
		inline bool backtrack_binary_link(Handle h, Type ltype, bool (T::*cb)(Handle), T *data)
		{
			return follow_link(h, ltype, 1, 0, cb, data);
		}

		/**
		 * follow_link -- follow an ordered, binary link.
		 *
		 * Look at the incoming set of the specified atom.
		 * Find all links of type link_type.
		 * Check to make sure that the input handle "h"
		 * occupies position "from" in the link. 
		 * If it does, then invoke the callback,
		 * passing the handle in position "to" to the callback.
		 * The callback is called for each endpoint found.
		 *
		 * The callback should return false to search for 
		 * more matches, or return true to halt the search.
		 */
		inline bool follow_link(Handle h, Type ltype, int from, int to, bool (T::*cb)(Handle), T *data)
		{
			Atom *atom = TLB::getAtom(h);

			if (NULL == atom) return NULL;

			// Look for incoming links that are of the given type.
			// Then grab the thing that they link to.
			link_type = ltype;
			from_atom = atom;
			to_atom = NULL;
			position_from = from;
			position_to = to;
			user_callback = cb;
			user_data = data;
			bool rc = foreach_incoming_atom(h, &ForeachChaseLink::find_link_type, this);
			return rc;
		}

	private:
		Type link_type;
		Atom * from_atom;
		Atom * to_atom;
		int position_from;
		int position_to;
		int cnt;
		bool (T::*user_callback)(Handle);
		T *user_data;

		/**
		 * Check for link of the desired type, then loop over its outgoing set.
		 */
		inline bool find_link_type(Atom *atom)
		{
			// Make sure the link is of the specified link type
			if (link_type != atom->getType()) return false;

			cnt = -1;
         to_atom = NULL;
			Handle h = TLB::getHandle(atom);
			foreach_outgoing_atom(h, &ForeachChaseLink::pursue_link, this);

			bool rc = false;
			if (to_atom)
			{
				rc = (user_data->*user_callback)(TLB::getHandle(to_atom));
			}
			return rc;
		}

		inline bool pursue_link(Atom *atom)
		{
			cnt ++;

			// The from-slot should be occupied by the node itself.
			if (position_from == cnt)
			{
				if (from_atom != atom)
				{
					to_atom = NULL;
					return true; // bad match, stop now.
				}
				return false;
			}

			// The to-slot is the one we're looking for.
			if (position_to == cnt)
			{
				to_atom = atom;
			}

			return false;
		}
};
}

#endif /* OPENCOG_LINK_CHASE_H_ */
