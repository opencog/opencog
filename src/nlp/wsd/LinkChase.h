/**
 * LinkChase.h
 *
 * Follow a binary link.
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
class LinkChase
{
	public:

		/**
		 * follow_binary_link -- follow an ordered, binary link.
		 *
		 * Look at the incoming set of this atom.
		 * Find the first link of type link_type,
		 * then follow this link to see where its going.
		 * Its asssumed that there is only one such link,
		 * and that its binary.
		 * Return a pointer to where the link is going.
		 */
#if 0
		inline Atom * follow_binary_link(Atom *atom, Type ltype)
		{
			return follow_link(atom, ltype, 0, 1);
		}

		inline Atom * backtrack_binary_link(Atom *atom, Type ltype)
		{
			return follow_link(atom, ltype, 1, 0);
		}
#endif

		inline bool follow_link(Handle h, Type ltype, int from, int to, bool (T::*cb)(Handle), T *data)
		{
			Atom *atom = TLB::getAtom(h);

			if (NULL == atom) return NULL;

			// Look for incoming links that are of the given type.
			// Then grab the thing that they link to.
			link_type = ltype;
			from_atom = atom;
			position_from = from;
			position_to = to;
			user_callback = cb;
			user_data = data;
			bool rc = foreach_incoming_atom(h, &LinkChase::find_link_type, this);
			return rc;
		}

	private:
		Type link_type;
		Atom * from_atom;
		int position_from;
		int position_to;
		int cnt;
		bool (T::*user_callback)(Handle);
		T *user_data;

		/**
		 * Find the (first!, assumed only!?) link of desired type.
		 */
		inline bool find_link_type(Atom *atom)
		{
			// Look for incoming links that are of the specified link type
			if (link_type != atom->getType()) return false;

			cnt = -1;
			Handle h = TLB::getHandle(atom);
			bool rc = foreach_outgoing_atom(h, &LinkChase::pursue_link, this);
			if (rc) return rc;
			return false;
		}

		inline bool pursue_link(Atom *atom)
		{
			cnt ++;

			// The from-slot should be occupied by the node itself.
			if (position_from == cnt)
			{
				if (from_atom != atom) return true;
				return false;
			}

			// The to-slot is the one we're looking for.
			if (position_to == cnt)
			{
				bool rc = (user_data->*user_callback)(TLB::getHandle(atom));
				if(rc) return rc;  // We're done now.
			}

			return false;
		}
};
}

#endif /* OPENCOG_LINK_CHASE_H_ */
