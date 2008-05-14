/**
 * FollowLink.h
 *
 * Follow a binary link.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_FOLLOW_LINK_H_
#define OPENCOG_FOLLOW_LINK_H_

#include "Atom.h"
#include "Link.h"
#include "Foreach.h"
#include "TLB.h"

namespace opencog {

class FollowLink
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
		inline Atom * follow_binary_link(Atom *atom, Type ltype)
		{
			return follow_link(atom, ltype, 0, 1);
		}

		inline Atom * backtrack_binary_link(Atom *atom, Type ltype)
		{
			return follow_link(atom, ltype, 1, 0);
		}

		inline Atom * follow_link(Atom *atom, Type ltype, int from, int to)
		{
			if (NULL == atom) return NULL;

			// Look for incoming links that are of the given type.
			// Then grab the thing that they link to.
			link_type = ltype;
			from_atom = atom;
			to_atom = NULL;
			position_from = from;
			position_to = to;
			Handle h = TLB::getHandle(atom);
			foreach_incoming_atom(h, &FollowLink::find_link_type, this);
			return to_atom;
		}

	private:
		Type link_type;
		Atom * from_atom;
		Atom * to_atom;
		int position_from;
		int position_to;
		int cnt;

		/**
		 * Find the (first!, assumed only!?) link of desired type.
		 */
		inline bool find_link_type(Atom *atom)
		{
			// Look for incoming links that are of the specified link type
			if (link_type != atom->getType()) return false;

			cnt = -1;
			to_atom = NULL;
			Handle h = TLB::getHandle(atom);
			foreach_outgoing_atom(h, &FollowLink::pursue_link, this);
			if (to_atom) return true;
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
				to_atom = atom;
				return true;  // We're done now.
			}

			return false;
		}
};
}

#endif /* OPENCOG_FOLLOW_LINK_H_ */
