/**
 * FollowLink.h
 *
 * Follow a binary link.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

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
			// Look for incoming links that are InheritanceLinks.
			// The "generalized concept" for this should be at the far end.
			link_type = ltype;
			from_atom = atom;
			Handle h = TLB::getHandle(atom);
			foreach_incoming_atom(h, &FollowLink::find_link_type, this);
			return to_atom;
		}

	private:
		Type link_type;
		Atom * from_atom;
		Atom * to_atom;
		int cnt;

		/**
		 * Find the (first!, assumed only!?) link of desired type.
		 */
		inline bool find_link_type(Atom *atom)
		{
			// Look for incoming links that are of the specified link type
			if (link_type != atom->getType()) return false;

			cnt = 0;
			to_atom = NULL;
			Handle h = TLB::getHandle(atom);
			foreach_outgoing_atom(h, &FollowLink::check_link, this);
			if (to_atom) return true;
			return false;
		}

		inline bool check_link(Atom *atom)
		{
			cnt ++;
			// The first node should be the node itself.
			if (1 == cnt)
			{
				if (from_atom != atom) return true;
				return false;
			}

			// The second node is the one we're looking for.
			to_atom = atom;

			// If there's a third node, then its an error,
			// but we are not error checking this.
			return true;
		}
};
}


/* =================== END OF FILE ====================== */
