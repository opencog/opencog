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
		 * Follow a binary link. Make sure that "atom" occurs
		 * first in the link (if not, return null). Return the 
		 * atom that is second in the link.
		 */
		inline Atom * follow_binary_link(Link *link, Atom *atom)
		{
			from_atom = atom;
			to_atom = NULL;
			cnt = 0;
			Handle h = TLB::getHandle(link);
			foreach_outgoing_atom(h, &FollowLink::check_link, this);
			return to_atom;
		};

	private:
		Type link_type;
		Atom * from_atom;
		Atom * to_atom;
		int cnt;
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
