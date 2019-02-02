/**
 * FollowLink.h
 *
 * Follow a link. Given an atom and a link type, follow the link and
 * and return another atom in the link. It is assumed that there is only
 * one link of the given type that is attached to the given atom; the
 * methods here return the first match found. If multiple-link support
 * is needed, use the ForeachChaseLink<T> class.
 *
 * This method includes two utility routines, meant to follow binary
 * links (i.e. links with only two members).
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_FOLLOW_LINK_H
#define _OPENCOG_FOLLOW_LINK_H

#include <opencog/atoms/base/Link.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

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
	inline Handle follow_binary_link(const Handle& h, Type ltype)
	{
		return follow_link(h, ltype, 0, 1);
	}

	inline Handle backtrack_binary_link(const Handle& h, Type ltype)
	{
		return follow_link(h, ltype, 1, 0);
	}

	inline Handle follow_link(const Handle& h, Type ltype, int from, int to)
	{
		// Look for incoming links that are of the given type.
		// Then grab the thing that they link to.
		link_type = ltype;
		from_atom = h;
		to_atom = Handle::UNDEFINED;
		position_from = from;
		position_to = to;
		h->foreach_incoming(&FollowLink::find_link_type, this);
		return to_atom;
	}

private:
	Type link_type;
	Handle from_atom;
	Handle to_atom;
	int position_from;
	int position_to;
	int cnt;

	/**
	 * Find the (first!, assumed only!?) link of desired type.
	 */
	inline bool find_link_type(const Handle& h)
	{
		// Make sure that the link is of the desired type.
		if (link_type != h->get_type()) return false;

		cnt = -1;
		to_atom = Handle::UNDEFINED;
		if (h->is_link()) LinkCast(h)->foreach_outgoing(&FollowLink::pursue_link, this);
		if (to_atom) return true;
		return false;
	}

	inline bool pursue_link(const Handle& h)
	{
		cnt ++;

		// The from-slot should be occupied by the node itself.
		if (position_from == cnt) {
			if (from_atom != h) {
				to_atom = Handle::UNDEFINED;
				return true; // Bad match, stop now.
			}
			return false;
		}

		// The to-slot is the one we're looking for.
		if (position_to == cnt) {
			to_atom = h;
		}

		return false;
	}
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_FOLLOW_LINK_H
