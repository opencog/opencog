/**
 * ForeachChaseLink.h
 *
 * This file implements a number of link-chasing routines.
 * By "link-chasing", it is meant: given an input atom, find
 * all links of some given link-type that contain it, and then,
 * call a callback on each of the other elements in that link.
 *
 * Thus, for example, given the hypergraph
 *
 *   SomeLink
 *	   FirstAtom
 *	   SecondAtom
 *
 * then, given a handle to "FirstAtom", and the link type "SomeLink",
 * the "foreach_binary_link" will call the callback on "SecondAtom".
 *
 * The link-chasing is order dependent; if, instead, one has
 * "SecondAtom" and one wants "FirstAtom", then use the
 * "foreach_reverse_binary_link" routine.
 *
 * The "foreach_unordered_binary_link" is not order-dependent; given
 * either atom in the link, it will return the other.
 *
 * The "foreach_link" routine provides the analogous service for N-ary
 * links.
 *
 * To summarize: 
 * foreach_binary_link -- follow an ordered, binary link.
 * foreach_reverse_binary_link - follow binary link in reverse.
 * foreach_link -- follow an ordered N-ary link.
 * foreach_unordered_binary_link - chase link from one to other atom.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_LINK_CHASE_H
#define _OPENCOG_LINK_CHASE_H

#include <opencog/atoms/base/Link.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * This class is not meant for external use, it is meant to be a
 * private utility for the use of the helper functions below.
 * Unfortunately, C++ does not give any way of hiding this from the
 * namespace. Too bad :-( 
 *
 * However, following this "private" definition are several public
 * callback iterators, which *are* intended for general use.
 *
 * This template class implements a foreach iterator that iterates over
 * all atoms associated with a given atom via a certain link type.
 * It does this by iterating over all links containing the given atom,
 * and then invoking a callback for a corresponding atom in that link.
 * Two utility methods are provided: follow a binary link in the foreward
 * direction, and in the reverse direction.
 *
 * If there is only one link of the given type, for the given atom, then
 * the FollowLink clas provides an easier-to-use interface.
 *
 * Example usage:
 *
 * class MyClass
 * {
 *	bool my_callback(const Handle& h)
 *	{
 *	   printf("Hello world, found %ul\n", (unsigned long) h);
 *	}
 *
 *	void do_stuff_with_handle(Handle h)
 *	{
 *	   PrivateUseOnlyChaseLink<MyClass> my_iter;
 *
 *	   my_iter.follow_binary_link(h, INHERITANCE_LINK,
 *								  MyClass::my_callback, this);
 *	}
 * };
 *
 * The above example invokes the callback "my_callback" on every handle
 * that is at the far end of an inheritence link containing the input
 * handle h.
 */

template <typename T>
class PrivateUseOnlyChaseLink
{
public:

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
	inline bool follow_link(const Handle& h, Type ltype,
	              int from, int to, bool (T::*cb)(const Handle&), T *data)
	{
		user_callback = cb;
		user_callback_lh = NULL;
		return do_follow_link (h, ltype, from, to, data);
	}

	/**
	 * Same as above, except the callback is passed the handle of
	 * the link itself in the second arg.
	 */
	inline bool follow_link_lh(const Handle& h, Type ltype, int from, int to,
							   bool (T::*cb)(const Handle&, const Handle&), T *data)
	{
		user_callback = NULL;
		user_callback_lh = cb;
		return do_follow_link (h, ltype, from, to, data);
	}

	inline bool follow_unordered_binary_link(const Handle& h, Type ltype,
	                                         bool (T::*cb)(const Handle&), T *data)
	{
		user_callback = cb;
		user_callback_lh = NULL;
		return do_follow_unordered_binary_link(h, ltype, data);
	}

	inline bool follow_unordered_binary_link(const Handle& h, Type ltype,
	                        bool (T::*cb)(const Handle&, const Handle&), T *data)
	{
		user_callback = NULL;
		user_callback_lh = cb;
		return do_follow_unordered_binary_link(h, ltype, data);
	}

private:
	Type link_type;
	Handle from_atom;
	Handle to_atom;
	int position_from;
	int position_to;
	int cnt;
	bool (PrivateUseOnlyChaseLink::*endpoint_matcher)(const Handle&);
	bool (T::*user_callback)(const Handle&);
	bool (T::*user_callback_lh)(const Handle&, const Handle&);
	T *user_data;

	inline bool do_follow_link(const Handle& h, Type ltype, int from, int to, T *data)
	{
		// Look for incoming links that are of the given type.
		// Then grab the thing that they link to.
		link_type = ltype;
		from_atom = h;
		to_atom = Handle::UNDEFINED;
		position_from = from;
		position_to = to;
		user_data = data;
		endpoint_matcher = &PrivateUseOnlyChaseLink::pursue_link;
		bool rc = h->foreach_incoming(&PrivateUseOnlyChaseLink::find_link_type, this);
		return rc;
	}

	inline bool do_follow_unordered_binary_link(const Handle& h, Type ltype, T *data)
	{
		// Look for incoming links that are of the given type.
		// Then grab the thing that they link to.
		link_type = ltype;
		from_atom = h;
		to_atom = Handle::UNDEFINED;
		user_data = data;
		endpoint_matcher = &PrivateUseOnlyChaseLink::pursue_unordered_link;
		bool rc = h->foreach_incoming(&PrivateUseOnlyChaseLink::find_link_type, this);
		return rc;
	}

	/**
	 * Check for link of the desired type, then loop over its outgoing set.
	 */
	inline bool find_link_type(const Handle& link_h)
	{
		// Make sure the link is of the specified link type
		if (link_type != link_h->get_type()) return false;

		cnt = -1;
		to_atom = Handle::UNDEFINED;
		// foreach_outgoing_handle(link_h, PrivateUseOnlyChaseLink::endpoint_matcher, this);

		LinkCast(link_h)->foreach_outgoing(endpoint_matcher, this);

		bool rc = false;
		if (to_atom)
		{
			if (user_callback)
				rc = (user_data->*user_callback)(to_atom);
			else
				rc = (user_data->*user_callback_lh)(to_atom, link_h);
		}
		return rc;
	}

	inline bool pursue_link(const Handle& h)
	{
		cnt ++;

		// The from-slot should be occupied by the node itself.
		if (position_from == cnt)
		{
			if (from_atom != h)
			{
				to_atom = Handle::UNDEFINED;
				return true; // bad match, stop now.
			}
			return false;
		}

		// The to-slot is the one we're looking for.
		if (position_to == cnt)
		{
			to_atom = h;
		}

		return false;
	}

	inline bool pursue_unordered_link(const Handle& h)
	{
		// There are only two atoms in a binary link. The one that is
		// not the from_atom is the one we are looking for.
		if (from_atom != h)
		{
			to_atom = h; // found it!
			return true;
		}
		return false;
	}
};

/**
 * foreach_binary_link -- follow an ordered, binary link.
 *
 * Look at the incoming set of the specified atom.
 * Find all links of type link_type,
 * then follow this link to see where its going.
 * Call the callback for each endpoint found.
 *
 * The callback should return false to search for
 * more matches, or return true to halt the search.
 */
template <typename T>
inline bool foreach_binary_link(const Handle& h, Type ltype,
                                bool (T::*cb)(const Handle&), T *data)
{
	PrivateUseOnlyChaseLink<T> cl;
	return cl.follow_link(h, ltype, 0, 1, cb, data);
}

/**
 * Same as above, except that callback has second argument.
 * The handle of the link itself is passed in the second argument.
 */
template <typename T>
inline bool foreach_binary_link(const Handle& h, Type ltype,
              bool (T::*cb)(const Handle&, const Handle&), T *data)
{
	PrivateUseOnlyChaseLink<T> cl;
	return cl.follow_link_lh(h, ltype, 0, 1, cb, data);
}

/**
 * foreach_reverse_binary_link - follow binary link in reverse.
 *
 * Same as above, except that the link is followed in the
 * reverse direction.
 */
template <typename T>
inline bool foreach_reverse_binary_link(const Handle& h, Type ltype,
                             bool (T::*cb)(const Handle&), T *data)
{
	PrivateUseOnlyChaseLink<T> cl;
	return cl.follow_link(h, ltype, 1, 0, cb, data);
}

/**
 * Same as above, except that callback has second argument.
 * The handle of the link itself is passed in the second argument.
 */
template <typename T>
inline bool foreach_reverse_binary_link(const Handle& h, Type ltype,
                  bool (T::*cb)(const Handle&, const Handle&), T *data)
{
	PrivateUseOnlyChaseLink<T> cl;
	return cl.follow_link_lh(h, ltype, 1, 0, cb, data);
}

/**
 * foreach_link -- follow an ordered N-ary link.
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
template <typename T>
inline bool foreach_link(const Handle& h, Type ltype, int from, int to,
                         bool (T::*cb)(const Handle&), T *data)
{
	PrivateUseOnlyChaseLink<T> cl;
	return cl.follow_link(h, ltype, from, to, cb, data);
}

/**
 * Same as above, except the callback is passed the handle of
 * the link itself in the second arg.
 */
template <typename T>
inline bool foreach_link(const Handle& h, Type ltype, int from, int to,
						 bool (T::*cb)(const Handle&, const Handle&), T *data)
{
	PrivateUseOnlyChaseLink<T> cl;
	return cl.follow_link_lh(h, ltype, from, to, cb, data);
}

/**
 * foreach_unordered_binary_link - chase link from one to other atom.
 */
template <typename T>
inline bool foreach_unordered_binary_link(const Handle& h, Type ltype,
                                     bool (T::*cb)(const Handle&), T *data)
{
	PrivateUseOnlyChaseLink<T> cl;
	return cl.follow_unordered_binary_link(h, ltype, cb, data);
}

template <typename T>
inline bool foreach_unordered_binary_link(const Handle& h, Type ltype,
                      bool (T::*cb)(const Handle&, const Handle&), T *data)
{
	PrivateUseOnlyChaseLink<T> cl;
	return cl.follow_unordered_binary_link(h, ltype, cb, data);
}

/** @}*/
} // namespace opencog

#endif // _OPENCOG_LINK_CHASE_H
