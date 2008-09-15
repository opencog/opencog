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

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Foreach.h>
#include <opencog/atomspace/TLB.h>

namespace opencog
{

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
    inline Atom * follow_binary_link(Atom *atom, Type ltype) {
        return follow_link(atom, ltype, 0, 1);
    }

    inline Atom * backtrack_binary_link(Atom *atom, Type ltype) {
        return follow_link(atom, ltype, 1, 0);
    }

    inline Atom * follow_link(Atom *atom, Type ltype, int from, int to) {
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
    inline bool find_link_type(Atom *atom) {
        // Make sure that the link is of the desired type.
        if (link_type != atom->getType()) return false;

        cnt = -1;
        to_atom = NULL;
        Handle h = TLB::getHandle(atom);
        foreach_outgoing_atom(h, &FollowLink::pursue_link, this);
        if (to_atom) return true;
        return false;
    }

    inline bool pursue_link(Atom *atom) {
        cnt ++;

        // The from-slot should be occupied by the node itself.
        if (position_from == cnt) {
            if (from_atom != atom) {
                to_atom = NULL;
                return true; // Bad match, stop now.
            }
            return false;
        }

        // The to-slot is the one we're looking for.
        if (position_to == cnt) {
            to_atom = atom;
        }

        return false;
    }
};

} // namespace opencog

#endif // _OPENCOG_FOLLOW_LINK_H
