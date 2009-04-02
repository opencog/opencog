/**
 * Foreach.h
 *
 * Basic iterator constructs.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_ATOMSPACE_FOREACH_H
#define _OPENCOG_ATOMSPACE_FOREACH_H

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/TLB.h>

namespace opencog
{

/**
 * Invoke the callback on each atom in the outgoing set of handle h.
 */
template<class T>
inline bool foreach_outgoing_handle(Handle h, bool (T::*cb)(Handle), T *data)
{
    Atom *atom = TLB::getAtom(h);
    Link *link = dynamic_cast<Link *>(atom);
    if (NULL == link) return false;

    const std::vector<Handle> &vh = link->getOutgoingSet();

    for (size_t i = 0; i < vh.size(); i++) {
        Handle hout = vh[i];
        bool rc = (data->*cb)(hout);
        if (rc) return rc;
    }
    return false;
}

/**
 * Invoke the callback on each atom in the outgoing set of handle h.
 */
template<class T>
inline bool foreach_outgoing_atom(Handle h, bool (T::*cb)(Atom *), T *data)
{
    Atom *atom = TLB::getAtom(h);
    Link *link = dynamic_cast<Link *>(atom);
    if (NULL == link) return false;

    const std::vector<Handle> &vh = link->getOutgoingSet();

    for (size_t i = 0; i < vh.size(); i++) {
        Handle hout = vh[i];
        Atom *aout = TLB::getAtom(hout);
        bool rc = (data->*cb)(aout);
        if (rc) return rc;
    }
    return false;
}

/* ----------------------------------------------------------- */

/**
 * Invoke the callback on each atom in the incoming set of handle h.
 */

template<class T>
inline bool foreach_incoming_atom(Handle h, bool (T::*cb)(Atom *), T *data)
{
    Atom *atom = TLB::getAtom(h);
    HandleEntry *he = atom->getIncomingSet();

    // Simple linked-list pointer chase
    while (he) {
        Atom *aout = TLB::getAtom(he->handle);
        bool rc = (data->*cb)(aout);
        if (rc) return rc;
        he = he->next;
    }
    return false;
}

template<class T>
inline bool foreach_incoming_handle(Handle h, bool (T::*cb)(Handle), T *data)
{
    Atom *atom = TLB::getAtom(h);
    HandleEntry *he = atom->getIncomingSet();

    // Simple linked-list pointer chase
    while (he) {
        bool rc = (data->*cb)(he->handle);
        if (rc) return rc;
        he = he->next;
    }
    return false;
}

} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_FOREACH_H
