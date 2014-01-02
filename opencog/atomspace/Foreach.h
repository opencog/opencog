/**
 * Foreach.h
 *
 * Basic iterator constructs.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_ATOMSPACE_FOREACH_H
#define _OPENCOG_ATOMSPACE_FOREACH_H

#include <opencog/atomspace/Link.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */


//! Invoke the callback on each atom in the outgoing set of handle h.
/// XXX TODO this should be moved to class Link
template<class T>
inline bool foreach_outgoing_handle(LinkPtr ll, bool (T::*cb)(Handle), T *data)
{
    const std::vector<Handle> &vh = ll->getOutgoingSet();
    size_t sz = vh.size();

    for (size_t i = 0; i < sz; i++) {
        bool rc = (data->*cb)(vh[i]);
        if (rc) return rc;
    }
    return false;
}

/* ----------------------------------------------------------- */

//! Invoke the callback on each atom in the incoming set of handle h.
/// XXX TODO this routine should be moved to class Atom
template<class T>
inline bool foreach_incoming_handle(Handle h, bool (T::*cb)(Handle), T *data)
{
    IncomingSet vh = h->getIncomingSet();
    size_t sz = vh.size();

    for (size_t i = 0; i < sz; i++) {
        Handle h(vh[i]);
        bool rc = (data->*cb)(h);
        if (rc) return rc;
    }
    return false;
}

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_FOREACH_H
