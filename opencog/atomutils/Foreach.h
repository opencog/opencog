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


//! Invoke the callback on each atom in the outgoing set of handle h
//! till one of them return true, and returns true as a
//! result. Otherwise the callback is called on all outgoings and
//! false is returned.
/// XXX TODO this should be moved to class Link
template<class T>
inline bool foreach_outgoing_handle(LinkPtr ll, bool (T::*cb)(const Handle&), T *data)
{
    for (Handle out_h : ll->getOutgoingSet()) {
        bool rc = (data->*cb)(out_h);
        if (rc) return rc;
    }
    return false;
}

/* ----------------------------------------------------------- */

//! Invoke the callback on each atom in the incoming set of handle h
//! till one of them returns true, and returns true as a
//! result. Otherwise the callback is called on all incomings and
//! false is returned.
/// XXX TODO this routine should be moved to class Atom
template<class T>
inline bool foreach_incoming_handle(Handle h, bool (T::*cb)(const Handle&), T *data)
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
