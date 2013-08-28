/**
 * Foreach.h
 *
 * Basic iterator constructs.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_ATOMSPACE_FOREACH_H
#define _OPENCOG_ATOMSPACE_FOREACH_H

#include <opencog/server/CogServer.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */


//! Invoke the callback on each atom in the outgoing set of handle h.
template<class T>
inline bool foreach_outgoing_handle(Handle h, bool (T::*cb)(Handle), T *data)
{
    const AtomSpace &as = atomspace();
    const std::vector<Handle> &vh = as.getOutgoing(h);
    size_t sz = vh.size();

    for (size_t i = 0; i < sz; i++) {
        Handle hout = vh[i];
        bool rc = (data->*cb)(hout);
        if (rc) return rc;
    }
    return false;
}

/* ----------------------------------------------------------- */

//! Invoke the callback on each atom in the incoming set of handle h.
template<class T>
inline bool foreach_incoming_handle(Handle h, bool (T::*cb)(Handle), T *data)
{
    AtomSpace &as = atomspace();
    const std::vector<Handle> &vh = as.getIncoming(h);
    size_t sz = vh.size();

    for (size_t i = 0; i < sz; i++) {
        Handle hin = vh[i];
        bool rc = (data->*cb)(hin);
        if (rc) return rc;
    }
    return false;
}

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_FOREACH_H
