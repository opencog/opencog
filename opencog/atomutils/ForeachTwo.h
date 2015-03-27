/**
 * ForeachTwo.h
 *
 * Basic pattern-matching iterator constructs.
 *
 * Copyright (c) 2008,2011 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_FOREACH_TWO_H
#define _OPENCOG_FOREACH_TWO_H

#include <opencog/atomspace/Handle.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * Invoke the callback on each pair of corresponding atoms in the
 * vectors va and vb. This iterator is typically useful for making
 * comparisons between atoms.
 *
 * Returns true if all calls return true, return false otherwise.
 */
template<class T>
inline bool foreach_atom_pair(const std::vector<Handle> &va,
                              const std::vector<Handle> &vb,
                              bool (T::*cb)(Handle, Handle), T *data)
{
    size_t vasz = va.size();
    size_t vbsz = vb.size();
    size_t minsz = std::min(vasz, vbsz);

    for (size_t i = 0; i < minsz; i++) {
        bool rc = (data->*cb)(va[i], vb[i]);
        if (not rc) return false;
    }

    for (size_t i = vasz; i < vbsz; i++) {
        bool rc = (data->*cb)(Handle::UNDEFINED, vb[i]);
        if (not rc) return false;
    }
    for (size_t i = vbsz; i < vasz; i++) {
        bool rc = (data->*cb)(va[i], Handle::UNDEFINED);
        if (not rc) return false;
    }
    return true;
}

/** @}*/
} // namespace opencog

#endif // _OPENCOG_FOREACH_TWO_H
