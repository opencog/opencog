/**
 * ForeachTwo.h
 *
 * Basic pattern-matching iterator constructs.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_FOREACH_TWO_H
#define _OPENCOG_FOREACH_TWO_H

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{

/**
 * Invoke the callback on each pair of corresponding atoms in
 * the outgoing sets of handles ha and hb. This iterator is
 * typically useful for making comparisons between atoms.
 */
template<class T>
inline bool foreach_outgoing_atom_pair(Handle ha, Handle hb,
                                       bool (T::*cb)(Atom *, Atom *), T *data)
{
    AtomSpace *as = &atomspace();
    boost::shared_ptr<Link> la(as->cloneLink(ha));
    boost::shared_ptr<Link> lb(as->cloneLink(hb));
    if (!la || !lb) return false;
    const std::vector<Handle> &va = la->getOutgoingSet();
    const std::vector<Handle> &vb = lb->getOutgoingSet();

    size_t vasz = va.size();
    size_t vbsz = vb.size();
    size_t minsz = std::min(vasz, vbsz);

    for (size_t i = 0; i < minsz; i++) {
        ha = va[i];
        hb = vb[i];
        boost::shared_ptr<Atom> aa(as->cloneLink(ha));
        boost::shared_ptr<Atom> ab(as->cloneLink(hb));
        bool rc = (data->*cb)(aa.get(), ab.get());
        if (rc) return rc;
    }

    for (size_t i = vasz; i < vbsz; i++) {
        hb = vb[i];
        boost::shared_ptr<Atom> ab(as->cloneLink(hb));
        bool rc = (data->*cb)(NULL, ab.get());
        if (rc) return rc;
    }
    for (size_t i = vbsz; i < vasz; i++) {
        ha = va[i];
        boost::shared_ptr<Atom> aa(as->cloneLink(ha));
        bool rc = (data->*cb)(aa.get(), NULL);
        if (rc) return rc;
    }
    return false;
}

} // namespace opencog

#endif // _OPENCOG_FOREACH_TWO_H
