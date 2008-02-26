/**
 * ForeachTwo.h
 *
 * Basic pattern-matching iterator constructs.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_FOREACH_TWO_H_ 
#define OPENCOG_FOREACH_TWO_H_ 

#include "Atom.h"
#include "AtomSpace.h"
#include "TLB.h"

namespace opencog {

/**
 * Invoke the callback on each pair of corresponding atoms in 
 * the outgoing sets of handles ha and hb. This iterator is
 * typically useful for making comparisons between atoms.
 */
template<class T>
inline bool foreach_outgoing_atom_pair(Handle ha, Handle hb,
                                       bool (T::*cb)(Atom *, Atom *), T *data)
{
	Atom *aa = TLB::getAtom(ha);
	Atom *ab = TLB::getAtom(hb);
	const std::vector<Handle> &va = aa->getOutgoingSet();
	const std::vector<Handle> &vb = ab->getOutgoingSet();

	size_t vasz = va.size();
	size_t vbsz = vb.size();
	size_t minsz = min(vasz, vbsz);

	for (size_t i=0; i<minsz; i++)
	{
		ha = va[i];
		hb = vb[i];
		aa = TLB::getAtom(ha);
		ab = TLB::getAtom(hb);
		bool rc = (data->*cb)(aa, ab);
		if (rc) return rc;
	}

	for (size_t i=vasz; i<vbsz; i++)
	{
		hb = vb[i];
		ab = TLB::getAtom(hb);
		bool rc = (data->*cb)(NULL, ab);
		if (rc) return rc;
	}
	for (size_t i=vbsz; i<vasz; i++)
	{
		ha = va[i];
		aa = TLB::getAtom(ha);
		bool rc = (data->*cb)(aa, NULL);
		if (rc) return rc;
	}
	return false;
}

}
#endif /* OPENCOG_FOREACH_TWO_H_ */
