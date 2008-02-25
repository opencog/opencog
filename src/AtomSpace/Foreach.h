/**
 * Foreach.h
 *
 * Basic iterator constructs.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "Atom.h"
#include "TLB.h"

namespace opencog {

/**
 * Invoke the callback on each atom in the outgoing set of
 * handle h.
 */
template<class T>
inline bool foreach_outgoing(Handle h, bool (T::*cb)(Atom *), T *data)
{
	Atom *atom = TLB::getAtom(h);
	const std::vector<Handle> &vh = atom->getOutgoingSet();

	for (size_t i=0; i<vh.size(); i++)
	{
		Handle hout = vh[i];
		Atom *aout = TLB::getAtom(hout);
		bool rc = (data->*cb)(aout);
		if (rc) return rc;
	}
	return false;
}

}
