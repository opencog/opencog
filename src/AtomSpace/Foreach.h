/**
 * Foreach.h
 *
 * Basic iterator constructs.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_FOREACH_H_
#define OPENCOG_FOREACH_H_

#include "Atom.h"
#include "AtomSpace.h"
#include "TLB.h"

namespace opencog {

/**
 * Invoke the callback on each atom in the outgoing set of handle h.
 */
template<class T>
inline bool foreach_outgoing_handle(Handle h, bool (T::*cb)(Handle), T *data)
{
	Atom *atom = TLB::getAtom(h);
	const std::vector<Handle> &vh = atom->getOutgoingSet();

	for (size_t i=0; i<vh.size(); i++)
	{
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
	while (he)
	{
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
	while (he)
	{
		bool rc = (data->*cb)(he->handle);
		if (rc) return rc;
		he = he->next;
	}
	return false;
}


}
#endif /* OPENCOG_FOREACH_H_ */
