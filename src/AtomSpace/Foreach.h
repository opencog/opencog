/**
 * Foreach.h
 *
 * Basic iterator constructs.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

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

/**
 * Invoke the callback on each handle of the given type.
 */
template<class T>
inline bool foreach_handle_of_type(AtomSpace *as, 
                                 Type atype,
                                 bool (T::*cb)(Handle), T *data)
{
	std::list<Handle> handle_set;
	as->getHandleSet(back_inserter(handle_set), atype, NULL);

	// Loop over all handles in the handle set.
	std::list<Handle>::iterator i;
	for (i = handle_set.begin(); i != handle_set.end(); i++)
	{
		Handle h = *i;
		bool rc = (data->*cb)(h);
		if (rc) return rc;
	}
	return false;
}

template<class T>
inline bool foreach_handle_of_type(AtomSpace *as, 
                                 const char * atypename, 
                                 bool (T::*cb)(Handle), T *data)
{
	Type atype = ClassServer::getType(atypename);
	return foreach_handle_of_type(as, atype, cb, data);
}


}
