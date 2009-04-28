/*
 * SchemeSmobAtom.c
 *
 * Scheme small objects (SMOBS) for opencog atom properties
 *
 * Copyright (c) 2008,2009 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <vector>

#include <libguile.h>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/server/CogServer.h>

using namespace opencog;

/* ============================================================== */
/**
 * Convert SCM to atom pointer
 */
Handle SchemeSmob::verify_handle (SCM satom, const char * subrname)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, satom))
		scm_wrong_type_arg_msg(subrname, 1, satom, "opencog atom");

	SCM shandle = SCM_SMOB_OBJECT(satom);
	Handle h(scm_to_ulong(shandle));
	return h;
}

Atom * SchemeSmob::verify_atom (SCM satom, const char * subrname)
{
	return TLB::getAtom(verify_handle(satom, subrname));
}

/* ============================================================== */
/**
 * Return the string name of the atom
 */
SCM SchemeSmob::ss_name (SCM satom)
{
	const Atom *atom = verify_atom(satom, "cog-name");
	const Node *node = dynamic_cast<const Node *>(atom);
	if (NULL == node) return SCM_EOL;
	std::string name = node->getName();
	SCM str = scm_from_locale_string(name.c_str());
	return str;
}

SCM SchemeSmob::ss_type (SCM satom)
{
	const Atom *atom = verify_atom(satom, "cog-type");
	Type t = atom->getType();
	const std::string &tname = classserver().getTypeName(t);
	SCM str = scm_from_locale_string(tname.c_str());
	SCM sym = scm_string_to_symbol(str);

	return sym;
}

SCM SchemeSmob::ss_arity (SCM satom)
{
	const Atom *atom = verify_atom(satom, "cog-arity");
	Arity ari = 0;
	const Link *l = dynamic_cast<const Link *>(atom);
	if (l) ari = l->getArity();

	/* Arity is currently an unsigned short */
	SCM sari = scm_from_ushort(ari);
	return sari;
}

SCM SchemeSmob::ss_tv (SCM satom)
{
	const Atom *atom = verify_atom(satom, "cog-tv");
	const TruthValue &tv = atom->getTruthValue();
	TruthValue *stv = tv.clone();
	return take_tv(stv);
}

SCM SchemeSmob::ss_set_tv (SCM satom, SCM stv)
{
	Atom *atom = verify_atom(satom, "cog-set-tv!");
	TruthValue *tv = verify_tv(stv, "cog-set-tv!");

	atom->setTruthValue(*tv);
	return satom;
}

/* ============================================================== */
/**
 * Convert the outgoing set of an atom into a list; return the list.
 */
SCM SchemeSmob::ss_outgoing_set (SCM satom)
{
	const Atom *atom = verify_atom(satom, "cog-outgoing-set");
	const Link *l = dynamic_cast<const Link *>(atom);
	if (l == NULL) return SCM_EOL;  // only links have outgoing sets.

	const std::vector<Handle> &oset = l->getOutgoingSet();

	SCM list = SCM_EOL;
	for (int i = oset.size()-1; i >= 0; i--)
	{
		Handle h = oset[i];
		SCM smob = handle_to_scm(h);
		list = scm_cons (smob, list);
	}

	return list;
}

/* ============================================================== */
/**
 * Convert the incoming set of an atom into a list; return the list.
 */
SCM SchemeSmob::ss_incoming_set (SCM satom)
{
	Handle h = verify_handle(satom, "cog-incoming-set");

	AtomSpace *as = CogServer::getAtomSpace();
	std::vector<Handle> iset = as->getIncoming(h);
	size_t isz = iset.size();
	if (0 == isz) return SCM_EOL;

	// This reverses the order of the incoming set, but so what ... 
	SCM head = SCM_EOL;
	for (size_t i=0; i<isz; i++)
	{
		Handle hi = iset[i];
		SCM smob = handle_to_scm(hi);
		head = scm_cons(smob, head);
	}

	return head;
}

/* ============================================================== */

/**
 * Apply proceedure proc to all atoms of type stype
 * If the proceedure returns something other than #f, 
 * terminate the loop.
 */
SCM SchemeSmob::ss_map_type (SCM proc, SCM stype)
{
	Type t = validate_atom (stype, "cog-map-type");

	// Get all of the handles of the indicated type
	std::list<Handle> handle_set;
	AtomSpace *as = CogServer::getAtomSpace();
	as->getHandleSet(back_inserter(handle_set), t, false);

	// Loop over all handles in the handle set.
	// Call proc on each handle, in turn.
	// Break out of the loop if proc returns anything other than #f
	std::list<Handle>::iterator i;
	for (i = handle_set.begin(); i != handle_set.end(); i++)
	{
		Handle h = *i;
		SCM smob = handle_to_scm(h);
		SCM rc = scm_call_1(proc, smob);
		if (!scm_is_false(rc)) return rc;
	}

	return SCM_BOOL_F;
}

/* ============================================================== */

/**
 * Return a list of all of the atom types in the system.
 */
SCM SchemeSmob::ss_get_types (void)
{
	SCM list = SCM_EOL;

	Type t = classserver().getNumberOfClasses();
	while (1)
	{
		t--;
		const std::string &tname = classserver().getTypeName(t);
		SCM str = scm_from_locale_string(tname.c_str());
		SCM sym = scm_string_to_symbol(str);
		list = scm_cons(sym, list);
		if (0 == t) break;
	}

	return list;
}

#endif

/* ===================== END OF FILE ============================ */
