/*
 * SchemeSmobAtom.c
 *
 * Scheme small objects (SMOBS) for opencog atom properties
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>
#include <vector>

#include "Atom.h"
#include "ClassServer.h"
#include "CogServer.h"
#include "Link.h"
#include "Node.h"
#include "SchemeSmob.h"
#include "SimpleTruthValue.h"
#include "TLB.h"

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
	Handle h = scm_to_ulong(shandle);
	return h;
}

const Atom * SchemeSmob::verify_atom (SCM satom, const char * subrname)
{
	return TLB::getAtom(verify_handle(satom, subrname));
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
		SCM sh = scm_from_ulong(h);
		SCM smob;
		SCM_NEWSMOB (smob, cog_handle_tag, sh);
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
	const Atom *atom = verify_atom(satom, "cog-incoming-set");

	HandleEntry *he = atom->getIncomingSet();
	if (!he) return SCM_EOL;

	SCM sh = scm_from_ulong(he->handle);
	SCM smob;
	SCM_NEWSMOB (smob, cog_handle_tag, sh);
	SCM head = scm_cons (smob, SCM_EOL);
	SCM tail = head;
	he = he->next;
	while (he)
	{
		SCM sh = scm_from_ulong(he->handle);
		SCM_NEWSMOB (smob, cog_handle_tag, sh);
		SCM pair = scm_cons (smob, SCM_EOL);
		scm_set_cdr_x(tail, pair);
		tail = pair;
		he = he->next;
	}

	return head;
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

SCM SchemeSmob::ss_tv (SCM satom)
{
	return SCM_EOL;
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

		SCM shandle = scm_from_ulong(h);
		SCM smob;
		SCM_NEWSMOB(smob, cog_handle_tag, shandle);
		SCM rc = scm_call_1(proc, smob);
		if (!scm_is_false(rc)) return rc;
	}

	return SCM_BOOL_F;
}

#endif

/* ===================== END OF FILE ============================ */
