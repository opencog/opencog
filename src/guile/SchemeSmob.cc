/*
 * SchemeSmob.c
 *
 * Scheme small objects.
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
#include "TLB.h"

using namespace opencog;

/* ============================================================== */

bool SchemeSmob::is_inited = false;
scm_t_bits SchemeSmob::cog_tag;

SchemeSmob::SchemeSmob(void)
{
	if (!is_inited)
	{
		is_inited = true;
		init_smob_type();
		register_procs();
	}
}

/* ============================================================== */

int SchemeSmob::print_cog(SCM node, SCM port, scm_print_state * ps)
{
	scm_puts ("#<atom>", port);
	return 1; //non-zero meanss success
}

SCM SchemeSmob::equalp_cog(SCM a, SCM b)
{
	// They are equal if thier handles are the same.
	if (SCM_SMOB_OBJECT(a) == SCM_SMOB_OBJECT(b)) return SCM_BOOL_T;
	return SCM_BOOL_F;
}

void SchemeSmob::init_smob_type(void)
{
	cog_tag = scm_make_smob_type ("opencog_atom", sizeof (scm_t_bits));
	scm_set_smob_print (cog_tag, print_cog);
	scm_set_smob_equalp (cog_tag, equalp_cog);
}

/**
 * return atom->toString() for the corresponding atom.
 */
std::string SchemeSmob::to_string(SCM node)
{
	SCM shandle = SCM_SMOB_OBJECT(node);
	Handle h = scm_to_ulong(shandle);

	if (UNDEFINED_HANDLE == h) return "Undefined atom handle";

	if (h < NOTYPE) return "non-real atom";

	Atom *atom = TLB::getAtom(h);
	if (NULL == atom) return "Invalid handle";

	std::string ret = "#<";
	ret += atom->toString();
	ret += ">\n";
	return ret;
}

/* ============================================================== */
/**
 * Create a new scheme object, holding the atom handle
 */
SCM SchemeSmob::ss_atom (SCM shandle)
{
	if (scm_is_false(scm_integer_p(shandle)))
		scm_wrong_type_arg_msg("cog-atom", 1, shandle, "integer opencog handle");
	SCM_RETURN_NEWSMOB (cog_tag, shandle);
}

/* ============================================================== */
/**
 * Return handle of atom
 */
SCM SchemeSmob::ss_handle (SCM satom)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_tag, satom))
		scm_wrong_type_arg_msg("cog-handle", 1, satom, "opencog atom");
	return SCM_SMOB_OBJECT(satom);
}

/* ============================================================== */
/**
 * Create a new node, of named type stype, and string name sname
 */
SCM SchemeSmob::ss_new_node (SCM stype, SCM sname)
{
	if (scm_is_true(scm_symbol_p(stype)))
		stype = scm_symbol_to_string(stype);

   char * ct = scm_to_locale_string(stype);
	Type t = ClassServer::getType(ct);
	free(ct);

	// Make sure that the type is good
	if (NOTYPE == t)
		scm_wrong_type_arg_msg("cog-new-node", 1, stype, "name of opencog atom type");
		
	if (false == ClassServer::isAssignableFrom(NODE, t))
		scm_wrong_type_arg_msg("cog-new-node", 1, stype, "name of opencog node type");

	if (scm_is_false(scm_string_p(sname)))
		scm_wrong_type_arg_msg("cog-new-node", 2, sname, "string name for the node");

	// Now, create the actual node... in the actual atom space.
	char * cname = scm_to_locale_string(sname);
	std::string name = cname;
	free(cname);

	AtomSpace *as = CogServer::getAtomSpace();
	Handle h = as->addNode(t, name);

	SCM shandle = scm_from_ulong(h);

	SCM smob;
	SCM_NEWSMOB (smob, cog_tag, shandle);
	return smob;
}

/* ============================================================== */
/**
 * Create a new link, of named type stype, holding the indicated atom list
 */
SCM SchemeSmob::ss_new_link (SCM stype, SCM satom_list)
{
	if (scm_is_true(scm_symbol_p(stype)))
		stype = scm_symbol_to_string(stype);

   char * ct = scm_to_locale_string(stype);
	Type t = ClassServer::getType(ct);
	free(ct);

	// Make sure that the type is good
	if (NOTYPE == t)
		scm_wrong_type_arg_msg("cog-new-link", 1, stype, "name of opencog atom type");
		
	if (false == ClassServer::isAssignableFrom(LINK, t))
		scm_wrong_type_arg_msg("cog-new-link", 1, stype, "name of opencog link type");

	// Verify that second arg is an actual list
	if (!scm_is_pair(satom_list))
		scm_wrong_type_arg_msg("cog-new-link", 2, satom_list, "a list of atoms");

	std::vector<Handle> outgoing_set;
	SCM sl = satom_list;
	int pos = 2;
	do
	{
		SCM satom = SCM_CAR(sl);

		// Verify that the contents of the list are actual atoms.
		if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_tag, satom))
			scm_wrong_type_arg_msg("cog-new-link", pos, satom, "opencog atom");

		// Get the handle  ... should we check for valid handles here? 
		SCM shandle = SCM_SMOB_OBJECT(satom);
		Handle h = scm_to_ulong(shandle);

		outgoing_set.push_back(h);
		sl = SCM_CDR(sl);
		pos++;
	}
	while (scm_is_pair(sl));
	
	// Now, create the actual link... in the actual atom space.
	AtomSpace *as = CogServer::getAtomSpace();
	Handle h = as->addLink(t, outgoing_set);

	SCM shandle = scm_from_ulong(h);

	SCM smob;
	SCM_NEWSMOB (smob, cog_tag, shandle);
	return smob;
}

/* ============================================================== */
/**
 * Convert the outgoing set of an atom into a list; return the list.
 */
SCM SchemeSmob::ss_outgoing_set (SCM satom)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_tag, satom))
		scm_wrong_type_arg_msg("cog-outgoing-set", 1, satom, "opencog atom");

	SCM shandle = SCM_SMOB_OBJECT(satom);
	Handle h = scm_to_ulong(shandle);
	Link *l = dynamic_cast<Link*>(TLB::getAtom(h));
	if (l == NULL) return SCM_EOL;  // only links have outgoing sets.

	const std::vector<Handle> &oset = l->getOutgoingSet();

	SCM list = SCM_EOL;
	for (int i = oset.size()-1; i >= 0; i--)
	{
		Handle h = oset[i];
		SCM sh = scm_from_ulong(h);
		SCM smob;
		SCM_NEWSMOB (smob, cog_tag, sh);
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
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_tag, satom))
		scm_wrong_type_arg_msg("cog-incoming-set", 1, satom, "opencog atom");

	SCM shandle = SCM_SMOB_OBJECT(satom);
	Handle h = scm_to_ulong(shandle);
	Atom *atom = TLB::getAtom(h);

	HandleEntry *he = atom->getIncomingSet();
	if (!he) return SCM_EOL;

	SCM sh = scm_from_ulong(he->handle);
	SCM smob;
	SCM_NEWSMOB (smob, cog_tag, sh);
	SCM head = scm_cons (smob, SCM_EOL);
	SCM tail = head;
	he = he->next;
	while (he)
	{
		SCM sh = scm_from_ulong(he->handle);
		SCM_NEWSMOB (smob, cog_tag, sh);
		SCM pair = scm_cons (smob, SCM_EOL);
		scm_set_cdr_x(tail, pair);
		tail = pair;
		he = he->next;
	}

	return head;
}

/* ============================================================== */
/**
 * delete the atom, but only if it has no incoming links.
 */
SCM SchemeSmob::ss_delete (SCM satom)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_tag, satom))
		scm_wrong_type_arg_msg("cog-delete", 1, satom, "opencog atom");

	SCM shandle = SCM_SMOB_OBJECT(satom);
	Handle h = scm_to_ulong(shandle);
	Atom *atom = TLB::getAtom(h);

	HandleEntry *he = atom->getIncomingSet();
	if (he) 
		scm_wrong_type_arg_msg("cog-delete", 1, satom, "atom with empty incoming set");

	AtomSpace *as = CogServer::getAtomSpace();
	as->removeAtom(h, false);

	return SCM_EOL;
}

/* ============================================================== */
/**
 * delete the atom, and everything pointing to it.
 */
SCM SchemeSmob::ss_delete_recursive (SCM satom)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_tag, satom))
		scm_wrong_type_arg_msg("cog-delete", 1, satom, "opencog atom");

	SCM shandle = SCM_SMOB_OBJECT(satom);
	Handle h = scm_to_ulong(shandle);

	AtomSpace *as = CogServer::getAtomSpace();
	as->removeAtom(h, true);

	return SCM_EOL;
}


/* ============================================================== */

#define C(X) ((SCM (*) ()) X)

void SchemeSmob::register_procs(void)
{
	scm_c_define_gsubr("cog-new-link",            1, 0, 1, C(ss_new_link));
	scm_c_define_gsubr("cog-new-node",            2, 0, 0, C(ss_new_node));
	scm_c_define_gsubr("cog-atom",                1, 0, 0, C(ss_atom));
	scm_c_define_gsubr("cog-handle",              1, 0, 0, C(ss_handle));
	scm_c_define_gsubr("cog-incoming-set",        1, 0, 0, C(ss_incoming_set));
	scm_c_define_gsubr("cog-outgoing-set",        1, 0, 0, C(ss_outgoing_set));
	scm_c_define_gsubr("cog-delete",              1, 0, 0, C(ss_delete));
	scm_c_define_gsubr("cog-delete-recursive",    1, 0, 0, C(ss_delete_recursive));
}

#endif
/* ===================== END OF FILE ============================ */
