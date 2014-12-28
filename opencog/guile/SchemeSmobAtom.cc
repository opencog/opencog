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

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

/* ============================================================== */
/**
 * Verify that SCM arg is an actual opencog Handle; returning Handle
 *
 * This routine is meant for validating arguments passed into
 * guile-wrapped C++ code.
 *
 * This routine takes an SCM arg and a string subroutine name. It
 * verifies that the arg is actually a handle (and not, for example,
 * an int, string, etc.)  If its not a handle, it throws an SCM error,
 * using the subroutine name in the error message.  (Such an error is
 * caught by the shell, and printed as a stack trace at the shell
 * prompt).  If the arg is a handle, then the actual opencog handle
 * is returned.
 */

Handle SchemeSmob::verify_handle (SCM satom, const char * subrname, int pos)
{
	Handle h(scm_to_handle(satom));
	if (Handle::UNDEFINED == h)
		scm_wrong_type_arg_msg(subrname, pos, satom, "opencog atom");

	return h;
}

/* ============================================================== */
/**
 * Return the string name of the atom
 */
SCM SchemeSmob::ss_name (SCM satom)
{
	std::string name;
	Handle h = verify_handle(satom, "cog-name");
	NodePtr nnn(NodeCast(h));
	if (nnn) name = nnn->getName();
	SCM str = scm_from_locale_string(name.c_str());
	return str;
}

SCM SchemeSmob::ss_type (SCM satom)
{
	Handle h = verify_handle(satom, "cog-type");
	Type t = h->getType();
	const std::string &tname = classserver().getTypeName(t);
	SCM str = scm_from_locale_string(tname.c_str());
	SCM sym = scm_string_to_symbol(str);

	return sym;
}

SCM SchemeSmob::ss_arity (SCM satom)
{
	Handle h = verify_handle(satom, "cog-arity");
	Arity ari = 0;
	LinkPtr lll(LinkCast(h));
	if (lll) ari = lll->getArity();

	/* Arity is currently an unsigned short */
	SCM sari = scm_from_ushort(ari);
	return sari;
}

SCM SchemeSmob::ss_tv (SCM satom)
{
	Handle h = verify_handle(satom, "cog-tv");
	TruthValuePtr tv(h->getTruthValue());
	TruthValue *stv = tv->rawclone();
	return take_tv(stv);
}

SCM SchemeSmob::ss_set_tv (SCM satom, SCM stv)
{
	Handle h = verify_handle(satom, "cog-set-tv!");
	TruthValue *tv = verify_tv(stv, "cog-set-tv!", 2);

	h->setTruthValue(tv->clone());
	scm_remember_upto_here_1(stv);
	return satom;
}

SCM SchemeSmob::ss_av (SCM satom)
{
	Handle h = verify_handle(satom, "cog-av");
	AttentionValue *sav = h->getAttentionValue()->rawclone();
	return take_av(sav);
}

SCM SchemeSmob::ss_set_av (SCM satom, SCM sav)
{
	Handle h = verify_handle(satom, "cog-set-av!");
	AttentionValue *av = verify_av(sav, "cog-set-av!", 2);

	h->setAttentionValue(av->clone());
	return satom;
}

SCM SchemeSmob::ss_inc_vlti (SCM satom)
{
	Handle h = verify_handle(satom, "cog-inc-vlti!");

	h->incVLTI();
	return satom;
}

SCM SchemeSmob::ss_dec_vlti (SCM satom)
{
	Handle h = verify_handle(satom, "cog-dec-vlti!");

	h->decVLTI();
	return satom;
}

/* ============================================================== */
/**
 * Convert the outgoing set of an atom into a list; return the list.
 */
SCM SchemeSmob::ss_outgoing_set (SCM satom)
{
	Handle h = verify_handle(satom, "cog-outgoping-set");

	LinkPtr lll(LinkCast(h));
	if (NULL == lll) return SCM_EOL;

	const HandleSeq& oset = lll->getOutgoingSet();

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
	AtomSpace* atomspace = ss_get_env_as("cog-incoming-set");

	std::vector<Handle> iset = atomspace->getIncoming(h);
	size_t isz = iset.size();
	if (0 == isz) return SCM_EOL;

	// This reverses the order of the incoming set, but so what ...
	SCM head = SCM_EOL;
	for (size_t i=0; i<isz; i++) {
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
	Type t = verify_atom_type (stype, "cog-map-type");
	AtomSpace* atomspace = ss_get_env_as("cog-map-type");

	// Get all of the handles of the indicated type
	std::list<Handle> handle_set;
	atomspace->getHandlesByType(back_inserter(handle_set), t, false);

	// Loop over all handles in the handle set.
	// Call proc on each handle, in turn.
	// Break out of the loop if proc returns anything other than #f
	std::list<Handle>::iterator i;
	for (i = handle_set.begin(); i != handle_set.end(); ++i) {
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
	while (1) {
		t--;
		const std::string &tname = classserver().getTypeName(t);
		SCM str = scm_from_locale_string(tname.c_str());
		SCM sym = scm_string_to_symbol(str);
		list = scm_cons(sym, list);
		if (0 == t) break;
	}

	return list;
}

/**
 * Return a list of the subtypes of the indicated type
 */
SCM SchemeSmob::ss_get_subtypes (SCM stype)
{
	SCM list = SCM_EOL;

	Type t = verify_atom_type(stype, "cog-get-subtypes");
	std::vector<Type> subl;
	unsigned int ns = classserver().getChildren(t, std::back_inserter(subl));

	for (unsigned int i=0; i<ns; i++) {
		t = subl[i];
		const std::string &tname = classserver().getTypeName(t);
		SCM str = scm_from_locale_string(tname.c_str());
		SCM sym = scm_string_to_symbol(str);
		list = scm_cons(sym, list);
	}

	return list;
}

/**
 * Return true if a type
 */
SCM SchemeSmob::ss_type_p (SCM stype)
{
	if (scm_is_true(scm_symbol_p(stype)))
		stype = scm_symbol_to_string(stype);

	if (scm_is_false(scm_string_p(stype)))
		return SCM_BOOL_F;

	char * ct = scm_to_locale_string(stype);
	Type t = classserver().getType(ct);
	free(ct);

	if (NOTYPE == t) return SCM_BOOL_F;

	return SCM_BOOL_T;
}

/**
 * Return true if a subtype
 */
SCM SchemeSmob::ss_subtype_p (SCM stype, SCM schild)
{
	if (scm_is_true(scm_symbol_p(stype)))
		stype = scm_symbol_to_string(stype);

	if (scm_is_false(scm_string_p(stype)))
		return SCM_BOOL_F;

	char * ct = scm_to_locale_string(stype);
	Type parent = classserver().getType(ct);
	free(ct);

	if (NOTYPE == parent) return SCM_BOOL_F;

	// Now investigate the child ...
	if (scm_is_true(scm_symbol_p(schild)))
		schild = scm_symbol_to_string(schild);

	if (scm_is_false(scm_string_p(schild)))
		return SCM_BOOL_F;

	ct = scm_to_locale_string(schild);
	Type child = classserver().getType(ct);
	free(ct);

	if (NOTYPE == child) return SCM_BOOL_F;

	if (classserver().isA(child, parent)) return SCM_BOOL_T;

	return SCM_BOOL_F;
}

#endif

/* ===================== END OF FILE ============================ */
