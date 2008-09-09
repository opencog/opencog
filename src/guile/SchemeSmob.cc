/*
 * SchemeSmob.c
 *
 * Scheme small objects (SMOBS) for opencog atoms and truth values.
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
 * Two scheme smob types are used to impelment the interface.
 *
 * The cog_handle_tag is used to store atom handles only.
 * The cog_misc_tag is used to store all other structures, such
 * as truth values. It is assumed that these structures are all
 * ephemeral (garbage-collected); this is in contrast to handles,
 * which are never garbage collected. Thus, opencog atoms have a
 * concrete existence outside of the scheme shell. By contrast,
 * truth values created by the scheme shell are garbage collected
 * by the shell.
 *
 * The type of the "misc" structure is stored in the flag bits;
 * thus, handling is dispatched based on these flags.
 */

enum
{
	COG_HANDLE = 1,
	COG_SIMPLE_TV,
};


bool SchemeSmob::is_inited = false;
scm_t_bits SchemeSmob::cog_handle_tag;
scm_t_bits SchemeSmob::cog_misc_tag;

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

int SchemeSmob::print_atom(SCM node, SCM port, scm_print_state * ps)
{
	std::string str = handle_to_string(node);
	scm_puts (str.c_str(), port);
	return 1; //non-zero means success
}

SCM SchemeSmob::equalp_atom(SCM a, SCM b)
{
	// Two atoms are equal if thier handles are the same.
	if (SCM_SMOB_OBJECT(a) == SCM_SMOB_OBJECT(b)) return SCM_BOOL_T;
	return SCM_BOOL_F;
}

size_t SchemeSmob::free_atom(SCM node)
{
	// Nothing to do here; the atom handles are stored as
	// immediate values in the SMOB's.
	return 0;
}

SCM SchemeSmob::mark_misc(SCM misc_smob)
{
	scm_t_bits misctype = SCM_SMOB_FLAGS(misc_smob);

	switch (misctype)
	{
		case COG_SIMPLE_TV: // Nothing to do here ...
			return SCM_BOOL_F;
		default:
			fprintf(stderr, "Error: opencog-guile: "
			        "don't know how to mark this type: %d\n",
			        (int) misctype);
			break;
	}

	return SCM_BOOL_F;
}

/**
 * Free the memory associated with an opencog guile object.
 * This routine is called by the guile garbage collector, from time to
 * time. For testing purposes, you can force the garbage collector to
 * run by saying (gc), while, for stats, try (gc-stats) and
 * (gc-live-object-stats). The later should show both "opencog-handle"
 * and "opencog-misc" stats.
 */
size_t SchemeSmob::free_misc(SCM node)
{
	scm_t_bits misctype = SCM_SMOB_FLAGS(node);

	switch (misctype)
	{
		case COG_SIMPLE_TV:
			SimpleTruthValue *stv;
			stv = (SimpleTruthValue *) SCM_SMOB_DATA(node);
			scm_gc_unregister_collectable_memory (stv,
			                  sizeof(SimpleTruthValue), "opencog simple tv");
			delete stv;
			return 0;

		default:
			fprintf(stderr, "Error: opencog-guile: "
			        "don't know how to free this type: %d\n",
			        (int) misctype);
			break;
	}
	return 0;
}

void SchemeSmob::init_smob_type(void)
{
	// a SMOB type for atom handles
	cog_handle_tag = scm_make_smob_type ("opencog-handle", sizeof (scm_t_bits));
	scm_set_smob_print (cog_handle_tag, print_atom);
	scm_set_smob_equalp (cog_handle_tag, equalp_atom);
	scm_set_smob_free (cog_handle_tag, free_atom);

	// A SMOB type for everything else
	cog_misc_tag = scm_make_smob_type ("opencog-misc", sizeof (scm_t_bits));
	scm_set_smob_mark (cog_misc_tag, mark_misc);
	scm_set_smob_free (cog_misc_tag, free_misc);
}

/* ============================================================== */

#ifdef USE_KEYWORD_LIST_NOT_USED
/**
 * Search for a truth value (demarked by #:tv) in a list of key-value
 * pairs.  Return the truth value if found, else return null.
 * Throw errors if the list is not stictly just key-value pairs
 *
 * XXX This code is not currently used, since it seems pointless
 * to have key-value pairs for this function. After all, an atom
 * can only have one truth value ever -- if we find a truth value, we
 * use it. We don't really need a key to tell us that its a truth value.
 * So punt, and get truth values implicitly. Meanwhile, this code is
 * stubbed out, for a rainy tay, in case we need to resurrect key-value
 * pairs in the future.
 */
static TruthValue *get_tv_from_kvp(SCM kvp, const char * subrname, int pos)
{
	if (!scm_is_pair(kvp)) return NULL;

	do
	{
		SCM skey = SCM_CAR(kvp);

		// Verify that the first item is a keyword.
		if (!scm_is_keyword(skey))
			scm_wrong_type_arg_msg(subrname, pos, skey, "keyword");

		skey = scm_keyword_to_symbol(skey);
		skey = scm_symbol_to_string(skey);
		char * key = scm_to_locale_string(skey);

		kvp = SCM_CDR(kvp);
		pos ++;
		if (!scm_is_pair(kvp))
		{
			free(key);
			scm_wrong_type_arg_msg(subrname, pos, kvp, "value following keyword");
		}

		if (0 == strcmp(key, "tv"))
		{
			SCM sval = SCM_CAR(kvp);
			scm_t_bits misctype = SCM_SMOB_FLAGS(sval);
			if (misctype != COG_SIMPLE_TV)
				scm_wrong_type_arg_msg(subrname, pos, sval, "opencog truth value");
			TruthValue *tv;
			tv = (TruthValue *) SCM_SMOB_DATA(sval);
			return tv;
		}
		free(key);

		kvp = SCM_CDR(kvp);
		pos ++;
	}
	while (scm_is_pair(kvp));

	return NULL;
}
#endif

/**
 * Search for a truth value in a list of values.
 * Return the truth value if found, else return null.
 * Throw errors if the list is not stictly just key-value pairs
 */
static TruthValue *get_tv_from_list(SCM slist)
{
	while (scm_is_pair(slist))
	{
		SCM sval = SCM_CAR(slist);
		scm_t_bits misctype = SCM_SMOB_FLAGS(sval);
		if (misctype == COG_SIMPLE_TV)
		{
			return ((TruthValue *) SCM_SMOB_DATA(sval));
		}

		slist = SCM_CDR(slist);
	}

	return NULL;
}

/* ============================================================== */
/**
 * return atom->toString() for the corresponding atom.
 */
std::string SchemeSmob::to_string(SCM node)
{
	if (SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, node))
		return handle_to_string(node);

	if (SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, node))
		return misc_to_string(node);

	return "";
}

std::string SchemeSmob::handle_to_string(Handle h, int indent)
{
	if (UNDEFINED_HANDLE == h) return "#<Undefined atom handle>";

	if (h <= NOTYPE) return "#<non-real atom>";

	Atom *atom = TLB::getAtom(h);
	if (NULL == atom) return "#<Invalid handle>";
	Node *node = dynamic_cast<Node *>(atom);
	Link *link = dynamic_cast<Link *>(atom);

#ifdef CRYPTIC_STYLE
	// output the olde-style opencog notation
	std::string ret = "#<";
	ret += atom->toString();
	ret += ">\n";
#endif

	// Print a scheme expression, so that the output can be saved
	// to file, and then restored, as needed.
	std::string ret = "";
	for (int i=0; i< indent; i++) ret += "   ";
	ret += "(";
	ret += ClassServer::getTypeName(atom->getType());
	if (node)
	{
		ret += " \"";
		ret += node->getName();
		ret += "\")";
		return ret;
	}
	else if (link)
	{
		std::vector<Handle> oset = link->getOutgoingSet();
		unsigned int arity = oset.size();
		for (unsigned int i=0; i<arity; i++)
		{
			ret += " ";
			ret += handle_to_string(oset[i], (0==i)?0:indent+1);
			if (i != arity-1) ret += "\n";
		}
		ret += ")";
		return ret;
	}

	return ret;
}

std::string SchemeSmob::handle_to_string(SCM node)
{
	SCM shandle = SCM_SMOB_OBJECT(node);
	Handle h = scm_to_ulong(shandle);

	return handle_to_string(h, 0) + "\n";
}

std::string SchemeSmob::misc_to_string(SCM node)
{
	std::string ret = "#<";
	scm_t_bits misctype = SCM_SMOB_FLAGS(node);
	switch (misctype)
	{
		case COG_SIMPLE_TV:
			SimpleTruthValue *stv;
			stv = (SimpleTruthValue *) SCM_SMOB_DATA(node);
			ret += "SimpleTruthValue ";
			ret += stv->toString();
			break;

		default:
			ret += "unknown opencog type";
			break;
	}

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
	SCM_RETURN_NEWSMOB (cog_handle_tag, shandle);
}

/* ============================================================== */
/**
 * Return handle of atom (the handle is in integer)
 */
SCM SchemeSmob::ss_handle (SCM satom)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, satom))
		scm_wrong_type_arg_msg("cog-handle", 1, satom, "opencog atom");
	return SCM_SMOB_OBJECT(satom);
}

/* ============================================================== */

/**
 * Check that the arguments represent a value node, else throw errors.
 * Return the node type.
 */
static Type validate_node (SCM stype, const char *subrname)
{
	if (scm_is_true(scm_symbol_p(stype)))
		stype = scm_symbol_to_string(stype);

	char * ct = scm_to_locale_string(stype);
	Type t = ClassServer::getType(ct);
	free(ct);

	// Make sure that the type is good
	if (NOTYPE == t)
		scm_wrong_type_arg_msg(subrname, 1, stype, "name of opencog atom type");

	if (false == ClassServer::isAssignableFrom(NODE, t))
		scm_wrong_type_arg_msg(subrname, 1, stype, "name of opencog node type");

	return t;
}

static std::string decode_string (SCM sname, const char *subrname)
{
	if (scm_is_false(scm_string_p(sname)))
		scm_wrong_type_arg_msg(subrname, 2, sname, "string name for the node");

	char * cname = scm_to_locale_string(sname);
	std::string name = cname;
	free(cname);
	return name;
}

/**
 * Create a new node, of named type stype, and string name sname
 */
SCM SchemeSmob::ss_new_node (SCM stype, SCM sname, SCM kv_pairs)
{
	Type t = validate_node(stype, "cog-new-node");
	std::string name = decode_string (sname, "cog-new-node");

	// Now, create the actual node... in the actual atom space.
	const TruthValue *tv = get_tv_from_list(kv_pairs);
	if (!tv) tv = &TruthValue::DEFAULT_TV();

	AtomSpace *as = CogServer::getAtomSpace();
	Handle h = as->addNode(t, name, *tv);

	SCM shandle = scm_from_ulong(h);
	SCM_RETURN_NEWSMOB (cog_handle_tag, shandle);
}

/**
 * Return the indicated node, of named type stype, and string name sname
 * if it exists; else return nil if it does not exist.
 * If the node exists, *and* a truth value was specified, then change
 * the truth value.
 */
SCM SchemeSmob::ss_node (SCM stype, SCM sname, SCM kv_pairs)
{
	Type t = validate_node(stype, "cog-node");
	std::string name = decode_string (sname, "cog-node");

	// Now, look for the actual node... in the actual atom space.
	AtomSpace *as = CogServer::getAtomSpace();
	Handle h = as->getHandle(t, name);
	if (!TLB::isValidHandle(h)) return SCM_EOL; // NIL

	// If there was a truth value, change it.
	const TruthValue *tv = get_tv_from_list(kv_pairs);
	if (tv)
	{
		Atom *atom = TLB::getAtom(h);
		atom->setTruthValue(*tv);
	}

	SCM shandle = scm_from_ulong(h);
	SCM_RETURN_NEWSMOB (cog_handle_tag, shandle);
}

/* ============================================================== */
/**
 * Verify that the arguments are appropriate for a link
 */
static Type verify_link (SCM stype, const char * subrname)
{
	if (scm_is_true(scm_symbol_p(stype)))
		stype = scm_symbol_to_string(stype);

	char * ct = scm_to_locale_string(stype);
	Type t = ClassServer::getType(ct);
	free(ct);

	// Make sure that the type is good
	if (NOTYPE == t)
		scm_wrong_type_arg_msg(subrname, 1, stype, "name of opencog atom type");

	if (false == ClassServer::isAssignableFrom(LINK, t))
		scm_wrong_type_arg_msg(subrname, 1, stype, "name of opencog link type");

	return t;
}

/**
 * Convert argument into a list of handles.
 */
std::vector<Handle>
SchemeSmob::decode_handle_list (SCM satom_list, const char * subrname)
{
	// Verify that second arg is an actual list
	if (!scm_is_pair(satom_list))
		scm_wrong_type_arg_msg(subrname, 2, satom_list, "a list of atoms");

	const TruthValue *tv = NULL;
	std::vector<Handle> outgoing_set;
	SCM sl = satom_list;
	int pos = 2;
	do
	{
		SCM satom = SCM_CAR(sl);

		// Verify that the contents of the list are actual atoms.
		if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, satom))
		{
			// Fish out a truth value, if its there.
			tv = get_tv_from_list(sl);
			if (tv != NULL) break;

			// If its not an atom, and its not a truth value, its bad
			scm_wrong_type_arg_msg("cog-new-link", pos, satom, "opencog atom");
		}

		// Get the handle  ... should we check for valid handles here?
		SCM shandle = SCM_SMOB_OBJECT(satom);
		Handle h = scm_to_ulong(shandle);

		outgoing_set.push_back(h);
		sl = SCM_CDR(sl);
		pos++;
	}
	while (scm_is_pair(sl));

	return outgoing_set;
}

/**
 * Create a new link, of named type stype, holding the indicated atom list
 */
SCM SchemeSmob::ss_new_link (SCM stype, SCM satom_list)
{
	Type t = verify_link (stype, "cog-new-link");

	std::vector<Handle> outgoing_set;
	outgoing_set = decode_handle_list (satom_list, "cog-new-link");

	// Fish out a truth value, if its there.
	const TruthValue *tv = get_tv_from_list(satom_list);
	if (!tv) tv = &TruthValue::DEFAULT_TV();

	// Now, create the actual link... in the actual atom space.
	AtomSpace *as = CogServer::getAtomSpace();
	Handle h = as->addLink(t, outgoing_set, *tv);

	SCM shandle = scm_from_ulong(h);

	SCM_RETURN_NEWSMOB (cog_handle_tag, shandle);
}

/**
 * Return the indicated link, of named type stype, holding the
 * indicated atom list, if it exists; else return nil if
 * it does not exist.
 */
SCM SchemeSmob::ss_link (SCM stype, SCM satom_list)
{
	Type t = verify_link (stype, "cog-link");

	std::vector<Handle> outgoing_set;
	outgoing_set = decode_handle_list (satom_list, "cog-link");

	// Now, look to find the actual link... in the actual atom space.
	AtomSpace *as = CogServer::getAtomSpace();
	Handle h = as->getHandle(t, outgoing_set);
	if (!TLB::isValidHandle(h)) return SCM_EOL; // NIL

	// If there was a truth value, change it.
	const TruthValue *tv = get_tv_from_list(satom_list);
	if (tv)
	{
		Atom *atom = TLB::getAtom(h);
		atom->setTruthValue(*tv);
	}

	SCM shandle = scm_from_ulong(h);
	SCM_RETURN_NEWSMOB (cog_handle_tag, shandle);
}

/* ============================================================== */
/**
 * Convert the outgoing set of an atom into a list; return the list.
 */
SCM SchemeSmob::ss_outgoing_set (SCM satom)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, satom))
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
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, satom))
		scm_wrong_type_arg_msg("cog-incoming-set", 1, satom, "opencog atom");

	SCM shandle = SCM_SMOB_OBJECT(satom);
	Handle h = scm_to_ulong(shandle);
	Atom *atom = TLB::getAtom(h);

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
 * delete the atom, but only if it has no incoming links.
 */
SCM SchemeSmob::ss_delete (SCM satom)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, satom))
		scm_wrong_type_arg_msg("cog-delete", 1, satom, "opencog atom");

	SCM shandle = SCM_SMOB_OBJECT(satom);
	Handle h = scm_to_ulong(shandle);

	AtomSpace *as = CogServer::getAtomSpace();
	bool rc = as->removeAtom(h, false);

	if (rc) return SCM_BOOL_T;
	return SCM_BOOL_F;
}

/* ============================================================== */
/**
 * delete the atom, and everything pointing to it.
 */
SCM SchemeSmob::ss_delete_recursive (SCM satom)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, satom))
		scm_wrong_type_arg_msg("cog-delete-recursive", 1, satom, "opencog atom");

	SCM shandle = SCM_SMOB_OBJECT(satom);
	Handle h = scm_to_ulong(shandle);

	AtomSpace *as = CogServer::getAtomSpace();
	bool rc = as->removeAtom(h, true);

	if (rc) return SCM_BOOL_T;
	return SCM_BOOL_F;
}


/* ============================================================== */
/**
 * Create a new simple truth value, with indicated mean and confidence.
 */
SCM SchemeSmob::ss_new_stv (SCM smean, SCM sconfidence)
{
	double mean = scm_to_double(smean);
	double confidence = scm_to_double(sconfidence);

	float cnt = SimpleTruthValue::confidenceToCount(confidence);
	SimpleTruthValue *stv = new SimpleTruthValue(mean, cnt);
	scm_gc_register_collectable_memory (stv,
	                 sizeof(SimpleTruthValue), "opencog simple tv");

	SCM smob;
	SCM_NEWSMOB (smob, cog_misc_tag, stv);
	SCM_SET_SMOB_FLAGS(smob, COG_SIMPLE_TV);
	return smob;
}

/* ============================================================== */

#define C(X) ((SCM (*) ()) X)

void SchemeSmob::register_procs(void)
{
	scm_c_define_gsubr("cog-atom",              1, 0, 0, C(ss_atom));
	scm_c_define_gsubr("cog-handle",            1, 0, 0, C(ss_handle));
	scm_c_define_gsubr("cog-new-node",          2, 0, 1, C(ss_new_node));
	scm_c_define_gsubr("cog-new-link",          1, 0, 1, C(ss_new_link));
	scm_c_define_gsubr("cog-node",              2, 0, 1, C(ss_node));
	scm_c_define_gsubr("cog-link",              1, 0, 1, C(ss_link));
	scm_c_define_gsubr("cog-incoming-set",      1, 0, 0, C(ss_incoming_set));
	scm_c_define_gsubr("cog-outgoing-set",      1, 0, 0, C(ss_outgoing_set));
	scm_c_define_gsubr("cog-delete",            1, 0, 0, C(ss_delete));
	scm_c_define_gsubr("cog-delete-recursive",  1, 0, 0, C(ss_delete_recursive));

	// Truth-values
	scm_c_define_gsubr("cog-new-stv",           2, 0, 0, C(ss_new_stv));
}

#endif
/* ===================== END OF FILE ============================ */
