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
		ret += "\"";

		// Print the truth value only after the node name
		const TruthValue &tv = atom->getTruthValue();
		if (tv != TruthValue::DEFAULT_TV())
		{
			ret += " ";
			ret += tv_to_string (&tv);
		}
		ret += ")";
		return ret;
	}
	else if (link)
	{
		// If there's a truth value, print it before the other atoms
		const TruthValue &tv = atom->getTruthValue();
		if (tv != TruthValue::DEFAULT_TV())
		{
			ret += " ";
			ret += tv_to_string (&tv);
		}

		// print the outgoing link set.
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
Type SchemeSmob::validate_atom (SCM stype, const char *subrname)
{
	if (scm_is_true(scm_symbol_p(stype)))
		stype = scm_symbol_to_string(stype);

	char * ct = scm_to_locale_string(stype);
	Type t = ClassServer::getType(ct);
	free(ct);

	// Make sure that the type is good
	if (NOTYPE == t)
		scm_wrong_type_arg_msg(subrname, 1, stype, "name of opencog atom type");

	return t;
}

Type SchemeSmob::validate_node (SCM stype, const char *subrname)
{
	Type t = validate_atom(stype, subrname);

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
		if (SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, satom))
		{
			// Get the handle  ... should we check for valid handles here?
			SCM shandle = SCM_SMOB_OBJECT(satom);
			Handle h = scm_to_ulong(shandle);
			outgoing_set.push_back(h);
		}
		else
		{
			// Its legit to have enmbedded truth values, just skip them.
			tv = get_tv_from_list(sl);
			if (tv == NULL)
			{
				// If its not an atom, and its not a truth value, its bad
				scm_wrong_type_arg_msg("cog-new-link", pos, satom, "opencog atom");
			}
		}
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
 * delete the atom, but only if it has no incoming links.
 */
SCM SchemeSmob::ss_delete (SCM satom)
{
	Handle h = verify_handle(satom, "cog-delete");

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
	Handle h = verify_handle(satom, "cog-delete-recursive");

	AtomSpace *as = CogServer::getAtomSpace();
	bool rc = as->removeAtom(h, true);

	if (rc) return SCM_BOOL_T;
	return SCM_BOOL_F;
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
	scm_c_define_gsubr("cog-delete",            1, 0, 0, C(ss_delete));
	scm_c_define_gsubr("cog-delete-recursive",  1, 0, 0, C(ss_delete_recursive));

	// property getters
	scm_c_define_gsubr("cog-incoming-set",      1, 0, 0, C(ss_incoming_set));
	scm_c_define_gsubr("cog-outgoing-set",      1, 0, 0, C(ss_outgoing_set));
	scm_c_define_gsubr("cog-name",              1, 0, 0, C(ss_name));
	scm_c_define_gsubr("cog-tv",                1, 0, 0, C(ss_tv));

	// Truth-values
	scm_c_define_gsubr("cog-new-stv",           2, 0, 0, C(ss_new_stv));

	// iterators
	scm_c_define_gsubr("cog-map-type",          2, 0, 0, C(ss_map_type));
	
}

#endif
/* ===================== END OF FILE ============================ */
