/*
 * SchemeSmobNew.cc
 *
 * Scheme small objects (SMOBS) --creating new atoms -- for opencog.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <vector>

#include <libguile.h>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/server/CogServer.h>

using namespace opencog;

/* ============================================================== */
/**
 * Return a string holding the scheme representation of an atom/truthvalue.
 *
 * The input is assumed to be pointing at a Handle, a TruthValue, 
 * an AttentionValue, or a VersionHandle. Returned is a valid scheme
 * expression that represents that Handle, etc. 
 */
std::string SchemeSmob::to_string(SCM node)
{
    if (SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, node))
        return handle_to_string(node);

    if (SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, node))
        return misc_to_string(node);

    return "";
}

/**
 * Return a string holding the scheme representation of an atom.
 *
 * The input handle is represented in terms of a valid scheme 
 * expression. Evaluating this expression should result in exactly
 * the same atom being created.
 */
std::string SchemeSmob::to_string(Handle h)
{
    return handle_to_string(h, 0);
}

std::string SchemeSmob::handle_to_string(Handle h, int indent)
{
    if (Handle::UNDEFINED == h) return "#<Undefined atom handle>";

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
    ret += classserver().getTypeName(atom->getType());
    if (node) {
        ret += " \"";
        ret += node->getName();
        ret += "\"";
        
        // Print the truth value only after the node name
        const TruthValue &tv = atom->getTruthValue();
        if (tv != TruthValue::DEFAULT_TV()) {
            ret += " ";
            ret += tv_to_string (&tv);
        }

        // Print the attention value after the truth value
        const AttentionValue &av = atom->getAttentionValue();
        if (av != AttentionValue::DEFAULT_AV()) {
            ret += " ";
            ret += av_to_string (&av);
        }
        ret += ")";
        return ret;
    }
    else if (link) {
        // If there's a truth value, print it before the other atoms
        const TruthValue &tv = atom->getTruthValue();
        if (tv != TruthValue::DEFAULT_TV()) {
            ret += " ";
            ret += tv_to_string (&tv);
        }

        // Print the attention value after the truth value
        const AttentionValue &av = atom->getAttentionValue();
        if (av != AttentionValue::DEFAULT_AV()) {
            ret += " ";
            ret += av_to_string (&av);
        }

        // print the outgoing link set.
        std::vector<Handle> oset = link->getOutgoingSet();
        unsigned int arity = oset.size();
        for (unsigned int i=0; i<arity; i++) {
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
    Handle h = scm_to_handle(node);
    return handle_to_string(h, 0) + "\n";
}

/* ============================================================== */
/**
 * Wrapper -- given a handle, return the corresponding scheme object.
 * Note that smobs hold the integer value of the handle, and not the
 * C++ handle object itself.
 */
SCM SchemeSmob::handle_to_scm (Handle h)
{
    UUID uuid = h.value();
    SCM shandle = scm_from_ulong(uuid);
    SCM_RETURN_NEWSMOB (cog_handle_tag, shandle);
}

Handle SchemeSmob::scm_to_handle (SCM sh)
{
    SCM suuid = SCM_SMOB_OBJECT(sh);
    UUID uuid = scm_to_ulong(suuid);
    return Handle(uuid);
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
 * Return handle of atom (the handle is the UUID integer)
 */
SCM SchemeSmob::ss_handle (SCM satom)
{
    if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, satom))
        scm_wrong_type_arg_msg("cog-handle", 1, satom, "opencog atom");

    // We store the uuid, so just return that.
    return SCM_SMOB_OBJECT(satom);
}

/* ============================================================== */
/** Return true if s is an atom */

SCM SchemeSmob::ss_atom_p (SCM s)
{
    if (SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, s))
        return SCM_BOOL_T;
    return SCM_BOOL_F;
}

/* ============================================================== */
/** Return true if s is a node */

SCM SchemeSmob::ss_node_p (SCM s)
{
    if (! SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, s))
        return SCM_BOOL_F;

    SCM shandle = SCM_SMOB_OBJECT(s);
    UUID uuid = scm_to_ulong(shandle);
    Handle h(uuid);
    Atom *a = TLB::getAtom(h);

    if (dynamic_cast<Node *>(a)) return SCM_BOOL_T;

    return SCM_BOOL_F;
}

/* ============================================================== */
/** Return true if s is a link */

SCM SchemeSmob::ss_link_p (SCM s)
{
    if (! SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, s))
        return SCM_BOOL_F;

    SCM shandle = SCM_SMOB_OBJECT(s);
    UUID uuid = scm_to_ulong(shandle);
    Handle h(uuid);
    Atom *a = TLB::getAtom(h);

    if (dynamic_cast<Link *>(a)) return SCM_BOOL_T;

    return SCM_BOOL_F;
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

    if (scm_is_false(scm_string_p(stype)))
        scm_wrong_type_arg_msg(subrname, 1, stype, "name of opencog atom type");

    char * ct = scm_to_locale_string(stype);
    Type t = classserver().getType(ct);
    free(ct);

    // Make sure that the type is good
    if (NOTYPE == t)
        scm_wrong_type_arg_msg(subrname, 1, stype, "name of opencog atom type");

    return t;
}

Type SchemeSmob::validate_node (SCM stype, const char *subrname)
{
    Type t = validate_atom(stype, subrname);

    if (false == classserver().isA(t, NODE))
        scm_wrong_type_arg_msg(subrname, 1, stype, "name of opencog node type");

    return t;
}

std::string SchemeSmob::decode_string (SCM sname, const char *subrname, const char * msg)
{
    if (scm_is_false(scm_string_p(sname)))
        scm_wrong_type_arg_msg(subrname, 2, sname, msg);

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
    std::string name = decode_string (sname, "cog-new-node", "string name for the node");

    // Now, create the actual node... in the actual atom space.
    const TruthValue *tv = get_tv_from_list(kv_pairs);
    if (!tv) tv = &TruthValue::DEFAULT_TV();
    
    AtomSpace *as = CogServer::getAtomSpace();
    Handle h = as->addNode(t, name, *tv);

    // Was an attention value explicitly specified?
    // If so, then we've got to set it.
    const AttentionValue *av = get_av_from_list(kv_pairs);
    if (av) {
        Atom *a = TLB::getAtom(h);
        a->setAttentionValue(*av);
    }

    return handle_to_scm (h);
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
    std::string name = decode_string (sname, "cog-node", "string name for the node");

    // Now, look for the actual node... in the actual atom space.
    AtomSpace *as = CogServer::getAtomSpace();
    Handle h = as->getHandle(t, name);
    if (!TLB::isValidHandle(h)) return SCM_EOL; // NIL

    Atom *atom = NULL;
    // If there was a truth value, change it.
    const TruthValue *tv = get_tv_from_list(kv_pairs);
    if (tv) {
        if (NULL == atom) atom = TLB::getAtom(h);
        atom->setTruthValue(*tv);
    }

    // If there was an attention value, change it.
    const AttentionValue *av = get_av_from_list(kv_pairs);
    if (av) {
        if (NULL == atom) atom = TLB::getAtom(h);
        atom->setAttentionValue(*av);
    }
    return handle_to_scm (h);
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
    Type t = classserver().getType(ct);
    free(ct);

    // Make sure that the type is good
    if (NOTYPE == t)
        scm_wrong_type_arg_msg(subrname, 1, stype, "name of opencog atom type");

    if (false == classserver().isA(t, LINK))
        scm_wrong_type_arg_msg(subrname, 1, stype, "name of opencog link type");

    return t;
}

/**
 * Convert argument into a list of handles.
 */
std::vector<Handle>
SchemeSmob::decode_handle_list (SCM satom_list, const char * subrname)
{
    // Verify that second arg is an actual list. Allow null list
    // (which is rather unusal, but legit.  Allow embedded nulls
    // as this can be convenient for writing scheme code.
    if (!scm_is_pair(satom_list) && !scm_is_null(satom_list))
        scm_wrong_type_arg_msg(subrname, 2, satom_list, "a list of atoms");

    std::vector<Handle> outgoing_set;
    SCM sl = satom_list;
    int pos = 2;
    while (scm_is_pair(sl)) {
        SCM satom = SCM_CAR(sl);

        // Verify that the contents of the list are actual atoms.
        if (SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, satom)) {
            // Get the handle  ... should we check for valid handles here?
            SCM shandle = SCM_SMOB_OBJECT(satom);
            UUID uuid = scm_to_ulong(shandle);
            Handle h(uuid);
            outgoing_set.push_back(h);
        }
        else if (scm_is_pair(satom) && !scm_is_null(satom_list)) {
            // Allow lists to be specified: e.g. 
            // (cog-new-link 'ListLink (list x y z))
            // Do this via a recursive call, flattening nested lists
            // as we go along.
            const std::vector<Handle> &oset =
                decode_handle_list(satom, subrname);
            std::vector<Handle>::const_iterator it;
            for (it = oset.begin(); it != oset.end(); it++) {
                outgoing_set.push_back(*it);
            }
        }
        else if (scm_is_null(satom)) {
            // No-op, just ignore.
        }
        else {
            // Its legit to have embedded truth values, just skip them.
            const TruthValue *tv = get_tv_from_list(sl);
            const AttentionValue *av = get_av_from_list(sl);
            if ((tv == NULL) && (av == NULL)) {
                // If its not an atom, and its not a truth value, and its
                // not an attention value, then whatever it is, its bad.
                scm_wrong_type_arg_msg("cog-new-link", pos, satom, "opencog atom");
            }
        }
        sl = SCM_CDR(sl);
        pos++;
    }

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

    // Was an attention value explicitly specified?
    // If so, then we've got to set it.
    const AttentionValue *av = get_av_from_list(satom_list);
    if (av) {
        Atom *a = TLB::getAtom(h);
        a->setAttentionValue(*av);
    }
    return handle_to_scm (h);
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

    Atom *atom = NULL;

    // If there was a truth value, change it.
    const TruthValue *tv = get_tv_from_list(satom_list);
    if (tv) {
        if (NULL == atom) atom = TLB::getAtom(h);
        atom->setTruthValue(*tv);
    }

    // If there was an attention value, change it.
    const AttentionValue *av = get_av_from_list(satom_list);
    if (av) {
        if (NULL == atom) atom = TLB::getAtom(h);
        atom->setAttentionValue(*av);
    }
    return handle_to_scm (h);
}

/* ============================================================== */
/**
 * Delete the atom, but only if it has no incoming links.
 * Return SCM_BOOL_T if the atom was deleted, else return SCM_BOOL_F
 */
SCM SchemeSmob::ss_delete (SCM satom)
{
    Handle h = verify_handle(satom, "cog-delete");

    // The remove will fail/log warning if the incoming set isn't null.
    Atom *a = TLB::getAtom(h);
    if (NULL != a->getIncomingSet()) return SCM_BOOL_F;

    AtomSpace *as = CogServer::getAtomSpace();
    // AtomSpace::removeAtom() returns true if atom was deleted, 
    // else returns false
    bool rc = as->removeAtom(h, false);

    // rc should always be true at this point ...
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

#endif
/* ===================== END OF FILE ============================ */
