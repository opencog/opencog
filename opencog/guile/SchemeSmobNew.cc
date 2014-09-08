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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

/* ============================================================== */
/**
 * Return a string holding the scheme representation of an atom/truthvalue.
 *
 * The input is assumed to be pointing at a Handle, a TruthValue,
 * an AttentionValue or a UUID. Returned is a valid scheme
 * expression that represents that Handle, etc.
 */
std::string SchemeSmob::to_string(SCM node)
{
    if (SCM_SMOB_PREDICATE(SchemeSmob::cog_uuid_tag, node))
        return uuid_to_string(node);

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
    if (NULL == h) return "#<Invalid handle>";

    // Print a scheme expression, so that the output can be saved
    // to file, and then restored, as needed.
    std::string ret = "";
    for (int i=0; i< indent; i++) ret += "   ";
    ret += "(";
    ret += classserver().getTypeName(h->getType());
    NodePtr nnn(NodeCast(h));
    LinkPtr lll(LinkCast(h));
    if (nnn) {
        ret += " \"";
        ret += nnn->getName();
        ret += "\"";

        // Print the truth value only after the node name
        TruthValuePtr tv(h->getTruthValue());
        if (!tv->isDefaultTV()) {
            ret += " ";
            ret += tv_to_string (tv.get());
        }

        // Print the attention value after the truth value
        AttentionValuePtr av(h->getAttentionValue());
        if (av != AttentionValue::DEFAULT_AV()) {
            ret += " ";
            ret += av_to_string (av.get());
        }
        ret += ")";
        return ret;
    }
    else if (lll) {
        // If there's a truth value, print it before the other atoms
        TruthValuePtr tv(h->getTruthValue());
        if (!tv->isDefaultTV()) {
            ret += " ";
            ret += tv_to_string (tv.get());
        }

        // Print the attention value after the truth value
        AttentionValuePtr av(h->getAttentionValue());
        if (av != AttentionValue::DEFAULT_AV()) {
            ret += " ";
            ret += av_to_string (av.get());
        }

        // print the outgoing link set.
        ret += "\n";
        std::vector<Handle> oset = lll->getOutgoingSet();
        unsigned int arity = oset.size();
        for (unsigned int i=0; i<arity; i++) {
            //ret += " ";
            ret += handle_to_string(oset[i], /*(0==i)?0:*/indent+1);
            ret += "\n";
            //if (i != arity-1) ret += "\n";
        }
        for (int i=0; i < indent; i++) ret += "   ";
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

std::string SchemeSmob::uuid_to_string(SCM node)
{
    Handle h = scm_uuid_to_handle(node);
    return handle_to_string(h, 0) + "\n";
}

/* ============================================================== */
/**
 * Wrapper -- given a handle, return the corresponding scheme object.
 * Note that smobs hold the integer value of the handle, and not the
 * C++ handle object itself.
 */
SCM SchemeSmob::uuid_to_scm (UUID uuid)
{
    SCM suuid = scm_from_ulong(uuid);
    SCM_RETURN_NEWSMOB (cog_uuid_tag, suuid);
}

SCM SchemeSmob::handle_to_scm (Handle h)
{
    Handle* hp = new Handle(h); // so that the smart pointer increments!
    // Force resolution to occur now, not later.
    hp->operator->();
    scm_gc_register_collectable_memory (hp,
                    sizeof(*hp), "opencog handle");

    SCM smob;
    SCM_NEWSMOB (smob, cog_misc_tag, hp);
    SCM_SET_SMOB_FLAGS(smob, COG_HANDLE);
    return smob;
}

Handle SchemeSmob::scm_uuid_to_handle (SCM sh)
{
    if (not SCM_SMOB_PREDICATE(SchemeSmob::cog_uuid_tag, sh))
        return Handle::UNDEFINED;

    SCM suuid = SCM_SMOB_OBJECT(sh);
    UUID uuid = scm_to_ulong(suuid);
    return Handle(uuid);
}

Handle SchemeSmob::scm_to_handle (SCM sh)
{
    if (SCM_SMOB_PREDICATE(SchemeSmob::cog_uuid_tag, sh))
    {
        SCM suuid = SCM_SMOB_OBJECT(sh);
        UUID uuid = scm_to_ulong(suuid);
        return Handle(uuid);
    }

    if (not SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, sh))
        return Handle::UNDEFINED;

    scm_t_bits misctype = SCM_SMOB_FLAGS(sh);
    if (COG_HANDLE != misctype)
        return Handle::UNDEFINED;

    Handle* hp = (Handle *) SCM_SMOB_DATA(sh);
    return *hp;
}

/* ============================================================== */
/**
 * Create a new scheme object, holding the atom uuid
 */
SCM SchemeSmob::ss_atom (SCM suuid)
{
    if (scm_is_false(scm_integer_p(suuid)))
        scm_wrong_type_arg_msg("cog-atom", 1, suuid, "integer opencog uuid");
    SCM_RETURN_NEWSMOB (cog_uuid_tag, suuid);
}

/* ============================================================== */
/**
 * Return uuid of atom
 */
SCM SchemeSmob::ss_handle (SCM satom)
{
    // If its the uuid, already, just return that.
    if (SCM_SMOB_PREDICATE(SchemeSmob::cog_uuid_tag, satom))
        return SCM_SMOB_OBJECT(satom);

    Handle h(scm_to_handle(satom));
    if (Handle::UNDEFINED == h)
        scm_wrong_type_arg_msg("cog-handle", 1, satom, "opencog atom");

    return scm_from_ulong(h.value());
}

/* ============================================================== */
/**
 * Return Handle::UNDEFINED
 */
SCM SchemeSmob::ss_undefined_handle (void)
{
    return scm_from_ulong(Handle::UNDEFINED.value());
}

/* ============================================================== */
/** Return true if s is an atom */

SCM SchemeSmob::ss_atom_p (SCM s)
{
    if (SCM_SMOB_PREDICATE(SchemeSmob::cog_uuid_tag, s))
        return SCM_BOOL_T;
    if (not SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, s))
        return SCM_BOOL_F;

    scm_t_bits misctype = SCM_SMOB_FLAGS(s);
    if (COG_HANDLE == misctype)
        return SCM_BOOL_T;

    return SCM_BOOL_F;
}

/* ============================================================== */
/** Return true if s is a node */

SCM SchemeSmob::ss_node_p (SCM s)
{
    Handle h(scm_to_handle(s));
    if (Handle::UNDEFINED == h)
        return SCM_BOOL_F;

    if (NodeCast(h)) return SCM_BOOL_T;

    return SCM_BOOL_F;
}

/* ============================================================== */
/** Return true if s is a link */

SCM SchemeSmob::ss_link_p (SCM s)
{
    Handle h(scm_to_handle(s));
    if (Handle::UNDEFINED == h)
        return SCM_BOOL_F;

    if (LinkCast(h)) return SCM_BOOL_T;
    return SCM_BOOL_F;
}

/* ============================================================== */
/**
 * Check that the argument is the string or symbol name of an atom,
 * else throw errors.
 * Return the atom type.
 */
Type SchemeSmob::verify_atom_type (SCM stype, const char *subrname, int pos)
{
    if (scm_is_true(scm_symbol_p(stype)))
        stype = scm_symbol_to_string(stype);

    if (scm_is_false(scm_string_p(stype)))
        scm_wrong_type_arg_msg(subrname, pos, stype, "name of opencog atom type");

    char * ct = scm_to_locale_string(stype);
    Type t = classserver().getType(ct);
    free(ct);

    // Make sure that the type is good
    if (NOTYPE == t)
        scm_wrong_type_arg_msg(subrname, pos, stype, "name of opencog atom type");

    return t;
}

/**
 * Check that the argument is the string or symbol name of a node,
 * else throw errors.
 * Return the node type.
 */
Type SchemeSmob::verify_node_type (SCM stype, const char *subrname, int pos)
{
    Type t = verify_atom_type(stype, subrname, pos);

    if (false == classserver().isA(t, NODE))
        scm_wrong_type_arg_msg(subrname, pos, stype, "name of opencog node type");

    return t;
}

/**
 * Check that the argument is a string, else throw errors.
 * Return the string, in C.
 */
std::string SchemeSmob::verify_string (SCM sname, const char *subrname,
                                       int pos, const char * msg)
{
    if (scm_is_false(scm_string_p(sname)))
        scm_wrong_type_arg_msg(subrname, pos, sname, msg);

    char * cname = scm_to_locale_string(sname);
    std::string name = cname;
    free(cname);
    return name;
}

/**
 * Check that the argument is an int, else throw errors.
 * Return the int.
 */
int SchemeSmob::verify_int (SCM sint, const char *subrname,
                            int pos, const char * msg)
{
    if (scm_is_false(scm_integer_p(sint)))
        scm_wrong_type_arg_msg(subrname, pos, sint, msg);

    return scm_to_int(sint);
}

/**
 * Create a new node, of named type stype, and string name sname
 */
SCM SchemeSmob::ss_new_node (SCM stype, SCM sname, SCM kv_pairs)
{
    Type t = verify_node_type(stype, "cog-new-node", 1);

    // Special case handling for NumberNode
    if (NUMBER_NODE == t and scm_is_number(sname)) {
        sname = scm_number_to_string(sname, _radix_ten);
        // TODO: if we're given a string, I guess maybe we should check
        // that the string is convertible to a number ??
    }
    std::string name = verify_string (sname, "cog-new-node", 2,
        "string name for the node");

    AtomSpace* atomspace = get_as_from_list(kv_pairs);
    if (NULL == atomspace) atomspace = ss_get_env_as("cog-new-node");

    Handle h;
    // Now, create the actual node... in the actual atom space.
    // tv->clone is called here, because, for the atomspace, we want
    // to use a use-counted std:shared_ptr, whereas in guile, we are
    // using a garbage-collected raw pointer.  So clone makes up the
    // difference.
    const TruthValue *tv = get_tv_from_list(kv_pairs);
    if (tv)
        h = atomspace->addNode(t, name, tv->clone());
    else
        h = atomspace->addNode(t, name);

    // Was an attention value explicitly specified?
    // If so, then we've got to set it.
    AttentionValue *av = get_av_from_list(kv_pairs);
    if (av) {
        h->setAttentionValue(av->clone());
    }

    return handle_to_scm(h);
}

/**
 * Return the indicated node, of named type stype, and string name sname
 * if it exists; else return nil if it does not exist.
 * If the node exists, *and* a truth value was specified, then change
 * the truth value.
 */
SCM SchemeSmob::ss_node (SCM stype, SCM sname, SCM kv_pairs)
{
    Type t = verify_node_type(stype, "cog-node", 1);
    std::string name = verify_string (sname, "cog-node", 2,
                                    "string name for the node");

    AtomSpace* atomspace = get_as_from_list(kv_pairs);
    if (NULL == atomspace) atomspace = ss_get_env_as("cog-node");

    // Now, look for the actual node... in the actual atom space.
    Handle h(atomspace->getHandle(t, name));
    if (Handle::UNDEFINED == h) return SCM_EOL;
    if (NULL == h) return SCM_EOL;

    // If there was a truth value, change it.
    const TruthValue *tv = get_tv_from_list(kv_pairs);
    if (tv) {
        h->setTruthValue(tv->clone());
    }

    // If there was an attention value, change it.
    const AttentionValue *av = get_av_from_list(kv_pairs);
    if (av) {
        h->setAttentionValue(av->clone());
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
SchemeSmob::verify_handle_list (SCM satom_list, const char * subrname, int pos)
{
    // Verify that second arg is an actual list. Allow null list
    // (which is rather unusal, but legit.  Allow embedded nulls
    // as this can be convenient for writing scheme code.
    if (!scm_is_pair(satom_list) and !scm_is_null(satom_list))
        scm_wrong_type_arg_msg(subrname, pos, satom_list, "a list of atoms");

    std::vector<Handle> outgoing_set;
    SCM sl = satom_list;
    pos = 2;
    while (scm_is_pair(sl)) {
        SCM satom = SCM_CAR(sl);

        // Verify that the contents of the list are actual atoms.
        Handle h(scm_to_handle(satom));
        if (Handle::UNDEFINED != h) {
            outgoing_set.push_back(h);
        }
        else if (scm_is_pair(satom) and !scm_is_null(satom_list)) {
            // Allow lists to be specified: e.g.
            // (cog-new-link 'ListLink (list x y z))
            // Do this via a recursive call, flattening nested lists
            // as we go along.
            const std::vector<Handle> &oset =
                verify_handle_list(satom, subrname, pos);
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
				if (not SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, satom)) {
                // If its not an atom, and its not a truth value, and
                // its not an attention value, and its not an atomspace,
                // then whatever it is, its bad.
                scm_wrong_type_arg_msg(subrname, pos, satom, "opencog atom");
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
    Handle h;
    Type t = verify_link (stype, "cog-new-link");

    std::vector<Handle> outgoing_set;
    outgoing_set = verify_handle_list (satom_list, "cog-new-link", 2);

    AtomSpace* atomspace = get_as_from_list(satom_list);
    if (NULL == atomspace) atomspace = ss_get_env_as("cog-new-link");

    // Fish out a truth value, if its there.
    const TruthValue *tv = get_tv_from_list(satom_list);
    if (tv) {
        h = atomspace->addLink(t, outgoing_set, tv->clone());
    } else {
        // Now, create the actual link... in the actual atom space.
        h = atomspace->addLink(t, outgoing_set);
    }

    // Was an attention value explicitly specified?
    // If so, then we've got to set it.
    const AttentionValue *av = get_av_from_list(satom_list);
    if (av) {
        h->setAttentionValue(av->clone());
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
    outgoing_set = verify_handle_list (satom_list, "cog-link", 2);

    AtomSpace* atomspace = get_as_from_list(satom_list);
    if (NULL == atomspace) atomspace = ss_get_env_as("cog-link");

    // Now, look to find the actual link... in the actual atom space.
    Handle h(atomspace->getHandle(t, outgoing_set));
    if (Handle::UNDEFINED == h) return SCM_EOL;
    if (NULL == h) return SCM_EOL;

    // If there was a truth value, change it.
    const TruthValue *tv = get_tv_from_list(satom_list);
    if (tv) h->setTruthValue(tv->clone());

    // If there was an attention value, change it.
    const AttentionValue *av = get_av_from_list(satom_list);
    if (av) h->setAttentionValue(av->clone());

    return handle_to_scm (h);
}

/* ============================================================== */
/**
 * Delete the atom, but only if it has no incoming links.
 * Return SCM_BOOL_T if the atom was deleted, else return SCM_BOOL_F
 * This deletes the atom from both the atomspace, and any attached
 * backing store; thus deletion is permanent!
 */
SCM SchemeSmob::ss_delete (SCM satom, SCM kv_pairs)
{
    Handle h = verify_handle(satom, "cog-delete");

    // It can happen that the atom has already been deleted, but we're
    // still holding on to its UUID.  This is rare... but possible. So
    // don't crash when it happens.
    if (NULL == h.operator->()) return SCM_BOOL_F;

    // The remove will fail/log warning if the incoming set isn't null.
    if (h->getIncomingSetSize() > 0) return SCM_BOOL_F;

    AtomSpace* atomspace = get_as_from_list(kv_pairs);
    if (NULL == atomspace) atomspace = ss_get_env_as("cog-delete");

    // AtomSpace::removeAtom() returns true if atom was deleted,
    // else returns false
    bool rc = atomspace->removeAtom(h, false);

    // rc should always be true at this point ...
    if (rc) return SCM_BOOL_T;
    return SCM_BOOL_F;
}

/* ============================================================== */
/**
 * Delete the atom, and everything pointing to it.
 * This deletes the atom from both the atomspace, and any attached
 * backing store; thus deletion is permanent!
 */
SCM SchemeSmob::ss_delete_recursive (SCM satom, SCM kv_pairs)
{
    Handle h = verify_handle(satom, "cog-delete-recursive");

    AtomSpace* atomspace = get_as_from_list(kv_pairs);
    if (NULL == atomspace) atomspace = ss_get_env_as("cog-delete-recursive");

    bool rc = atomspace->removeAtom(h, true);

    if (rc) return SCM_BOOL_T;
    return SCM_BOOL_F;
}

/* ============================================================== */
/**
 * Purgee the atom from the atomspace, but only if it has no incoming links.
 * Return SCM_BOOL_T if the atom was deleted, else return SCM_BOOL_F
 * This does NOT remove the atom from any attached backing store, only
 * from the atomspace.
 */
SCM SchemeSmob::ss_purge (SCM satom, SCM kv_pairs)
{
    Handle h = verify_handle(satom, "cog-purge");

    // It can happen that the atom has already been purged, but we're
    // still holding on to its UUID.  This is rare... but possible. So
    // don't crash when it happens.
    if (NULL == h.operator->()) return SCM_BOOL_F;

    // The purge will fail/log warning if the incoming set isn't null.
    if (h->getIncomingSetSize() > 0) return SCM_BOOL_F;

    AtomSpace* atomspace = get_as_from_list(kv_pairs);
    if (NULL == atomspace) atomspace = ss_get_env_as("cog-purge");

    // AtomSpace::purgeAtom() returns true if atom was purged,
    // else returns false
    bool rc = atomspace->purgeAtom(h, false);

    // rc should always be true at this point ...
    if (rc) return SCM_BOOL_T;
    return SCM_BOOL_F;
}

/* ============================================================== */
/**
 * Purge the atom, and everything pointing to it.
 * This does NOT remove the atom from any attached backing store, only
 * from the atomspace.
 */
SCM SchemeSmob::ss_purge_recursive (SCM satom, SCM kv_pairs)
{
    Handle h = verify_handle(satom, "cog-purge-recursive");

    AtomSpace* atomspace = get_as_from_list(kv_pairs);
    if (NULL == atomspace) atomspace = ss_get_env_as("cog-purge-recursive");

    bool rc = atomspace->purgeAtom(h, true);

    if (rc) return SCM_BOOL_T;
    return SCM_BOOL_F;
}

#endif
/* ===================== END OF FILE ============================ */
