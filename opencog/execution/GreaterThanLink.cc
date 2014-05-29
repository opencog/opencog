/*
 * opencog/atomspace/GreaterThanLink.cc
 *
 * Copyright (C) 2009, 2013, 2014 Linas Vepstas
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/cython/PythonEval.h>
#include <opencog/guile/SchemeEval.h>
#include "GreaterThanLink.h"

using namespace opencog;

GreaterThanLink::GreaterThanLink(const HandleSeq& oset,
                             TruthValuePtr tv,
                             AttentionValuePtr av)
    : Link(EXECUTION_LINK, oset, tv, av)
{
    if ((2 != oset.size()) or
       (LIST_LINK != oset[1]->getType()))
    {
        throw RuntimeException(TRACE_INFO, "GreaterThanLink must have schema and args!");
    }
}

GreaterThanLink::GreaterThanLink(Handle schema, Handle args,
                             TruthValuePtr tv,
                             AttentionValuePtr av)
    : Link(EXECUTION_LINK, schema, args, tv, av)
{
    if (LIST_LINK != args->getType()) {
        throw RuntimeException(TRACE_INFO, "GreaterThanLink must have schema and args!");
    }
}

/// do_execute -- execute the GroundedSchemaNode of the GreaterThanLink
///
/// Expects the argument to be an GreaterThanLink, which should have the
/// following structure:
///
///     GreaterThanLink
///         GroundedSchemaNode "lang: func_name"
///         ListLink
///             SomeAtom
///             OtherAtom
///
/// The "lang:" should be either "scm:" for scheme, or "py:" for python.
/// This method will then invoke "func_name" on the provided ListLink
/// of arguments to the function.
///
bool GreaterThanLink::do_execute(AtomSpace* as, Handle execlnk)
{
    if (EXECUTION_LINK != execlnk->getType()) {
        throw RuntimeException(TRACE_INFO, "Expecting to get an GreaterThanLink!");
    }
    LinkPtr l(LinkCast(execlnk));
    return do_execute(as, l->getOutgoingSet());
}

/// do_execute -- execute the GroundedSchemaNode of the GreaterThanLink
///
/// Expects the sequence to be exactly two atoms long.
/// Expects the first handle of the sequence to be a GroundedSchemaNode
/// Expects the second handle of the sequence to be a ListLink
/// Executes the GroundedSchemaNode, supplying the second handle as argument
///
bool GreaterThanLink::do_execute(AtomSpace* as, const HandleSeq& sna)
{
    if (2 != sna.size())
    {
        throw RuntimeException(TRACE_INFO, "Incorrect arity for an GreaterThanLink!");
    }
    return do_execute(as, sna[0], sna[1]);
}

/// do_execute -- execute the GroundedSchemaNode of the GreaterThanLink
///
/// Expects "gsn" to be a GroundedSchemaNode
/// Expects "args" to be a ListLink
/// Executes the GroundedSchemaNode, supplying the args as argument
///
bool GreaterThanLink::do_execute(AtomSpace* as, Handle gsn, Handle args)
{
    if (GROUNDED_SCHEMA_NODE != gsn->getType()) {
        throw RuntimeException(TRACE_INFO, "Expecting GroundedSchemaNode!");
    }
    if (LIST_LINK != args->getType())
    {
        throw RuntimeException(TRACE_INFO, "Expecting arguments to GreaterThanLink!");
    }

    // Get the schema name.
    const std::string& schema = NodeCast(gsn)->getName();
    // printf ("Grounded schema name: %s\n", schema.c_str());

    // A very special-case C++ comparison
    // This can compare two NumberNodes 
    // Hard-coded in C++ for speed. (well, and convenience ...)
    if (0 == schema.compare(0,4,"c++:greater", 11))
    {
        LinkPtr ll(LinkCast(args));
        Handle h1(ll->getOutgoingAtom(0));
        Handle h2(ll->getOutgoingAtom(1));
        if (NUMBER_NODE != h1->getType() or NUMBER_NODE != h2->getType())
            throw RuntimeException(TRACE_INFO,
                "Expecting c++:greater arguments to be NumberNode's!");
        const std::string& s1 = NodeCast(h1)->getName();
        const std::string& s2 = NodeCast(h2)->getName();
        double v1 = atof(s1.c_str());
        double v2 = atof(s2.c_str());
        return v1 > v2;
    }

    // At this point, we only run scheme and python schemas.
    if (0 == schema.compare(0,4,"scm:", 4))
    {
#ifdef HAVE_GUILE
        // Be friendly, and strip leading white-space, if any.
        size_t pos = 4;
        while (' ' == schema[pos]) pos++;

        SchemeEval* applier = new SchemeEval(as);
        std::string rc = applier->apply_generic(schema.substr(pos), args);
        delete applier;

        // If its false or nil, then false; everything else is true.
        if (rc.compare("#f") or rc.compare("()")) return false;
        return true;
#else
        throw RuntimeException(TRACE_INFO, "Cannot evaluate scheme GroundedSchemaNode!");
#endif /* HAVE_GUILE */
    }

    if (0 == schema.compare(0, 3,"py:", 3))
    {
#ifdef HAVE_CYTHON
        // Be friendly, and strip leading white-space, if any.
        // size_t pos = 3;
        // while (' ' == schema[pos]) pos++;

        // PythonEval &applier = PythonEval::instance();
        // ???? applier.apply(schema.substr(pos), args);
        throw RuntimeException(TRACE_INFO, "Python support not implemented!");

        return false;
#else
        throw RuntimeException(TRACE_INFO, "Cannot evaluate python GroundedSchemaNode!");
#endif /* HAVE_CYTHON */
    }

    // Unkown proceedure type.
    throw RuntimeException(TRACE_INFO, "Cannot evaluate unknown GroundedSchemaNode!");
}

