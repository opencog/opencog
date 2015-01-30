/*
 * opencog/execution/ExecutionOutputLink.cc
 *
 * Copyright (C) 2009, 2013 Linas Vepstas
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
#include "ExecutionOutputLink.h"

using namespace opencog;

ExecutionOutputLink::ExecutionOutputLink(const HandleSeq& oset,
                                         TruthValuePtr tv,
                                         AttentionValuePtr av)
    : Link(EXECUTION_OUTPUT_LINK, oset, tv, av)
{
    if ((2 != oset.size()) or
       (LIST_LINK != oset[1]->getType()))
    {
        throw RuntimeException(TRACE_INFO, "ExecutionOutputLink must have schema and args!");
    }
}

ExecutionOutputLink::ExecutionOutputLink(Handle schema, Handle args,
                                         TruthValuePtr tv,
                                         AttentionValuePtr av)
    : Link(EXECUTION_OUTPUT_LINK, schema, args, tv, av)
{
    if (LIST_LINK != args->getType()) {
        throw RuntimeException(TRACE_INFO, "ExecutionOutputLink must have schema and args!");
    }
}

/// do_execute -- execute the GroundedSchemaNode of the ExecutionOutputLink
///
/// Expects the argument to be an ExecutionOutputLink, which should have the
/// following structure:
///
///     ExecutionOutputLink
///         GroundedSchemaNode "lang: func_name"
///         ListLink
///             SomeAtom
///             OtherAtom
///
/// The "lang:" should be either "scm:" for scheme, or "py:" for python.
/// This method will then invoke "func_name" on the provided ListLink
/// of arguments to the function.
///
Handle ExecutionOutputLink::do_execute(AtomSpace* as, Handle execlnk)
{
    if (EXECUTION_OUTPUT_LINK != execlnk->getType()) {
        throw RuntimeException(TRACE_INFO, "Expecting to get an ExecutionOutputLink!");
    }
    LinkPtr l(LinkCast(execlnk));
    return do_execute(as, l->getOutgoingSet());
}

/// do_execute -- execute the GroundedSchemaNode of the ExecutionOutputLink
///
/// Expects the sequence to be exactly two atoms long.
/// Expects the first handle of the sequence to be a GroundedSchemaNode
/// Expects the second handle of the sequence to be a ListLink
/// Executes the GroundedSchemaNode, supplying the second handle as argument
///
Handle ExecutionOutputLink::do_execute(AtomSpace* as, const HandleSeq& sna)
{
    if (2 != sna.size())
    {
        throw RuntimeException(TRACE_INFO, "Incorrect arity for an ExecutionOutputLink!");
    }
    return do_execute(as, sna[0], sna[1]);
}

/// do_execute -- execute the GroundedSchemaNode of the ExecutionOutputLink
///
/// Expects "gsn" to be a GroundedSchemaNode
/// Expects "args" to be a ListLink
/// Executes the GroundedSchemaNode, supplying the args as argument
///
Handle ExecutionOutputLink::do_execute(AtomSpace* as, Handle gsn, Handle args)
{
    if (GROUNDED_SCHEMA_NODE != gsn->getType()) {
        throw RuntimeException(TRACE_INFO, "Expecting GroundedSchemaNode!");
    }
    if (LIST_LINK != args->getType())
    {
        throw RuntimeException(TRACE_INFO,
            "Expecting arguments to ExecutionOutputLink!");
    }

    // Get the schema name.
    const std::string& schema = NodeCast(gsn)->getName();
    // printf ("Grounded schema name: %s\n", schema.c_str());

    // At this point, we only run scheme and python schemas.
    if (0 == schema.compare(0,4,"scm:", 4))
    {
#ifdef HAVE_GUILE
        // Be friendly, and strip leading white-space, if any.
        size_t pos = 4;
        while (' ' == schema[pos]) pos++;

        SchemeEval* applier = get_evaluator(as);
        return applier->apply(schema.substr(pos), args);
#else
        throw RuntimeException(TRACE_INFO,
            "Cannot evaluate scheme GroundedSchemaNode!");
#endif /* HAVE_GUILE */
    }

    if (0 == schema.compare(0, 3,"py:", 3))
    {
#ifdef HAVE_CYTHON
        // Be friendly, and strip leading white-space, if any.
        size_t pos = 3;
        while (' ' == schema[pos]) pos++;

        PythonEval &applier = PythonEval::instance();

        Handle h = applier.apply(schema.substr(pos), args);

        // Return the handle
        return h;
#else
        throw RuntimeException(TRACE_INFO,
            "Cannot evaluate python GroundedSchemaNode!");
#endif /* HAVE_CYTHON */
    }

    // Unkown proceedure type.
    throw RuntimeException(TRACE_INFO,
        "Cannot evaluate unknown GroundedSchemaNode!");
}

