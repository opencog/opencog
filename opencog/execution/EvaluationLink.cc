/*
 * opencog/atomspace/EvaluationLink.cc
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
#include "EvaluationLink.h"

using namespace opencog;

EvaluationLink::EvaluationLink(const HandleSeq& oset,
                             TruthValuePtr tv,
                             AttentionValuePtr av)
    : Link(EVALUATION_LINK, oset, tv, av)
{
    if ((2 != oset.size()) or
       (LIST_LINK != oset[1]->getType()))
    {
        throw RuntimeException(TRACE_INFO,
            "EvaluationLink must have predicate and args!");
    }
}

EvaluationLink::EvaluationLink(Handle schema, Handle args,
                             TruthValuePtr tv,
                             AttentionValuePtr av)
    : Link(EVALUATION_LINK, schema, args, tv, av)
{
    if (LIST_LINK != args->getType()) {
        throw RuntimeException(TRACE_INFO,
            "EvaluationLink must have args in a ListLink!");
    }
}

/// do_evaluate -- evaluate the GroundedPredicateNode of the EvaluationLink
///
/// Expects the argument to be an EvaluationLink, which should have the
/// following structure:
///
///     EvaluationLink
///         GroundedPredicateNode "lang: func_name"
///         ListLink
///             SomeAtom
///             OtherAtom
///
/// The "lang:" should be either "scm:" for scheme, or "py:" for python.
/// This method will then invoke "func_name" on the provided ListLink
/// of arguments to the function.
///
bool EvaluationLink::do_evaluate(AtomSpace* as, Handle execlnk)
{
    if (EVALUATION_LINK != execlnk->getType()) {
        throw RuntimeException(TRACE_INFO, "Expecting to get an EvaluationLink!");
    }
    LinkPtr l(LinkCast(execlnk));
    return do_evaluate(as, l->getOutgoingSet());
}

/// do_evaluate -- evaluate the GroundedPredicateNode of the EvaluationLink
///
/// Expects the sequence to be exactly two atoms long.
/// Expects the first handle of the sequence to be a GroundedPredicateNode
/// Expects the second handle of the sequence to be a ListLink
/// Executes the GroundedPredicateNode, supplying the second handle as argument
///
bool EvaluationLink::do_evaluate(AtomSpace* as, const HandleSeq& sna)
{
    if (2 != sna.size())
    {
        throw RuntimeException(TRACE_INFO, "Incorrect arity for an EvaluationLink!");
    }
    return do_evaluate(as, sna[0], sna[1]);
}

/// do_evaluate -- evaluate the GroundedPredicateNode of the EvaluationLink
///
/// Expects "gsn" to be a GroundedPredicateNode
/// Expects "args" to be a ListLink
/// Executes the GroundedPredicateNode, supplying the args as argument
///
bool EvaluationLink::do_evaluate(AtomSpace* as, Handle gsn, Handle args)
{
    if (GROUNDED_PREDICATE_NODE != gsn->getType()) {
        throw RuntimeException(TRACE_INFO, "Expecting GroundedPredicateNode!");
    }
    if (LIST_LINK != args->getType())
    {
        throw RuntimeException(TRACE_INFO, "Expecting arguments to EvaluationLink!");
    }

    // Get the schema name.
    const std::string& schema = NodeCast(gsn)->getName();
    // printf ("Grounded schema name: %s\n", schema.c_str());

    // A very special-case C++ comparison.
    // This compares two NumberNodes, by their numeric value.
    // Hard-coded in C++ for speed. (well, and for convenience ...)
    if (0 == schema.compare("c++:greater"))
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

    // A very special-case C++ comparison.
    // This compares a set of atoms, verifying that they are all different.
    // Hard-coded in C++ for speed. (well, and for convenience ...)
    if (0 == schema.compare("c++:exclusive"))
    {
        LinkPtr ll(LinkCast(args));
        Arity sz = ll->getArity();
        for (Arity i=0; i<sz-1; i++) {
            Handle h1(ll->getOutgoingAtom(i));
            for (Arity j=i+1; j<sz; j++) {
                Handle h2(ll->getOutgoingAtom(j));
                if (h1 == h2) return false;
            }
        }
        return true;
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
        throw RuntimeException(TRACE_INFO, "Cannot evaluate scheme GroundedPredicateNode!");
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
        throw RuntimeException(TRACE_INFO, "Cannot evaluate python GroundedPredicateNode!");
#endif /* HAVE_CYTHON */
    }

    // Unkown proceedure type.
    throw RuntimeException(TRACE_INFO, "Cannot evaluate unknown GroundedPredicateNode!");
}

