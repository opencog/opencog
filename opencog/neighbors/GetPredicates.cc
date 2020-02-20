/*
 * GetPredicates.cc
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: William Ma <https://github.com/williampma>
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

#include <iostream>

#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include "GetPredicates.h"

namespace opencog
{

HandleSeq get_predicates(const Handle& target, 
                         Type predicateType,
                         bool subClasses)
{
    if (target == nullptr) {
        throw InvalidParamException(TRACE_INFO,
            "get_predicates: Target handle %d doesn't refer to an Atom", target.value());
    }
    NameServer& nameServer = nameserver();
    HandleSeq answer;

    // First find any ListLinks that point to the target
    for (const Handle& link : target->getIncomingSet())
    {
        // Skip any links that aren't subclasses of ListLink.
        Type linkType = link->get_type();
        if (!nameServer.isA(linkType, LIST_LINK))
           continue;
 
        // Look for EvaluationLink's that contain this ListLink.
        for (const Handle& evaluationLink : link->getIncomingSet())
        {
            // Skip any links that aren't subclasses of EvaluationLink.
            linkType = evaluationLink->get_type();
            if (!nameServer.isA(linkType, EVALUATION_LINK))
                continue;

            // Check the first outgoing atom for this EvaluationLink against
            // the desired predicate type.
            Handle candidatePredicate = evaluationLink->getOutgoingAtom(0);
            Type candidateType = candidatePredicate->get_type();
            if ((candidateType == predicateType)
                or (subClasses &&
                    nameServer.isA(candidateType, predicateType)))
            {
                answer.emplace_back(evaluationLink->get_handle());
            }
        }
    }

    return answer;
}

HandleSeq get_predicates_for(const Handle& target, 
                             const Handle& predicate)
{
    if (target == nullptr) {
        throw InvalidParamException(TRACE_INFO,
            "get_predicates_for: Target handle %d doesn't refer to an Atom", target.value());
    }
    if (predicate == nullptr) {
        throw InvalidParamException(TRACE_INFO,
            "get_predicates_for: Predicate handle %d doesn't refer to an Atom", predicate.value());
    }
    NameServer& nameServer = nameserver();
    HandleSeq answer;

    // First find any ListLinks that point to the target
    for (const Handle& link : target->getIncomingSet())
    {
        // Skip any links that aren't subclasses of ListLink.
        Type linkType = link->get_type();
        if (!nameServer.isA(linkType, LIST_LINK))
           continue;
 
        // Look for EvaluationLink's that contain this ListLink.
        for (const Handle& evaluationLink : link->getIncomingSet())
        {
            // Skip any links that aren't subclasses of EvaluationLink.
            linkType = evaluationLink->get_type();
            if (!nameServer.isA(linkType, EVALUATION_LINK))
                continue;

            // Check if the first outgoing atom for this EvaluationLink is
            // the desired predicate.
            if (predicate == evaluationLink->getOutgoingAtom(0))
                answer.emplace_back(evaluationLink->get_handle());
        }
    }

    return answer;
}

} // namespace OpenCog
