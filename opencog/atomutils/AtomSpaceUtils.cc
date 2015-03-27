/*
 * AtomSpaceUtils.cc
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

#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/Link.h>
#include "AtomSpaceUtils.h"


using namespace opencog;


/**
 * Get all the nodes within a link and its sublinks.
 *
 * @param h     the top level link
 * @return      a HandleSeq of nodes
 */
HandleSeq AtomSpaceUtils::getAllNodes(Handle h)
{
    HandleSeq results;

    LinkPtr lll(LinkCast(h));
    if (lll)
        for (const Handle& o : lll->getOutgoingSet())
        {
            HandleSeq sub = getAllNodes(o);
            results.insert(results.end(), sub.begin(), sub.end());
        }
    else
        results.push_back(h);

    // Handle is copy safe, but in this case C++11 would move it
    return results;
}

/**
 * Get all unique nodes within a link and its sublinks.
 *
 * Similar to getAllNodes except there will be no repetition.
 *
 * @param h     the top level link
 * @return      a UnorderedHandleSet of nodes
 */
UnorderedHandleSet AtomSpaceUtils::getAllUniqueNodes(Handle h)
{
    UnorderedHandleSet results;

    LinkPtr lll(LinkCast(h));
    if (lll)
        for (const Handle& o : lll->getOutgoingSet())
        {
            UnorderedHandleSet sub = getAllUniqueNodes(o);
            results.insert(sub.begin(), sub.end());
        }
    else
        results.insert(h);

    return results;
}

namespace opencog
{

HandleSeq getNeighbors(const Handle& h, bool fanin,
                       bool fanout, Type desiredLinkType,
                       bool subClasses)
{
    if (h == NULL) {
        throw InvalidParamException(TRACE_INFO,
            "Handle %d doesn't refer to a Atom", h.value());
    }
    HandleSeq answer;

    for (const LinkPtr& link : h->getIncomingSet())
    {
        Type linkType = link->getType();
        if ((linkType == desiredLinkType)
            or (subClasses && classserver().isA(linkType, desiredLinkType))) {
            for (const Handle& handle : link->getOutgoingSet()) {
                if (handle == h) continue;
                if (!fanout && link->isSource(h)) continue;
                if (!fanin && link->isTarget(h)) continue;
                answer.push_back(handle);
            }
        }
    }
    return answer;
}

}
