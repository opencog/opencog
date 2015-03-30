/*
 * AtomUtils.cc
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

#include <opencog/atomspace/Link.h>
#include "AtomUtils.h"

namespace opencog
{

/**
 * Get all the nodes within a link and its sublinks.
 *
 * @param h     the top level link
 * @return      a HandleSeq of nodes
 */
HandleSeq getAllNodes(Handle h)
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
UnorderedHandleSet getAllUniqueNodes(Handle h)
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

/* Tail-recursive helper function. We mark it static, so that
 * gcc can optimize this, i.e. call it without buying the stack
 * frame. */
static void get_distant_neighbors_rec(const Handle& h,
                                      UnorderedHandleSet& res,
                                      unsigned dist)
{
	res.insert(h);

	// Recursive calls
	if (dist > 0) {
        // 1. Fetch incomings
        for (const LinkPtr& in_l : h->getIncomingSet()) {
            Handle in_h = in_l->getHandle();
            if (res.find(in_h) == res.cend()) // Do not re-explore
                get_distant_neighbors_rec(in_h, res, dist - 1);
        }
        // 2. Fetch outgoings
        LinkPtr link = LinkCast(h);
        if (link) {
	        for (const Handle& out_h : link->getOutgoingSet()) {
		        if (res.find(out_h) == res.cend()) // Do not re-explore
			        get_distant_neighbors_rec(out_h, res, dist - 1);
	        }
        }
    }
}

UnorderedHandleSet get_distant_neighbors(const Handle& h, unsigned dist)
{
    UnorderedHandleSet results;
    get_distant_neighbors_rec(h, results, dist);
    results.erase(h);
    return results;
}

}
