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

#include "AtomSpaceUtils.h"


using namespace opencog;


/**
 * Get all the nodes within a link and its sublinks.
 *
 * @param pAS   the AtomSpace to look into
 * @param h     the top level link
 * @return      a HandleSeq of nodes
 */
HandleSeq AtomSpaceUtils::getAllNodes(AtomSpace* pAS, Handle h)
{
    HandleSeq results;

    if (pAS->isLink(h))
    {
        HandleSeq oset = pAS->getOutgoing(h);

        for (Handle o : oset)
        {
            HandleSeq sub = getAllNodes(pAS, o);
            results.insert(results.end(), sub.begin(), sub.end());
        }
    }
    else if (pAS->isNode(h))
    {
        results.push_back(h);
    }

    // Handle is copy safe, but in this case C++11 would move it
    return results;
}

/**
 * Get all unique nodes within a link and its sublinks.
 *
 * Similar to getAllNodes except there will be no repetition.
 *
 * @param pAS   the AtomSpace to look into
 * @param h     the top level link
 * @return      a UnorderedHandleSet of nodes
 */
UnorderedHandleSet AtomSpaceUtils::getAllUniqueNodes(AtomSpace* pAS, Handle h)
{
    UnorderedHandleSet results;

    if (pAS->isLink(h))
    {
        HandleSeq oset = pAS->getOutgoing(h);

        for (Handle o : oset)
        {
            UnorderedHandleSet sub = getAllUniqueNodes(pAS, o);
            results.insert(sub.begin(), sub.end());
        }
    }
    else if (pAS->isNode(h))
    {
        results.insert(h);
    }

    return results;
}

