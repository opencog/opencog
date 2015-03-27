/*
 * embodiment/AtomSpaceExtensions/GetByOutgoing.cc
 *
 * Copyright (C) 2008-2011 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
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

#include <iterator>
#include <set>
#include <stdlib.h>
#include <string>
#include <unordered_set>

#include <boost/bind.hpp>

#include <opencog/util/Logger.h>
#include <opencog/util/functional.h>
#include "GetByOutgoing.h"
#include "Intersect.h"

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

// ====================================================================
//

UnorderedHandleSet getHandlesByOutgoingSet(
                                     AtomSpace& as,
                                     Type type,
                                     const HandleSeq& handles,
                                     Type* types,
                                     bool* subclasses,
                                     Arity arity,
                                     bool subclass)
{
    // Check if it is the special case of looking for an specific atom
    if (classserver().isA(type, LINK) and
        (arity == 0 || !handles.empty()))
    {
        DPRINTF("special case arity=%d\n", arity);
        bool hasAllHandles = true;
        for (Arity i = 0; hasAllHandles && i < arity; i++) {
            hasAllHandles = as.isValidHandle(handles[i]);
        }
        DPRINTF("hasAllHandles = %d, subclass = %d\n", hasAllHandles, subclass);
        if (hasAllHandles && !subclass) {
            DPRINTF("building link for lookup: type = %d, handles.size() = %zu\n", type, handles.size());
            Handle h(as.getHandle(type, handles));

            UnorderedHandleSet result;
            if (as.isValidHandle(h)) {
                result.insert(h);
            }
            DPRINTF("Returning HandleSet by using atom hash_set!\n");
            return result;
        }
    }

    if (classserver().isA(type, LINK) && (arity == 0)) {
        UnorderedHandleSet uhs;
        as.getHandlesByType(inserter(uhs), type, subclass);

        UnorderedHandleSet result;
        std::copy_if(uhs.begin(), uhs.end(), inserter(result),
            // result = HandleEntry::filterSet(result, arity);
            [&](Handle h)->bool {
                LinkPtr l(LinkCast(h));
                // If a Node, then accept it.
                if (NULL == l) return true;
                return (0 == l->getArity());
        });
        return result;
    }

    std::vector<UnorderedHandleSet> sets(arity);

    int countdown = 0;

    // builds a set for each element in the outgoing set. Empty sets are
    // counted to be removed a posteriori
    for (Arity i = 0; i < arity; i++) {
        if ((!handles.empty()) && as.isValidHandle(handles[i])) {
            Handle h(handles[i]);
            HandleSeq hs;
            h->getIncomingSet(back_inserter(hs));

            std::copy_if(hs.begin(), hs.end(), inserter(sets[i]),
                // sets[i] = HandleEntry::filterSet(sets[i], handles[i], i, arity);
                [&](Handle h)->bool {
                    LinkPtr l(LinkCast(h));
                    // If a Node, then accept it.
                    if (NULL == l) return true;
                    return (l->getArity() == arity) and
                           (handles[i] == l->getOutgoingSet()[i]);
                });

            if (sets[i].size() == 0)
                return UnorderedHandleSet();

        } else if ((types != NULL) && (types[i] != NOTYPE)) {
            bool sub = subclasses == NULL ? false : subclasses[i];
            as.getHandlesByTargetType(inserter(sets[i]), type, types[i], subclass, sub);
            if (sets[i].size() == 0)
                return UnorderedHandleSet();
        } else {
            countdown++;
        }
    }

    int newLength = arity;
    // if the empty set counter is not zero, removes them by shrinking the
    // list of sets
    if (countdown > 0) {
        DPRINTF("newset allocated size = %d\n", (arity - countdown));
        // TODO: Perhaps it's better to simply erase the NULL entries of the sets
        std::vector<UnorderedHandleSet> newset;
        for (int i = 0; i < arity; i++) {
            if (sets[i].size() != 0)
                newset.push_back(sets[i]);
        }
        sets = newset;
    }
    DPRINTF("newLength = %d\n", newLength);

    if ((type != ATOM) || (!subclass)) {
        for (int i = 0; i < newLength; i++) {
            // filters by type and subclass in order to remove unwanted elements.
            // This is done before the intersection method to reduce the number of
            // elements being passed (intersection uses qsort, which is n log n)
            // sets[i] = HandleEntry::filterSet(sets[i], type, subclass);
            UnorderedHandleSet hs;
            std::copy_if(sets[i].begin(), sets[i].end(), inserter(hs),
                [&](Handle h)->bool { return h->isType(type, subclass); });
        }
    }

    // computes the intersection of all non-empty sets
    UnorderedHandleSet set = intersection(sets);
    // TODO: Why not move this filtering to the begining...
    // Pehaps it will filter more before the intersection
    // (which seems to be the most expensive operation)
    // filters the answer set for every type in the array of target types
    if (types != NULL) {
        for (int i = 0; i < arity; i++) {
            if (types[i] != NOTYPE) {
                bool sub = subclasses == NULL ? false : subclasses[i];
                // set = HandleEntry::filterSet(set, types[i], sub, i, arity);
                UnorderedHandleSet filt;
                std::copy_if(set.begin(), set.end(), inserter(filt),
                    [&](Handle h)->bool {
                        LinkPtr l(LinkCast(h));
                        // If a Node, then accept it.
                        if (NULL == l) return true;
                        if (l->getArity() != arity) return false;
                        Handle hosi(l->getOutgoingSet()[i]);
                        return hosi->isType(types[i], sub);
                    });
                set = filt;
            }
        }
    }

    return set;
}
