/*
 * opencog/atomspace/Intersect.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2013 Linas Vepstas
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Welter Silva <welter@vettalabs.com>
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

#include "Intersect.h"
#include <opencog/util/exceptions.h>

using namespace opencog;


static int nextMatch(Handle** sets, int* sizes, std::vector<int>& cursors)
{
    int n = cursors.size();
    for (;;) {

        bool fail = false;
        int cursor = cursors[0];
        if (cursor >= sizes[0]) return -1; // prevents invalid data read
        Handle* handles = sets[0];
        Handle max = handles[cursor];
        // Handle max(0); // this is zero, not null (the smallest possible pointer)
        // if current positions stored in cursors is not a match, the largest
        // value pointed by cursors is computed to reposition cursors
        // accordingly
        for (int j = 0; j < n; j++) {
            // overflow in one of the arrays: no more matches
            if (cursors[j] >= sizes[j]) {
                return -1;
            }
            if (sets[j][cursors[j]] >  max) {
                max = sets[j][cursors[j]];
            }
            if (sets[0][cursors[0]] != sets[j][cursors[j]]) {
                fail = true;
                // can't break here in order to compute max
            }
        }


        if (fail) {
            // current position is not a match, so cursors are updated to point
            // to the next element in each array which is greater than or
            // equals to max
            for (int j = 0; j < n; j++) {
                while (sets[j][cursors[j]] < max) {
                    cursors[j]++;
                    // overflow in one of the arrays: no more matches
                    if (cursors[j] >= sizes[j]) {
                        return -1;
                    }
                }
            }
        } else {
            // current position is a match so all cursors are incremented of 1
            for (int j = 0; j < n; j++) {
                cursors[j]++;
            }
            return cursors[0] - 1;
        }
    }
}


static UnorderedHandleSet intersect(Handle** sets, int* sizes, int n)
{
    UnorderedHandleSet answer;

    // there is one cursor for each list.
    std::vector<int> cursors(n, 0);
    int pos;

    while ((pos = nextMatch(sets, sizes, cursors)) >= 0)
        answer.insert(sets[0][pos]);

    for (int i = 0; i < n; i++) {
        delete[](sets[i]);
    }
    delete[](sets);
    delete[](sizes);

    return answer;
}

static int handleCompare(const void* e1, const void* e2)
{
    return Handle::compare(*((Handle *)e1), *((Handle *)e2));
}


UnorderedHandleSet
opencog::intersection(const std::vector<UnorderedHandleSet>& sets) 
    throw (RuntimeException)
{
    // leave this method if there are no lists
    if (sets.size() == 0) {
        return UnorderedHandleSet();
    }

    // if there is at least one null list, intersection is empty
    for (unsigned int i = 0; i < sets.size(); i++) {
        if (sets[i].size() == 0)
            return UnorderedHandleSet();
    }

    Handle** v = new Handle*[sets.size()];
    int* sizeArray = new int[sets.size()];

    // The set of linked-lists is transformed into a set of sorted arrays.
    // Each list is traversed and its elements are put into a regular array
    // which is sorted before being passed to the actual intersection
    // computation algorithm. Once the lists are transformed, they are deleted.
    for (unsigned int i = 0; i < sets.size(); i++) {
        sizeArray[i] = sets[i].size();
        v[i] = new Handle[sizeArray[i]];

        // linked-list linearization
        UnorderedHandleSet::const_iterator it;
        int j = 0;
        for (it = sets[i].begin(); it != sets[i].end(); ++it) {
            v[i][j++] = *it;
        }

        if (j != sizeArray[i]) {
            throw RuntimeException(TRACE_INFO, "consistency check failed for intersection");
        }

        // each list is ordered, to make intersection algorithm cheaper.
        qsort(v[i], sizeArray[i], sizeof(Handle), handleCompare);
    }

    // this call will actually compute the intersection
    return intersect(v, sizeArray, sets.size());
}

