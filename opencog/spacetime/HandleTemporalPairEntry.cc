/*
 * opencog/spacetime/HandleTemporalPairEntry.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#include <algorithm>

#include <opencog/util/platform.h>

#include "TemporalEntry.h"
#include "HandleTemporalPairEntry.h"

using namespace opencog;

int HandleTemporalPairEntry::existingObjects = 0;

HandleTemporalPairEntry::HandleTemporalPairEntry(const HandleTemporalPair& htp) : handleTemporalPair(htp)
{

    ++existingObjects;
    // linked-list with no head cell
    next = NULL;
}

HandleTemporalPairEntry::HandleTemporalPairEntry(Handle h, Temporal* t): handleTemporalPair(h, t)
{

    ++existingObjects;
    // linked-list with no head cell
    next = NULL;
}

HandleTemporalPairEntry::~HandleTemporalPairEntry()
{

    // recursion is not used to avoid stack overflow on large lists
    if (next != NULL) {
        HandleTemporalPairEntry *p, *q;
        p = q = next;
        while (p->next != NULL) {
            p = p->next;
            q->next = NULL;
            delete q;
            q = p;
        }
        delete p;
    }
    --existingObjects;
}

HandleTemporalPairEntry* HandleTemporalPairEntry::clone()
{

    if (this == NULL) return(NULL);

    HandleTemporalPairEntry *answer, *p, *q;

    // answer keeps the cloned list head while p is used to iterate in the
    // cloned list
    answer = p = new HandleTemporalPairEntry(handleTemporalPair);
    q = this->next;
    while (q != NULL) {
        p->next = new HandleTemporalPairEntry(q->handleTemporalPair);
        p = p->next;
        q = q->next;
    }
    return answer;
}

int HandleTemporalPairEntry::getSize()
{

    HandleTemporalPairEntry *current = this;
    int size = 0;
    // O(n) to get list size
    while (current != NULL) {
        current = current->next;
        size++;
    }
    return size;
}

HandleTemporalPairEntry* HandleTemporalPairEntry::last()
{
    HandleTemporalPairEntry *current = this;
    // O(n) to get last member
    while (current->next != NULL) {
        current = current->next;
    }
    return current;
}

bool HandleTemporalPairEntry::contains(const HandleTemporalPair& t)
{
    HandleTemporalPairEntry *current = this;
    while (current != NULL) {
        int comparison = compare(current->handleTemporalPair, t);
        if (comparison == 0) {
            return true;
        } else if (comparison > 0) {
            return false;
        }
        current = current->next;
    }
    return false;
}

HandleTemporalPairEntry* HandleTemporalPairEntry::intersection(HandleTemporalPairEntry* set1, HandleTemporalPairEntry* set2)
{

    HandleTemporalPairEntry** sets = new HandleTemporalPairEntry*[2];
    sets[0] = set1;
    sets[1] = set2;
    return intersection(sets, 2);
}

int HandleTemporalPairEntry::handleTemporalPairCompare(const void* e1, const void* e2)
{
    HandleTemporalPair* ht1 = (HandleTemporalPair*)e1;
    HandleTemporalPair* ht2 = (HandleTemporalPair*)e2;
    return HandleTemporalPairEntry::compare(*ht1, *ht2);
}

int HandleTemporalPairEntry::compare(const HandleTemporalPair& ht1, const HandleTemporalPair& ht2)
{
    int handleDiff = Handle::compare(ht1.getHandle(), ht2.getHandle());
    if (handleDiff != 0) {
        return handleDiff;
    } else {
        return TemporalEntry::compare(ht1.getTemporal(), ht2.getTemporal());
    }
}

HandleTemporalPairEntry* HandleTemporalPairEntry::remove(HandleTemporalPairEntry* set, const HandleTemporalPair& t)
{
    HandleTemporalPairEntry* buffer;
    // The search for invalid elements need to be done in two steps because
    // invalid elements found in the middle of the list need to be treated
    // differently from invalid elements found in its begining.
    while ((set != NULL) &&
            (HandleTemporalPairEntry::compare(set->handleTemporalPair, t) == 0)) {
        buffer = set;
        set = set->next;
        buffer->next = NULL;
        delete buffer;
    }
    if (set == NULL) return NULL;
    HandleTemporalPairEntry* head = set;
    while (set->next != NULL) {
        if (HandleTemporalPairEntry::compare(set->next->handleTemporalPair, t) == 0) {
            buffer = set->next;
            set->next = set->next->next;
            buffer->next = NULL;
            delete buffer;
        } else {
            set = set->next;
        }
    }
    // head contains the filtered list.
    return head;
}

HandleTemporalPairEntry* HandleTemporalPairEntry::add(HandleTemporalPairEntry* sortedSet, const HandleTemporalPair& ht)
{
    HandleTemporalPairEntry* current = sortedSet;
    HandleTemporalPairEntry* previous = NULL;
    while (current != NULL) {
        int comparison = HandleTemporalPairEntry::compare(current->handleTemporalPair, ht);
        if (comparison == 0) {
            // Already in the sorted list. Do nothing
            return sortedSet;
        }
        if (comparison > 0) {
            // Found insertion position
            break;
        }
        previous = current;
        current = current->next;
    }
    // Found insertion position (even if list is empty or reach last entry)
    HandleTemporalPairEntry* newEntry = new HandleTemporalPairEntry(ht);
    newEntry->next = current;
    if (previous != NULL) {
        // Added in the middle or end of the list
        previous->next = newEntry;
        return sortedSet;
    } else {
        // Added in the first position
        return newEntry;
    }
}

HandleTemporalPairEntry* HandleTemporalPairEntry::intersection(HandleTemporalPairEntry** sets, unsigned int n)
{

    // leave this method if there are no lists
    if (n == 0) {
        delete[](sets);
        return NULL;
    }

    // if there is at least one null list, intersection is empty
    for (unsigned int i = 0; i < n; i++) {
        if (sets[i] == NULL) {
            for (unsigned int j = 0; j < n; j++) {
                if (sets[j] != NULL) {
                    delete sets[j];
                }
            }
            //delete(sets);
            delete[](sets);
            return NULL;
        }
    }

    std::vector<std::vector<HandleTemporalPair> > v(n);

    // The set of linked-lists is transformed into a set of sorted arrays.
    // Each list is traversed and its elements are put into a regular array
    // which is sorted before being passed to the actual intersection
    // computation algorithm. Once the lists are transformed, they are deleted.
    for (unsigned int i = 0; i < n; i++) {
        v[i].resize(sets[i]->getSize());

        // linked-list linearization
        HandleTemporalPairEntry* current = sets[i];
        unsigned int j = 0;
        while (current != NULL) {
            v[i][j++] = current->handleTemporalPair;
            current = current->next;
        }

        if (j != v[i].size()) {
            throw InconsistenceException(TRACE_INFO, "consistency check failed for intersection");
        }

        // sets[i] is destroyed after v is set.
        delete sets[i];

        // each list is ordered, to make intersection algorithm cheaper.
        std::sort(v[i].begin(), v[i].end(), HandleTemporalPairEntry::SortComparison());
    }
    delete[](sets);

    // this call will actually compute the intersection
    return intersection(v);
}

int HandleTemporalPairEntry::nextMatch(std::vector<std::vector<HandleTemporalPair> >& sets, std::vector<unsigned int>& cursors)
{

    for (;;) {

        bool fail = false;
        HandleTemporalPair localmax(Handle::UNDEFINED, NULL);  // Temporal's pointer as NULL is considered the smallest Temporal value in compare method
        // if current positions stored in cursors is not a match, the largest
        // value pointed by cursors is computed to reposition cursors
        // accordingly
        for (unsigned int j = 0; j < sets.size(); j++) {
            // overflow in one of the arrays: no more matches
            if (cursors[j] >= sets[j].size()) {
                return -1;
            }
            if (compare(sets[j][cursors[j]], localmax) > 0) {
                localmax = sets[j][cursors[j]];
            }
            if (compare(sets[0][cursors[0]], sets[j][cursors[j]])) {
                fail = true;
                // can't break here in order to compute max
            }
        }


        if (fail) {
            // current position is not a match, so cursors are updated to point
            // to the next element in each array which is greater than or
            // equals to max
            for (unsigned int j = 0; j < sets.size(); j++) {
                while (compare(sets[j][cursors[j]], localmax) < 0) {
                    cursors[j]++;
                    // overflow in one of the arrays: no more matches
                    if (cursors[j] >= sets[j].size()) {
                        return -1;
                    }
                }
            }
        } else {
            // current position is a match so all cursors are incremented of 1
            for (unsigned int j = 0; j < sets.size(); j++) {
                cursors[j]++;
            }
            return cursors[0] - 1;
        }
    }
}

HandleTemporalPairEntry* HandleTemporalPairEntry::intersection(std::vector<std::vector<HandleTemporalPair> >& sets)
{

    HandleTemporalPairEntry *answer = NULL, *current = NULL;

    // there is one cursor for each list.
    std::vector<unsigned int> cursors(sets.size(), 0);
    int pos;

    while ((pos = nextMatch(sets, cursors)) >= 0) {
        if (answer == NULL) {
            answer = current = new HandleTemporalPairEntry(sets[0][pos]);
        } else {
            current->next = new HandleTemporalPairEntry(sets[0][pos]);
            current = current->next;
        }
    }

    return answer;
}

HandleTemporalPairEntry* HandleTemporalPairEntry::concatenation(HandleTemporalPairEntry* set1, HandleTemporalPairEntry* set2)
{

    if (set1 == NULL) return set2;

    // it scans the first list until the last element is reached, and then
    // it makes the next of the last element point to the first element of the
    // second list.
    HandleTemporalPairEntry* current = set1;
    while (current->next != NULL) {
        current = current->next;
    }
    current->next = set2;

    return set1;
}

std::string HandleTemporalPairEntry::toString()
{

    std::string answer;

    for (HandleTemporalPairEntry* current = this; current != NULL; current = current->next) {
        answer += current->handleTemporalPair.toString();
        answer +=  " -> ";
    }
    answer +=  "NULL";
    return answer;
}

HandleTemporalPair* HandleTemporalPairEntry::toHandleTemporalPairVector(HandleTemporalPair*& vector, int& n)
{
    n = getSize();
    vector = new HandleTemporalPair[n];
    int i = 0;
    for (HandleTemporalPairEntry* current = this; current != NULL; current = current->next) {
        vector[i++] = current->handleTemporalPair;
    }
    if (i != n) {
        throw InconsistenceException(TRACE_INFO, "consistency check failed for HandleTemporalPair entry size");
    }
    return vector;
}

HandleTemporalPairEntry* HandleTemporalPairEntry::fromHandleTemporalPairVector(HandleTemporalPair* vector, int size)
{
    HandleTemporalPairEntry *ret = NULL;
    for (int i = size - 1; i >= 0; i--) {
        HandleTemporalPairEntry *temp = new HandleTemporalPairEntry(vector[i]);
        ret = HandleTemporalPairEntry::concatenation(temp, ret);
    }
    return(ret);
}
