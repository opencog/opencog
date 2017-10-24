/*
 * opencog/spacetime/TemporalEntry.cc
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

#include "TemporalEntry.h"

#include <iostream>

#include <opencog/util/exceptions.h>

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

int TemporalEntry::existingObjects = 0;

TemporalEntry::TemporalEntry(Temporal* time)
{

    ++existingObjects;

    DPRINTF("Total temporalEntrys: %d\n", existingObjects);

    // linked-list with no head cell
    this->time = time;
    next = NULL;
}

TemporalEntry::~TemporalEntry()
{

    --existingObjects;
    // recursion is not used to avoid stack overflow on large lists
    if (next != NULL) {
        TemporalEntry *p, *q;
        p = q = next;
        while (p->next != NULL) {
            p = p->next;
            q->next = NULL;
            delete q;
            q = p;
        }
        delete p;
    }
}

TemporalEntry* TemporalEntry::clone()
{

    if (this == NULL) return(NULL);

    TemporalEntry *answer, *p, *q;

    // answer keeps the cloned list head while p is used to iterate in the
    // cloned list
    answer = p = new TemporalEntry(time);
    q = this->next;
    while (q != NULL) {
        p->next = new TemporalEntry(q->time);
        p = p->next;
        q = q->next;
    }
    return answer;
}

int TemporalEntry::getSize()
{

    TemporalEntry *current = this;
    int size = 0;
    // O(n) to get list size
    while (current != NULL) {
        current = current->next;
        size++;
    }
    return size;
}

TemporalEntry* TemporalEntry::last()
{
    TemporalEntry *current = this;
    // O(n) to get last member
    while (current->next != NULL) {
        current = current->next;
    }
    return current;
}

bool TemporalEntry::contains(Temporal* t)
{
    TemporalEntry *current = this;
    while (current != NULL) {
        int comparison = compare(current->time, t);
        if (comparison == 0) {
            return true;
        } else if (comparison > 0) {
            return false;
        }
        current = current->next;
    }
    return false;
}

TemporalEntry* TemporalEntry::intersection(TemporalEntry* set1, TemporalEntry* set2)
{

    TemporalEntry** sets = new TemporalEntry*[2];
    sets[0] = set1;
    sets[1] = set2;
    return intersection(sets, 2);
}

int TemporalEntry::temporalCompare(const void* e1, const void* e2)
{
    const Temporal **t1 = (const Temporal **)e1;
    const Temporal **t2 = (const Temporal **)e2;
    return TemporalEntry::compare(*t1, *t2);
}

int TemporalEntry::compare(const Temporal* t1, const Temporal* t2)
{
    // Special cases where at least one of them are NULL.
    if (t1 == NULL) {
        if (t2 != NULL) {
            return -1;
        } else {
            return 0;
        }
    } else if (t2 == NULL) {
        return 1;
    } else {
        return t1->compareTo(t2);
    }
}

TemporalEntry* TemporalEntry::remove(TemporalEntry* set, Temporal* t)
{
    TemporalEntry* buffer;
    // The search for invalid elements needs to be done in two steps because
    // invalid elements found in the middle of the list need to be treated
    // differently from invalid elements found in its begining.
    DPRINTF("TemporalEntry::remove(set=%s, t=%s)\n", set->toString().c_str(), t->toString().c_str());
    while ((set != NULL) &&
            (*(set->time) == *t)) {
        DPRINTF("set->time == t. Removing head\n");
        buffer = set;
        set = set->next;
        buffer->next = NULL;
        delete buffer;
    }
    DPRINTF("remove; after first loop: (set=%s, t=%s)\n", set->toString().c_str(), t->toString().c_str());
    if (set == NULL) return NULL;
    TemporalEntry* head = set;
    while (set->next != NULL) {
        DPRINTF("remove: set->next->time = %s, t = %s, equals = %d\n", set->next->time->toString().c_str(), t->toString().c_str(), set->next->time == t);
        if (*(set->next->time) == *t) {
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

TemporalEntry* TemporalEntry::add(TemporalEntry* sortedSet, Temporal* t)
{
    DPRINTF("TemporalEntry::add(%s,%s)\n", sortedSet->toString().c_str(), t->toString().c_str());
    TemporalEntry* current = sortedSet;
    TemporalEntry* previous = NULL;
    while (current != NULL) {
        int comparison = TemporalEntry::compare(current->time, t);
        DPRINTF("Comparison = %d\n", comparison);
        if (comparison == 0) {
            // Already in the sorted list. Do nothing
            DPRINTF("TemporalEntry::add => Already exists\n");
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
    TemporalEntry* newEntry = new TemporalEntry(t);
    newEntry->next = current;
    if (previous != NULL) {
        // Added in the middle or end of the list
        previous->next = newEntry;
        DPRINTF("TemporalEntry::add => %s\n", sortedSet->toString().c_str());
        return sortedSet;
    } else {
        // Added in the first position
        DPRINTF("TemporalEntry::add => %s\n", newEntry->toString().c_str());
        return newEntry;
    }
}

TemporalEntry* TemporalEntry::intersection(TemporalEntry** sets, int n)
{

    // leave this method if there are no lists
    if (n == 0) {
        delete[](sets);
        return NULL;
    }

    // if there is at least one null list, intersection is empty
    for (int i = 0; i < n; i++) {
        if (sets[i] == NULL) {
            for (int j = 0; j < n; j++) {
                if (sets[j] != NULL) {
                    delete sets[j];
                }
            }
            //delete(sets);
            delete[](sets);
            return NULL;
        }
    }

    Temporal*** v = new Temporal**[n];
    int* size = new int[n];

    // The set of linked-lists is transformed into a set of sorted arrays.
    // Each list is traversed and its elements are put into a regular array
    // which is sorted before being passed to the actual intersection
    // computation algorithm. Once the lists are transformed, they are deleted.
    for (int i = 0; i < n; i++) {
        size[i] = sets[i]->getSize();
        v[i] = new Temporal*[size[i]];

        // linked-list linearization
        TemporalEntry* current = sets[i];
        int j = 0;
        while (current != NULL) {
            v[i][j++] = current->time;
            current = current->next;
        }

        if (j != size[i]) {
            // Ugh, lots of memory to free...
            delete[] size;
            for (int k = 0; k <= i; k++) delete v[k];
            delete v;
            throw InconsistenceException(TRACE_INFO,
                                         "TemporalEntry - Consistency check failed for intersection.");
        }

        // sets[i] is destroyed after v is set.
        delete sets[i];

        // each list is ordered, to make intersection algorithm cheaper.
        qsort(v[i], size[i], sizeof(Temporal*), TemporalEntry::temporalCompare);
    }
    delete[](sets);

    // this call will actually compute the intersection
    return intersection(v, size, n);
}

int TemporalEntry::nextMatch(Temporal*** sets, int* sizes, std::vector<int>& cursors)
{

    int n = cursors.size();
    for (;;) {
        bool fail = false;
        Temporal* max = (Temporal*) ((unsigned int) 0); // this is zero, not null (the smallest possible pointer)
        // if current positions stored in cursors is not a match, the largest
        // value pointed by cursors is computed to reposition cursors
        // accordingly
        for (int j = 0; j < n; j++) {
            // overflow in one of the arrays: no more matches
            if (cursors[j] >= sizes[j]) {
                return -1;
            }
            if (compare(sets[j][cursors[j]], max) > 0) {
                max = sets[j][cursors[j]];
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
            for (int j = 0; j < n; j++) {
                while (compare(sets[j][cursors[j]], max) < 0) {
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

TemporalEntry* TemporalEntry::intersection(Temporal*** sets, int* sizes, int n)
{

    TemporalEntry *answer = NULL, *current = NULL;

    // there is one cursor for each list.
    std::vector<int> cursors(n, 0);
    int pos;

    while ((pos = nextMatch(sets, sizes, cursors)) >= 0) {
        if (answer == NULL) {
            answer = current = new TemporalEntry(sets[0][pos]);
        } else {
            current->next = new TemporalEntry(sets[0][pos]);
            current = current->next;
        }
    }

    for (int i = 0; i < n; i++) {
        delete[](sets[i]);
    }
    delete[](sets);
    delete[](sizes);

    return answer;
}

TemporalEntry* TemporalEntry::concatenation(TemporalEntry* set1, TemporalEntry* set2)
{

    if (set1 == NULL) return set2;

    // it scans the first list until the last element is reached, and then
    // it makes the next of the last element point to the first element of the
    // second list.
    TemporalEntry* current = set1;
    while (current->next != NULL) {
        current = current->next;
    }
    current->next = set2;

    return set1;
}

std::string TemporalEntry::toString()
{

    std::string answer;

    for (TemporalEntry* current = this; current != NULL; current = current->next) {
        answer += current->time->toString();
        answer +=  " -> ";
    }
    answer +=  "NULL";
    return answer;
}

Temporal** TemporalEntry::toTemporalVector(Temporal**& vector, int& n)
{
    n = getSize();
    vector = new Temporal*[n];
    int i = 0;
    for (TemporalEntry* current = this; current != NULL; current = current->next) {
        vector[i++] = current->time;
    }
    if (i != n) {
        throw InconsistenceException(TRACE_INFO, "TemporalEntry - Consistency check failed for time entry size");
    }
    return vector;
}

TemporalEntry* TemporalEntry::fromTemporalVector(Temporal** vector, int size)
{
    TemporalEntry *ret = NULL;
    for (int i = size - 1; i >= 0; i--) {
        TemporalEntry *temp = new TemporalEntry(vector[i]);
        ret = TemporalEntry::concatenation(temp, ret);
    }
    return(ret);
}
