/*
 * opencog/spacetime/TemporalTable.cc
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

#include <set>
#include <opencog/util/Logger.h>

#include "TemporalTable.h"


//#define DPRINTF printf
#define DPRINTF(...)

// Minimal rate (number of entries / pending updates) so that an index table must be re-built
#define PENDING_UPDATE_RATE_THRESHOLD 8
#define INITIAL_INDEX_TABLE_SIZE 1

using namespace opencog;

TemporalTable::TemporalTable()
{
    handleMap = new HandleToTemporalEntryMap();
    tailHandleMap = new HandleToTemporalEntryMap();
    temporalMap = new TemporalToHandleSetMap();
    sortedTemporalList = NULL;
    temporalIndexTable = new TemporalEntry*[INITIAL_INDEX_TABLE_SIZE];
    indexTableSize = INITIAL_INDEX_TABLE_SIZE;
    indexTableCount = 0;
    pendingUpdateCount = 0;
}

TemporalTable::~TemporalTable()
{
    //delete(tailHandleMap);
    HandleMapIterator<TemporalEntry *> *keys = tailHandleMap->keys();
    while (keys->hasNext()) {
        Handle nextKey = keys->next();
        tailHandleMap->remove(nextKey);
    }
    delete(keys);
    
    delete(tailHandleMap);

    delete(handleMap);

    // get all Temporal* keys to be deleted later
    std::set<Temporal*> toBeDeleted;
    TemporalMapIterator* itr = temporalMap->keys();
    while (itr->hasNext()) {
        toBeDeleted.insert(itr->next());
    }
    delete itr;
    delete(temporalMap);
    delete(sortedTemporalList);
    delete[](temporalIndexTable);
    // delete all Temporal* keys
    for (std::set<Temporal*>::iterator it = toBeDeleted.begin();
            it != toBeDeleted.end(); ++it) {
        delete *it;
    }
}

void TemporalTable::add(Handle h, const Temporal& t)
{
    DPRINTF("TemporalTable::add(%ld, %s)\n", h.value(), t.toString().c_str());
    Temporal* internal_t = temporalMap->getKey(t);
    if (!internal_t) {
        internal_t = t.clone();
    }
    int oldNumTemporalEntries = temporalMap->getCount();
    DPRINTF("oldNumTemporalEntries = %d\n", oldNumTemporalEntries);
    DPRINTF("old sortedTemporalList = %s\n", sortedTemporalList->toString().c_str());
    TemporalEntry* previousEntry = NULL;
    if (indexTableCount > 0) {
        previousEntry = getPreviousTemporalEntry(t);
        if (previousEntry == NULL) {
            DPRINTF("previous entry is NULL => inserting directly in sortedTemporalList\n");
            sortedTemporalList = TemporalEntry::add(sortedTemporalList, internal_t);
            previousEntry = sortedTemporalList;
        } else {
            DPRINTF("Previous entry's head is %s => inserting in this entry list\n", previousEntry->time->toString().c_str());
            TemporalEntry::add(previousEntry, internal_t);
        }
    } else {
        DPRINTF("Insertion not using index table\n");
        sortedTemporalList = TemporalEntry::add (sortedTemporalList, internal_t);
    }
    DPRINTF("sortedTemporalList = %s\n", sortedTemporalList->toString().c_str());
    addToMaps(h, internal_t);
    int numTemporalEntries = temporalMap->getCount();
    DPRINTF("numTemporalEntries = %d\n", numTemporalEntries);
    if (numTemporalEntries != oldNumTemporalEntries) {
        if ((previousEntry != NULL) &&
                (indexTableCount > 0) &&
                (indexTableCount < indexTableSize) &&
                (TemporalEntry::compare(&t, temporalIndexTable[indexTableCount-1]->time) > 0)) {
            // just update IndexTable by putting the new TemporalEntry in the last position
            TemporalEntry* newEntry = previousEntry;
            while (newEntry != NULL && TemporalEntry::compare(newEntry->time, &t) != 0) {
                newEntry = newEntry->next;
            }
            if (newEntry == NULL) {
                // Should never enter here
                throw RuntimeException(TRACE_INFO,
                                       "TemporalTable - Error inserting in TemporalTable (newEntry NULL).");
            }
            temporalIndexTable[indexTableCount++] = newEntry;
            DPRINTF("indexTableCount++ => %d\n", indexTableCount);
        } else {
            pendingUpdateCount++;
            int pendingRate = (numTemporalEntries / pendingUpdateCount);
            if (pendingRate < PENDING_UPDATE_RATE_THRESHOLD) {
                updateIndexTable(numTemporalEntries);
            }
        }
    }
    DPRINTF("Affer adding => sortedTemporalList = %s\n", sortedTemporalList->toString().c_str());
}

HandleTemporalPairEntry* TemporalTable::get(Handle h, const Temporal& t, TemporalRelationship criterion)
{
    DPRINTF("TemporalTable::getHandle(h, t = %s, criterion = %s\n", t.toString().c_str(), getTemporalRelationshipStr(criterion));

    if (h == Handle::UNDEFINED) {
        return get(t, criterion);
    }
    TemporalEntry* te = handleMap->get(h);
    HandleTemporalPairEntry* result = NULL;
    HandleTemporalPairEntry* tail = NULL;
    DPRINTF("te = %s\n", te->toString().c_str());
    bool searchFinished = false;
    while (te != NULL && !searchFinished) {
        // Check if Temporal matches
        bool matches = false;
        if (t == UNDEFINED_TEMPORAL) {
            matches = true;
            DPRINTF("t == null => matched\n");
        } else {
            matches = matchesTimeCriterion(*(te->time), t, criterion, searchFinished);
            DPRINTF("Called matchesTimeCriterion(T' = %s, T = %s, criterion = %s) => matches = %d, searchFinished = %d\n", te->time->toString().c_str(), t.toString().c_str(), getTemporalRelationshipStr(criterion), matches, searchFinished);
        }
        if (matches) {
            DPRINTF("Matched!\n");
            HandleTemporalPairEntry* hte = new HandleTemporalPairEntry(HandleTemporalPair(h, te->time));
            if (t != UNDEFINED_TEMPORAL && (criterion == PREVIOUS_BEFORE_START_OF || criterion == PREVIOUS_BEFORE_END_OF)) {
                if (result) delete result; // discard previous result
                result = hte; // get the new matching result.
            } else {
                if (result == NULL) {
                    result = hte;
                } else {
                    tail->next = hte;
                }
                DPRINTF("result = %s\n", result->toString().c_str());
                tail = hte;
            }
        } else {
            DPRINTF("Not matched!\n");
        }
        te = te->next;
    }

    DPRINTF("TemporalTable::getHandle - end.\n");
    return result;
}

HandleTemporalPairEntry* TemporalTable::get(const Temporal& t, TemporalRelationship criterion)
{
    DPRINTF("TemporalTable::get(%s, %d)\n", t.toString().c_str(), criterion);

    HandleTemporalPairEntry* result = NULL;
    if (t == UNDEFINED_TEMPORAL) {
        // get all entries
        HandleTemporalPairEntry* tail = NULL;
        TemporalEntry* te = sortedTemporalList;
        while (te != NULL) {
            Temporal* time = te->time;
            UnorderedHandleSet* hs = temporalMap->get(time);
            UnorderedHandleSet::iterator itr = hs->begin();
            while (itr != hs->end()) {
                Handle h = *itr;
                ++itr;
                HandleTemporalPairEntry* hte = new HandleTemporalPairEntry(HandleTemporalPair(h, time));
                if (result == NULL) {
                    result = hte;
                } else {
                    tail->next = hte;
                }
                tail = hte;
            }
            te = te->next;
        }
    } else {
        switch (criterion) {
        case EXACT: {
            DPRINTF("ExactMatch! temporalMap = %p\n", temporalMap);
            UnorderedHandleSet* hs = temporalMap->get((Temporal*) & t);
            DPRINTF("Got hs = %p\n", hs);
            if (hs != NULL) {
                Temporal* time = NULL; // internal Temporal object (Temporal argument cannot be used in the result)
                // Build the HandleTemporalPairEntry
                UnorderedHandleSet::iterator itr = hs->begin();
                DPRINTF("Got hs iterator = %ld\n", itr->value());
                while (itr != hs->end()) {
                    DPRINTF("hs iterator has next\n");
                    Handle handle = *itr;
                    ++itr;
                    DPRINTF("Got handle = %ld\n", handle.value());
                    if (time == NULL) {
                        DPRINTF("time is NULL. Creating Temporal object\n");
                        // Find the internal Temporal object to put in the result list
                        TemporalEntry* te = handleMap->get(handle);
                        DPRINTF("Got TemporalEntry\n");
                        while (te != NULL) {
                            DPRINTF("Checking next Temporal entry\n");
                            if (TemporalEntry::compare(&t, te->time) == 0) {
                                time = te->time;
                                break;
                            }
                            te = te->next;
                        }
                    }
                    if (time == NULL) {
                        throw RuntimeException(TRACE_INFO,
                                               "TemporalTable - Could not find internal Temporal to insert into HandleTemporalPairEntry result.");
                    }
                    DPRINTF("About to concatenate handle\n");
                    HandleTemporalPairEntry* newEntry = new HandleTemporalPairEntry(handle, time);
                    DPRINTF("new Entry = %p\n", newEntry);
                    result = HandleTemporalPairEntry::concatenation(newEntry, result);
                    DPRINTF("Handle concatenated result = %p\n", result);
                    DPRINTF("result's head = (%s)\n", result->toString().c_str());
                }
                DPRINTF("hs iteration finished\n");
            }
            break;
        } // EXACT
        case OVERLAPS: {
            if (sortedTemporalList != NULL) {
                // Get all Temporal entries that insersects with the given time interval of t
                // => all Temporal time where (time->lowerBound <= t.upperBound) && (time->upperBound >= t.lowerBound)

                // Eliminates the linear lower bound tests in the beggining of the list
                // by using binary search to find the upper limit in the list.
                TemporalEntry* upperLimitEntry = getPreviousTemporalEntry(t);
                if (upperLimitEntry == NULL) {
                    upperLimitEntry = sortedTemporalList;
                }
                // Now iterates on the list until finds the upper limit
                while (upperLimitEntry != NULL) {
                    DPRINTF("Checking lowerBound of %s ...", upperLimitEntry->time->toString().c_str());
                    DPRINTF(" (upperBound of t = %ld) ", t.getUpperBound());
                    if (upperLimitEntry->time->getLowerBound() > t.getUpperBound()) {
                        DPRINTF("failed\n");
                        break;
                    }
                    DPRINTF("ok\n");
                    upperLimitEntry = upperLimitEntry->next;
                }
                // Here, we know that all entries from the beggining to the upperLimitEntry satisfy (time->lowerBound <= t.upperBound).
                // Now, check (time->upperBound >= t.lowerBound) for each one of such entries
                TemporalEntry* currentEntry = sortedTemporalList;
                HandleTemporalPairEntry* resultTail = NULL;
                while (currentEntry != upperLimitEntry) {
                    // Check if current entry matches the interval by checking the upperBound of each entry.
                    if (currentEntry->time->getUpperBound() >= t.getLowerBound()) {
                        // Matched! Add associated handles to the result.
                        UnorderedHandleSet* hs = temporalMap->get(currentEntry->time);
                        // Build the HandleTemporalPairEntry
                        UnorderedHandleSet::iterator itr = hs->begin();
                        DPRINTF("Handles for time %s:\n", currentEntry->time->toString().c_str());
                        DPRINTF("=> (not printed\n");
                        // DPRINTF("=> %s:\n", hs->toString().c_str());
                        while (itr != hs->end()) {
                            HandleTemporalPairEntry* newEntry = new HandleTemporalPairEntry(*itr, currentEntry->time);
                            ++itr;
                            if (result) {
                                resultTail->next = newEntry;
                            } else {
                                result = newEntry;
                            }
                            resultTail = newEntry;
                        }
                    }
                    currentEntry = currentEntry->next;
                }
            }
            break;
        } // OVERLAPS
        default: {
            HandleTemporalPairEntry* tail = NULL;
            bool searchFinished = false;
            TemporalEntry* te = sortedTemporalList;
            Temporal* previousTime = NULL;
            while (te != NULL && !searchFinished) {
                // Check if Temporal matches
                bool matches = false;
                if (t == UNDEFINED_TEMPORAL) {
                    matches = true;
                    DPRINTF("t == null => matched\n");
                } else {
                    matches = matchesTimeCriterion(*(te->time), t, criterion, searchFinished);
                    DPRINTF("Called matchesTimeCriterion(T' = %s, T = %s, criterion = %s) => matches = %d, searchFinished = %d\n", te->time->toString().c_str(), t.toString().c_str(), getTemporalRelationshipStr(criterion), matches, searchFinished);
                }
                if (matches) {
                    DPRINTF("Matched!\n");
                    if (t != UNDEFINED_TEMPORAL && (criterion == PREVIOUS_BEFORE_START_OF || criterion == PREVIOUS_BEFORE_END_OF)) {
                        // discard previous result and get the new matching result.
                        previousTime = te->time;
                    } else {
                        Temporal* time = te->time;
                        UnorderedHandleSet* hs = temporalMap->get(time);
                        UnorderedHandleSet::iterator itr = hs->begin();
                        while (itr != hs->end()) {
                            Handle h = *itr;
                            ++itr;
                            HandleTemporalPairEntry* hte = new HandleTemporalPairEntry(HandleTemporalPair(h, time));
                            if (result == NULL) {
                                result = hte;
                            } else {
                                tail->next = hte;
                            }
                            tail = hte;
                        }
                    }
                }
                te = te->next;
            }
            if (previousTime) {
                // get all entries with the previous time
                UnorderedHandleSet* hs = temporalMap->get(previousTime);
                UnorderedHandleSet::iterator itr = hs->begin();
                while (itr != hs->end()) {
                    Handle h = *itr;
                    ++itr;
                    HandleTemporalPairEntry* hte = new HandleTemporalPairEntry(HandleTemporalPair(h, previousTime));
                    if (result == NULL) {
                        result = hte;
                    } else {
                        tail->next = hte;
                    }
                    tail = hte;
                }
            }
            break;
        }
        }
    }

    DPRINTF("TemporalTable::get() returning...\n");fflush(stdout);
    return result;
}

bool TemporalTable::remove(Handle h, const Temporal& t, TemporalRelationship criterion)
{
    DPRINTF("TemporalTable::remove(Handle h, const Temporal& t, TemporalRelationship criterion)\n");

    if (h == Handle::UNDEFINED) {
        return remove(t, criterion);
    }
    std::set<Temporal*> toBeDeleted;
    DPRINTF("TemporalTable::remove(handle)\n");
    TemporalEntry* te = handleMap->get(h);
    TemporalEntry* tailTe = tailHandleMap->get(h);

    TemporalEntry* currentTe = te;
    DPRINTF("te = %s\n", te->toString().c_str());
    bool searchFinished = false;
    bool result = false;
    TemporalEntry* previousTe = NULL;
    while (currentTe != NULL && !searchFinished) {
        // Check if Temporal matches
        bool matches = false;
        if (t == UNDEFINED_TEMPORAL) {
            matches = true;
            DPRINTF("t == null => matched\n");
        } else {
            matches = matchesTimeCriterion(*(currentTe->time), t, criterion, searchFinished);
        }
        if (matches) {
            result = true;
            DPRINTF("Matched!\n");
            if (t != UNDEFINED_TEMPORAL && (criterion == PREVIOUS_BEFORE_START_OF || criterion == PREVIOUS_BEFORE_END_OF)) {
                // discard previous result and get the new matching result.
                previousTe = currentTe;
                currentTe = currentTe->next;
            } else {
                UnorderedHandleSet* hs = temporalMap->get(currentTe->time);
                hs->erase(h);
                DPRINTF("H removed from handle set!\n");
                if (hs->size() == 0) {
                    DPRINTF("Handle set became empty!\n");
                    temporalMap->remove(currentTe->time);
                    delete hs;
                    DPRINTF("Removed from temporalMap!\n");
                    removeFromSortedEntries(*(currentTe->time));
                    DPRINTF("Removed from sorted entries!\n");
                    toBeDeleted.insert(currentTe->time);
                }
                TemporalEntry* tmpTe = currentTe;
                currentTe = currentTe->next;

                //remove from tailHandleMap
                if (tailTe != NULL && tailTe == tmpTe) {
                    tailHandleMap->remove(h);
                }

                te = TemporalEntry::remove(te, tmpTe->time);

                DPRINTF("Removed from handleMap entries!\n");
            }
        } else {
            DPRINTF("Not matched!\n");
            currentTe = currentTe->next;
        }
    }
    if (previousTe) {
        Temporal* previousTime = previousTe->time;
        UnorderedHandleSet* hs = temporalMap->get(previousTime);
        hs->erase(h);
        DPRINTF("H removed from handle set!\n");
        if (hs->size() == 0) {
            DPRINTF("Handle set became empty!\n");
            temporalMap->remove(previousTime);
            delete hs;
            DPRINTF("Removed from temporalMap!\n");
            removeFromSortedEntries(*(previousTime));
            DPRINTF("Removed from sorted entries!\n");
            toBeDeleted.insert(previousTime);
        }
        te = TemporalEntry::remove(te, previousTime);
        DPRINTF("Removed from handleMap entries! te=%p\n", te);
    }
    handleMap->remove(h);
    DPRINTF("HandleMap entries removed!\n");
    if (te != NULL) {
        DPRINTF("te not null -- HandleMap entries will be reinserted!\n");
        // Replace te in the handleMap
        handleMap->add(h, te);
        DPRINTF("HandleMap entries reinserted!\n");
    }
    // If empty temporal list => already removed from handleMap
    for (std::set<Temporal*>::iterator it = toBeDeleted.begin();
            it != toBeDeleted.end(); ++it) {
        delete *it;
    }

    DPRINTF("TemporalTable::remove(Handle, Temporal) - end.\n");

    return result;
}

bool TemporalTable::remove(const Temporal& t, TemporalRelationship criterion)
{
    bool result = false;
    DPRINTF("TemporalTable::remove(%s, %d)\n", t.toString().c_str(), criterion);
    if (t == UNDEFINED_TEMPORAL) {
        // remove all entries
        TemporalEntry* te = sortedTemporalList;
        result = (te != NULL);
        while (te != NULL) {
            Temporal* time = te->time;
            te = te->next;
            remove(*time); // TODO: Make this removal better, if performance is an issue.
        }
    } else {
        switch (criterion) {
        case EXACT: {
            DPRINTF("ExactMatch!\n");
            Temporal* internal_t = temporalMap->getKey(t);
            if (internal_t) {
                result = true;
                UnorderedHandleSet* hs = temporalMap->remove(internal_t);
                DPRINTF("Got hs = %p\n", hs);
                UnorderedHandleSet::iterator itr = hs->begin();
                DPRINTF("Got hs iterator = %ld\n", itr->value());
                while (itr != hs->end()) {
                    DPRINTF("hs iterator has next\n");
                    Handle handle = *itr;
                    ++itr;
                    DPRINTF("Got handle = %ld\n", handle.value());
                    TemporalEntry* te = handleMap->remove(handle);
                    TemporalEntry* tailTe = tailHandleMap->get(handle);
                    //remove from tailHandleMap
                    if (tailTe != NULL && tailTe == te) {
                        tailHandleMap->remove(handle);
                    }

                    te = TemporalEntry::remove(te, internal_t);
                    if (te != NULL) {
                        handleMap->add(handle, te);
                    }
                }
                DPRINTF("hs iteration finished\n");
                delete hs;
                removeFromSortedEntries(t);
                delete internal_t;
            }
            break;
        } // EXACT
        case OVERLAPS: {
            if (sortedTemporalList != NULL) {
                DPRINTF("Non ExactMatch!\n");
                // Get all Temporal entries that overlaps the given time interval of t
                // => all Temporal time where (time->lowerBound <= t.upperBound) && (time->upperBound >= t.lowerBound)

                // Eliminates the linear lower bound tests in the beggining of the list
                // by using binary search to find the upper limit in the list.
                TemporalEntry* upperLimitEntry = getPreviousTemporalEntry(t);
                if (upperLimitEntry == NULL) {
                    upperLimitEntry = sortedTemporalList;
                }
                // Now iterates on the list until finds the upper limit
                while (upperLimitEntry != NULL) {
                    DPRINTF("Checking lowerBound of %s ...", upperLimitEntry->time->toString().c_str());
                    DPRINTF(" (upperBound of t = %ld) ", t.getUpperBound());
                    if (upperLimitEntry->time->getLowerBound() > t.getUpperBound()) {
                        DPRINTF("failed\n");
                        break;
                    }
                    DPRINTF("ok\n");
                    upperLimitEntry = upperLimitEntry->next;
                }
                // Here, we know that all entries from the beggining to the upperLimitEntry satisfy (time->lowerBound <= t.upperBound).
                // Now, check (time->upperBound >= t.lowerBound) for each one of such entries
                TemporalEntry* currentEntry = sortedTemporalList;
                while (currentEntry != upperLimitEntry) {
                    // Check if current entry matches the interval by checking the upperBound of each entry.
                    if (currentEntry->time->getUpperBound() >= t.getLowerBound()) {
                        result = true;
                        // Matched! Gets the time to be removed.
                        Temporal* time = currentEntry->time;
                        // Gets the next time entry to check before removing the current one
                        currentEntry = currentEntry->next;
                        // Removes the matched time using this same method, but with exact match
                        remove(*time); // TODO: Make this removal better, if performance is an issue.
                    } else {
                        currentEntry = currentEntry->next;
                    }
                }
            }
            break;
        } // OVERLAPS
        case INCLUDES:
        default: {
            TemporalEntry* te = sortedTemporalList;
            DPRINTF("Default: te = %s\n", te->toString().c_str());
            bool searchFinished = false;
            Temporal* previousTime = NULL;
            while (te != NULL && !searchFinished) {
                // Check if Temporal matches
                Temporal* time = te->time;
                te = te->next;
                bool matches = matchesTimeCriterion(*time, t, criterion, searchFinished);
                if (matches) {
                    result = true;
                    if (criterion == PREVIOUS_BEFORE_START_OF || criterion == PREVIOUS_BEFORE_END_OF) {
                        previousTime = time;
                    } else {
                        remove(*time); // TODO: Make this removal better, if performance is an issue.
                        DPRINTF("After removal: te = %s\n", te->toString().c_str());
                    }
                }
            }
            if (previousTime) {
                remove(*previousTime); // TODO: Make this removal better, if performance is an issue.
            }
            break;
        }
        } // switch (criterion)
    }
    
    DPRINTF("TemporalTable::remove(Temporal) - end.\n");

    return result;
}

void TemporalTable::removeFromSortedEntries(const Temporal& t)
{
    DPRINTF("TemporalTable::removeFromSortedEntries(%s)\n", t.toString().c_str());
    // First check if corresponding te is in the index table
    int pos = getTemporalIndexTablePos(t);
    DPRINTF("Index table pos = %d!\n", pos);
    if (pos >= 0 && TemporalEntry::compare(temporalIndexTable[pos]->time, &t) == 0) {
        TemporalEntry* entry = temporalIndexTable[pos];
        DPRINTF("Temporal t found in index!\n");
        if (entry->next != NULL) {
            // TODO: THIS IS VERY BAD WHEN REMOVING TEMPORAL OBJECTS IN CRONOLOGICAL ASCENDENT ORDER: EACH REMOVAL ONE MORE REPLACEMENT IS NEEDED.
            DPRINTF("TemporalEntry has next => replace all its ocurrences by the its next in index table!\n");
            pos = replaceIndexTablePosition(pos, entry->next);
        } else {
            DPRINTF("TemporalEntry has no next => So, just remove all positions with this entry from the index!\n");
            // Look backward for this entry
            pos--;
            while (pos >= 0 && temporalIndexTable[pos] == entry) {
                pos--;
            }
            indexTableCount = pos + 1;
            DPRINTF("indexTableCount = pos+1 => %d\n", indexTableCount);
        }
    }
    DPRINTF("Previous pos = %d!\n", pos);
    // Now removes the entry from the linked list
    TemporalEntry* previousTe;
    TemporalEntry* currentTe;
    if (pos < 0) {
        previousTe = NULL;
        currentTe = sortedTemporalList;
    } else {
        previousTe = temporalIndexTable[pos];
        currentTe = previousTe->next;
    }
    DPRINTF("Looking for TemporalEntry to be removed in %s!\n", currentTe->toString().c_str());
    while (currentTe != NULL) {
        int compare = TemporalEntry::compare(currentTe->time, &t);
        if (compare == 0) {
            DPRINTF("Found TemporalEntry to be removed!\n");
            // Found the point to remove
            if (previousTe != NULL) {
                DPRINTF("Has previous entry. Just update its next!\n");
                previousTe->next = currentTe->next;
            } else {
                DPRINTF("Has no previous entry. Update sorted list!\n");
                sortedTemporalList = currentTe->next;
            }
            currentTe->next = NULL;
            DPRINTF("TemporalEntry removed!\n");
            break;
        } else if (compare > 0) {
            DPRINTF("TemporalEntry not found!\n");
            // Not found
            return;
        }
        previousTe = currentTe;
        currentTe = currentTe->next;
    }
    // Check if it was really removed.
    if (currentTe != NULL) {
        if ((indexTableCount > 0) && (currentTe == temporalIndexTable[indexTableCount-1])) {
            indexTableCount--;
            DPRINTF("indexTableCount-- => %d\n", indexTableCount);
        } else {
            pendingUpdateCount++;
            int numTemporalEntries = temporalMap->getCount();
            int pendingRate = (numTemporalEntries / pendingUpdateCount);
            if (pendingRate < PENDING_UPDATE_RATE_THRESHOLD) {
                updateIndexTable(numTemporalEntries);
            }
        }
        delete currentTe;
    }
    DPRINTF("TemporalTable::removeFromSortedEntries - end.\n");
}

int TemporalTable::replaceIndexTablePosition(int pos, TemporalEntry* newEntry)
{
    DPRINTF("TemporalTable::replaceIndexTablePosition - init.\n");

    TemporalEntry* oldEntry = temporalIndexTable[pos];
    temporalIndexTable[pos] = newEntry;
    // Propagates replacement forward
    int fPos = pos + 1;
    while (fPos < indexTableCount && TemporalEntry::compare(oldEntry->time, temporalIndexTable[fPos]->time) == 0) {
        temporalIndexTable[fPos] = newEntry;
        fPos++;
    }
    // Propagates replacement backward
    int bPos = pos - 1;
    while (bPos >= 0 && TemporalEntry::compare(oldEntry->time, temporalIndexTable[bPos]->time) == 0) {
        temporalIndexTable[bPos] = newEntry;
        bPos--;
    }

    DPRINTF("TemporalTable::replaceIndexTablePosition - end.\n");
    return bPos;
}

void TemporalTable::addToMaps(Handle h, Temporal* t)
{
    DPRINTF("TemporalTable::addToMaps - init.\n");

    // Add to handleMap
    TemporalEntry* timeEntry;
    if (handleMap->contains(h)) {
        timeEntry = handleMap->remove(h);
    } else {
        timeEntry = NULL;
    }
    //search in tail handle map
    TemporalEntry* tailTimeEntry = NULL;
    if (tailHandleMap->contains(h)) {
        tailTimeEntry = tailHandleMap->remove(h);
        if (TemporalEntry::compare(t, tailTimeEntry->time) > 0) {
            DPRINTF("TemporalTable::addToMaps - using tail.\n");
            TemporalEntry::add(tailTimeEntry, t);
            tailTimeEntry = tailTimeEntry->last();
        } else {
            timeEntry = TemporalEntry::add(timeEntry, t);
        }
    } else {
        DPRINTF("TemporalTable::addToMaps - set tail.\n");
        timeEntry = TemporalEntry::add(timeEntry, t);
        tailTimeEntry = timeEntry->last();
    }

    handleMap->add(h, timeEntry);
    tailHandleMap->add(h, tailTimeEntry);


    // Add to temporalMap
    UnorderedHandleSet* handleSet;
    if (temporalMap->contains(t)) {
        handleSet = temporalMap->remove(t);
    } else {
        handleSet = new UnorderedHandleSet();
    }
    handleSet->insert(h);
    temporalMap->add(t, handleSet);

    DPRINTF("TemporalTable::addToMaps - end.\n");
}

void TemporalTable::updateIndexTable(int numEntries)
{
    DPRINTF("TemporalTable::updateIndexTable(%d)\n", numEntries);
    if ((numEntries > indexTableSize) || (numEntries < indexTableSize / 4)) {
        // needs to allocate more memory or free unecessary memory
        delete[](temporalIndexTable);
        indexTableSize = numEntries * 2;
        DPRINTF("indexTableSize => %d\n", indexTableSize);
        temporalIndexTable = new TemporalEntry*[indexTableSize];
    }
    TemporalEntry* current = sortedTemporalList;
    int i = 0;
    while (current != NULL && i < numEntries) {
        temporalIndexTable[i++] = current;
        current = current->next;
    }
    indexTableCount = i;
    DPRINTF("indexTableCount = i => %d\n", indexTableCount);
    if (i != numEntries) {
        logger().warn("WARN: Inconsistent sizes (sortedTemporalList => %d, numEntries => %d)\n", i, numEntries);
    }
    pendingUpdateCount = 0;
    DPRINTF("TemporalTable::updateIndexTable - end.\n");
}


int TemporalTable::getTemporalIndexTablePos(const Temporal& t)
{
    DPRINTF("TemporalTable::getTemporalIndexTablePos - init.\n");

    // Look up at index table, using binary search
    int low = 0;
    int up = indexTableCount - 1;
    do {
        int pos = (up + low) / 2;
        DPRINTF("getTemporalIndexTablePos(): low = %d, up = %d, pos = %d\n", low, up, pos);
        int compare = TemporalEntry::compare(temporalIndexTable[pos]->time, &t);
        if (compare > 0) {
            up = pos - 1;
        } else if (compare < 0) {
            low = pos + 1;
        } else {
            DPRINTF("getTemporalIndexTablePos(): key found!\n");
            return pos;
        }
    } while (low <= up);
    DPRINTF("getTemporalIndexTablePos(): key not found!\n");
    return up;
}

TemporalEntry* TemporalTable::getPreviousTemporalEntry(const Temporal& t)
{
    DPRINTF("TemporalTable::getPreviousTemporalEntry init\n");

    int pos = getTemporalIndexTablePos(t);
    DPRINTF("Got pos = %d for Temporal %s\n", pos, t.toString().c_str());
    if (pos >= 0 && TemporalEntry::compare(temporalIndexTable[pos]->time, &t) == 0) {
        // key found. Get previous pos
        pos--;
    }

    DPRINTF("TemporalTable::getPreviousTemporalEntry end\n");
    if (pos < 0) {
        return NULL;
    } else {
        return temporalIndexTable[pos];
    }
}

bool TemporalTable::matchesTimeCriterion(const Temporal& time, const Temporal& t, TemporalRelationship criterion, bool& searchFinished)
{
    bool matches = false;
    switch (criterion) {
    case EXACT:
        matches = (t == time);
        if (matches) {
            searchFinished = true;
        }
        break;
    case OVERLAPS:
        matches = (t.getLowerBound() <= time.getUpperBound()) && (t.getUpperBound() >= time.getLowerBound());
        if (!matches && (time.getLowerBound() > t.getUpperBound())) {
            searchFinished = true;
        }
        break;
    case INCLUDES:
        matches = (time.getLowerBound() <= t.getLowerBound()) && (time.getUpperBound() >= t.getUpperBound());
        if (!matches && (time.getLowerBound() > t.getLowerBound())) {
            searchFinished = true;
        }
        break;
    case STARTS_BEFORE:
        matches = (time.getLowerBound() < t.getLowerBound());
        if (!matches && (time.getLowerBound() >= t.getLowerBound())) {
            searchFinished = true;
        }
        break;
    case STARTS_WITHIN:
        matches = (time.getLowerBound() >= t.getLowerBound()) && (time.getLowerBound() <= t.getUpperBound());
        if (!matches && (time.getLowerBound() > t.getUpperBound())) {
            searchFinished = true;
        }
        break;
    case STARTS_AFTER:
        matches = (time.getLowerBound() > t.getUpperBound());
        break;
    case ENDS_BEFORE:
        matches = (time.getUpperBound() < t.getLowerBound());
        if (!matches && (time.getLowerBound() >= t.getLowerBound())) {
            searchFinished = true;
        }
        break;
    case ENDS_WITHIN:
        matches = (time.getUpperBound() >= t.getLowerBound()) && (time.getUpperBound() <= t.getUpperBound());
        if (!matches && (time.getLowerBound() > t.getUpperBound())) {
            searchFinished = true;
        }
        break;
    case ENDS_AFTER:
        matches = (time.getUpperBound() > t.getUpperBound());
        break;
    case NEXT_AFTER_START_OF:
        matches =  (time.getLowerBound() > t.getLowerBound());
        if (matches) {
            searchFinished = true; // gets only the next
        }
        break;
    case NEXT_AFTER_END_OF:
        matches =  (time.getLowerBound() > t.getUpperBound());
        if (matches) {
            searchFinished = true; // gets only the next
        }
        break;
    case PREVIOUS_BEFORE_START_OF:
        matches =  (time.getLowerBound() < t.getLowerBound());
        if (!matches) {
            searchFinished = true; // gets only the previous
        }
        break;
    case PREVIOUS_BEFORE_END_OF:
        matches =  (time.getLowerBound() < t.getUpperBound());
        if (!matches) {
            searchFinished = true; // gets only the previous
        }
        break;
    default:
        throw RuntimeException(TRACE_INFO,
                               "Operation is not implemented yet: '%s'.", getTemporalRelationshipStr(criterion));
        break;
    }
    return matches;
}


const char* TemporalTable::getTemporalRelationshipStr(TemporalRelationship criterion)
{
    switch (criterion) {
    case EXACT:
        return "EXACT";
        break;
    case OVERLAPS:
        return "OVERLAPS";
        break;
    case INCLUDES:
        return "INCLUDES";
        break;
    case STARTS_BEFORE:
        return "STARTS_BEFORE";
        break;
    case STARTS_WITHIN:
        return "STARTS_WITHIN";
        break;
    case STARTS_AFTER:
        return "STARTS_AFTER";
        break;
    case ENDS_BEFORE:
        return "ENDS_BEFORE";
        break;
    case ENDS_WITHIN:
        return "ENDS_WITHIN";
        break;
    case ENDS_AFTER:
        return "ENDS_AFTER";
        break;
    case NEXT_AFTER_START_OF:
        return "NEXT_AFTER_START_OF";
        break;
    case NEXT_AFTER_END_OF:
        return "NEXT_AFTER_END_OF";
        break;
    case PREVIOUS_BEFORE_START_OF:
        return "PREVIOUS_BEFORE_START_OF";
        break;
    case PREVIOUS_BEFORE_END_OF:
        return "PREVIOUS_BEFORE_END_OF";
        break;
    }
    return "UNKNOWN";

}

