/*
 * opencog/atomspace/TemporalTable.cc
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

#include "Atom.h"
#include "HandleMap.h"
#include "TemporalTable.h"
#include "TLB.h"

#include <set>

#include <opencog/util/Logger.h>

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
    for (std::set<Temporal*>::iterator it = toBeDeleted.begin(); it != toBeDeleted.end(); it++) {
        delete *it;
    }
}

void TemporalTable::add(Handle h, const Temporal& t)
{

//    logger().debug("TemporalTable::add - init.");

    //printf("TemporalTable::add(%p, %s)\n", h, t.toString().c_str());
    Temporal* internal_t = temporalMap->getKey(t);
    if (!internal_t) {
        internal_t = t.clone();
    }
    int oldNumTemporalEntries = temporalMap->getCount();
    //printf("oldNumTemporalEntries = %d\n", oldNumTemporalEntries);
    //printf("old sortedTemporalList = %s\n", sortedTemporalList->toString().c_str());
    TemporalEntry* previousEntry = NULL;
    if (indexTableCount > 0) {
        previousEntry = getPreviousTemporalEntry(t);
        if (previousEntry == NULL) {
            //printf("previous entry is NULL => inserting directly in sortedTemporalList\n");
            sortedTemporalList = TemporalEntry::add(sortedTemporalList, internal_t);
            previousEntry = sortedTemporalList;
        } else {
            //printf("Previous entry's head is %s => inserting in this entry list\n", previousEntry->time->toString().c_str());
            TemporalEntry::add(previousEntry, internal_t);
        }
    } else {
        //printf("Insertion not using index table\n");
        sortedTemporalList = TemporalEntry::add (sortedTemporalList, internal_t);
    }
    //printf("sortedTemporalList = %s\n", sortedTemporalList->toString().c_str());
    addToMaps(h, internal_t);
    int numTemporalEntries = temporalMap->getCount();
    //printf("numTemporalEntries = %d\n", numTemporalEntries);
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
            //printf("indexTableCount++ => %d\n", indexTableCount);
        } else {
            pendingUpdateCount++;
            int pendingRate = (numTemporalEntries / pendingUpdateCount);
            if (pendingRate < PENDING_UPDATE_RATE_THRESHOLD) {
                updateIndexTable(numTemporalEntries);
            }
        }
    }
    //cprintf(DEBUG, "Affer adding => sortedTemporalList = %s\n", sortedTemporalList->toString().c_str());
//    logger().debug("TemporalTable::add - end.");

}

HandleTemporalPairEntry* TemporalTable::get(Handle h, const Temporal& t, TemporalRelationship criterion)
{
    //printf("get(h, t = %s, criterion = %s\n", t.toString().c_str(), getTemporalRelationshipStr(criterion));
//    logger().debug("TemporalTable::getHandle - init.");

    if (h == Handle::UNDEFINED) {
        return get(t, criterion);
    }
    TemporalEntry* te = handleMap->get(h);
    HandleTemporalPairEntry* result = NULL;
    HandleTemporalPairEntry* tail = NULL;
    //printf("te = %s\n", te->toString().c_str());
    bool searchFinished = false;
    while (te != NULL && !searchFinished) {
        // Check if Temporal matches
        bool matches = false;
        if (t == UNDEFINED_TEMPORAL) {
            matches = true;
            //printf("t == null => matched\n");
        } else {
            matches = matchesTimeCriterion(*(te->time), t, criterion, searchFinished);
            //printf("Called matchesTimeCriterion(T' = %s, T = %s, criterion = %s) => matches = %d, searchFinished = %d\n", te->time->toString().c_str(), t.toString().c_str(), getTemporalRelationshipStr(criterion), matches, searchFinished);
        }
        if (matches) {
            //printf("Matched!\n");
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
                //printf("result = %s\n", result->toString().c_str());
                tail = hte;
            }
        } else {
            //printf("Not matched!\n");
        }
        te = te->next;
    }

//    logger().debug("TemporalTable::getHandle - end.");
    //printf("Returning!\n");
    return result;
}

HandleTemporalPairEntry* TemporalTable::get(const Temporal& t, TemporalRelationship criterion)
{
    //cprintf(NORMAL, "TemporalTable::get(%s, %d)\n", t.toString().c_str(), criterion);

//    logger().debug("TemporalTable::getTemporal - init.");
    HandleTemporalPairEntry* result = NULL;
    if (t == UNDEFINED_TEMPORAL) {
        // get all entries
        HandleTemporalPairEntry* tail = NULL;
        TemporalEntry* te = sortedTemporalList;
        while (te != NULL) {
            Temporal* time = te->time;
            HandleSet* hs = temporalMap->get(time);
            HandleSetIterator* itr = hs->keys();
            while (itr->hasNext()) {
                Handle h = itr->next();
                HandleTemporalPairEntry* hte = new HandleTemporalPairEntry(HandleTemporalPair(h, time));
                if (result == NULL) {
                    result = hte;
                } else {
                    tail->next = hte;
                }
                tail = hte;
            }
            te = te->next;
            delete (itr);
        }
    } else {
        switch (criterion) {
        case EXACT: {
            //cprintf(NORMAL, "ExactMatch! temporalMap = %p\n", temporalMap);
            HandleSet* hs = temporalMap->get((Temporal*) & t);
            //cprintf(NORMAL, "Got hs = %p\n", hs);
            if (hs != NULL) {
                Temporal* time = NULL; // internal Temporal object (Temporal argument cannot be used in the result)
                // Build the HandleTemporalPairEntry
                HandleSetIterator* itr = hs->keys();
                //cprintf(NORMAL, "Got hs iterator = %p\n", itr);
                while (itr->hasNext()) {
                    //cprintf(NORMAL, "hs iterator has next\n");
                    Handle handle = itr->next();
                    //cprintf(NORMAL, "Got handle = %p\n", handle);
                    if (time == NULL) {
                        //cprintf(NORMAL, "time is NULL. Creating Temporal object\n");
                        // Find the internal Temporal object to put in the result list
                        TemporalEntry* te = handleMap->get(handle);
                        //cprintf(NORMAL, "Got TemporalEntry\n");
                        while (te != NULL) {
                            //cprintf(NORMAL, "Checking next Temporal entry\n");
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
                    //cprintf(NORMAL, "About to concatenate handle\n");
                    HandleTemporalPairEntry* newEntry = new HandleTemporalPairEntry(handle, time);
                    //cprintf(NORMAL, "new Entry = %p\n", newEntry);
                    result = HandleTemporalPairEntry::concatenation(newEntry, result);
                    //cprintf(NORMAL, "Handle concatenated result = %p\n", result);
                    //cprintf(NORMAL, "result's head = (%p,%s)\n", result->handleTime->getHandle(), result->handleTime->getTemporal()->toString().c_str());
                }
                delete itr;
                //cprintf(NORMAL, "hs iteration finished\n");
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
                    // printf("Checking lowerBound of %s ...", upperLimitEntry->time->toString().c_str());
                    // printf(" (upperBound of t = %ld) ", t.getUpperBound());
                    if (upperLimitEntry->time->getLowerBound() > t.getUpperBound()) {
                        //printf("failed\n");
                        break;
                    }
                    //printf("ok\n");
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
                        HandleSet* hs = temporalMap->get(currentEntry->time);
                        // Build the HandleTemporalPairEntry
                        HandleSetIterator* itr = hs->keys();
                        //printf("Handles for time %s:\n", currentEntry->time->toString().c_str());
                        //printf("=> %s:\n", hs->toString().c_str());
                        while (itr->hasNext()) {
                            HandleTemporalPairEntry* newEntry = new HandleTemporalPairEntry(itr->next(), currentEntry->time);
                            if (result) {
                                resultTail->next = newEntry;
                            } else {
                                result = newEntry;
                            }
                            resultTail = newEntry;
                        }
                        delete itr;
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
                    //printf("t == null => matched\n");
                } else {
                    matches = matchesTimeCriterion(*(te->time), t, criterion, searchFinished);
                    //printf("Called matchesTimeCriterion(T' = %s, T = %s, criterion = %s) => matches = %d, searchFinished = %d\n", te->time->toString().c_str(), t.toString().c_str(), getTemporalRelationshipStr(criterion), matches, searchFinished);
                }
                if (matches) {
                    //printf("Matched!\n");
                    if (t != UNDEFINED_TEMPORAL && (criterion == PREVIOUS_BEFORE_START_OF || criterion == PREVIOUS_BEFORE_END_OF)) {
                        // discard previous result and get the new matching result.
                        previousTime = te->time;
                    } else {
                        Temporal* time = te->time;
                        HandleSet* hs = temporalMap->get(time);
                        HandleSetIterator* itr = hs->keys();
                        while (itr->hasNext()) {
                            Handle h = itr->next();
                            HandleTemporalPairEntry* hte = new HandleTemporalPairEntry(HandleTemporalPair(h, time));
                            if (result == NULL) {
                                result = hte;
                            } else {
                                tail->next = hte;
                            }
                            tail = hte;
                        }
                        delete itr;
                    }
                }
                te = te->next;
            }
            if (previousTime) {
                // get all entries with the previous time
                HandleSet* hs = temporalMap->get(previousTime);
                HandleSetIterator* itr = hs->keys();
                while (itr->hasNext()) {
                    Handle h = itr->next();
                    HandleTemporalPairEntry* hte = new HandleTemporalPairEntry(HandleTemporalPair(h, previousTime));
                    if (result == NULL) {
                        result = hte;
                    } else {
                        tail->next = hte;
                    }
                    tail = hte;
                }
                delete itr;
            }
            break;
        }
        }
    }

//    logger().debug("TemporalTable::getTemporal - end.");

    //cprintf(NORMAL, "TemporalTable::get() returning...\n");fflush(stdout);
    return result;
}

bool TemporalTable::remove(Handle h, const Temporal& t, TemporalRelationship criterion)
{
//printf("TemporalTable::remove(Handle h, const Temporal& t, TemporalRelationship criterion)\n");
//    logger().debug("TemporalTable::removeHandle - init.");

    if (h == Handle::UNDEFINED) {
        return remove(t, criterion);
    }
    std::set<Temporal*> toBeDeleted;
    //printf("TemporalTable::remove(handle)\n");
    TemporalEntry* te = handleMap->get(h);
    TemporalEntry* tailTe = tailHandleMap->get(h);

    TemporalEntry* currentTe = te;
    //printf("te = %s\n", te->toString().c_str());
    bool searchFinished = false;
    bool result = false;
    TemporalEntry* previousTe = NULL;
    while (currentTe != NULL && !searchFinished) {
        // Check if Temporal matches
        bool matches = false;
        if (t == UNDEFINED_TEMPORAL) {
            matches = true;
            //printf("t == null => matched\n");
        } else {
            matches = matchesTimeCriterion(*(currentTe->time), t, criterion, searchFinished);
        }
        if (matches) {
            result = true;
            //printf("Matched!\n");
            if (t != UNDEFINED_TEMPORAL && (criterion == PREVIOUS_BEFORE_START_OF || criterion == PREVIOUS_BEFORE_END_OF)) {
                // discard previous result and get the new matching result.
                previousTe = currentTe;
                currentTe = currentTe->next;
            } else {
                HandleSet* hs = temporalMap->get(currentTe->time);
                hs->remove(h);
                //printf("H removed from handle set!\n");
                if (hs->getSize() == 0) {
                    //printf("Handle set became empty!\n");
                    temporalMap->remove(currentTe->time);
                    delete hs;
                    //printf("Removed from temporalMap!\n");
                    removeFromSortedEntries(*(currentTe->time));
                    //printf("Removed from sorted entries!\n");
                    toBeDeleted.insert(currentTe->time);
                }
                TemporalEntry* tmpTe = currentTe;
                currentTe = currentTe->next;

                //remove from tailHandleMap
                if (tailTe != NULL && tailTe == tmpTe) {
                    tailHandleMap->remove(h);
                }

                te = TemporalEntry::remove(te, tmpTe->time);



                //printf("Removed from handleMap entries!\n");
            }
        } else {
            //printf("Not matched!\n");
            currentTe = currentTe->next;
        }
    }
    if (previousTe) {
        Temporal* previousTime = previousTe->time;
        HandleSet* hs = temporalMap->get(previousTime);
        hs->remove(h);
        //printf("H removed from handle set!\n");
        if (hs->getSize() == 0) {
            //printf("Handle set became empty!\n");
            temporalMap->remove(previousTime);
            delete hs;
            //printf("Removed from temporalMap!\n");
            removeFromSortedEntries(*(previousTime));
            //printf("Removed from sorted entries!\n");
            toBeDeleted.insert(previousTime);
        }
        te = TemporalEntry::remove(te, previousTime);
        //printf("Removed from handleMap entries!\n");
    }
    handleMap->remove(h);
    //printf("HandleMap entries removed!\n");
    if (te != NULL) {
        // Replace te in the handleMap
        handleMap->add(h, te);
        //printf("HandleMap entries reinserted!\n");
    }
    // If empty temporal list => already removed from handleMap
    for (std::set<Temporal*>::iterator it = toBeDeleted.begin(); it != toBeDeleted.end(); it++) {
        delete *it;
    }

//    logger().debug("TemporalTable::rwmoveHandle - emd.");

    return result;
}

bool TemporalTable::remove(const Temporal& t, TemporalRelationship criterion)
{

//    logger().debug("TemporalTable::removeTemporal - init.");

    bool result = false;
    //printf("TemporalTable::remove(%s, %d)\n", t.toString().c_str(), criterion);
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
            //printf("ExactMatch!\n");
            Temporal* internal_t = temporalMap->getKey(t);
            if (internal_t) {
                result = true;
                HandleSet* hs = temporalMap->remove(internal_t);
                //cprintf(NORMAL, "Got hs = %p\n", hs);
                HandleSetIterator* itr = hs->keys();
                //cprintf(NORMAL, "Got hs iterator = %p\n", itr);
                while (itr->hasNext()) {
                    //cprintf(NORMAL, "hs iterator has next\n");
                    Handle handle = itr->next();
                    //cprintf(NORMAL, "Got handle = %p\n", handle);
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
                delete itr;
                //cprintf(NORMAL, "hs iteration finished\n");
                delete hs;
                removeFromSortedEntries(t);
                delete internal_t;
            }
            break;
        } // EXACT
        case OVERLAPS: {
            if (sortedTemporalList != NULL) {
                //printf("Non ExactMatch!\n");
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
                    // printf("Checking lowerBound of %s ...", upperLimitEntry->time->toString().c_str());
                    // printf(" (upperBound of t = %ld) ", t.getUpperBound());
                    if (upperLimitEntry->time->getLowerBound() > t.getUpperBound()) {
                        //printf("failed\n");
                        break;
                    }
                    //printf("ok\n");
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
        /* 
         * Comment from Nil Geisweiller
         * That case is not correct, it makes the wrong assumptions
         * 1) any entry lower than t can potentially include t
         * 2) checking time->upperBound <= t.upperBound is simply not in accordance with
         *    the definition of INCLUDES
         * For those reason I comment it and it uses instead the default case. Note I don't
         * think one can write a particular optimization with the considered order 
         * (start time of temporal) and forward list (next element and not previous).
         * So the default case is already the best wait to handle INCLUDES
        case INCLUDES: {
            if (sortedTemporalList != NULL) {
                //printf("Non ExactMatch!\n");
                // Get all Temporal entries that are included by the given time interval of t
                // => all Temporal time where (time->lowerBound >= t.lowerBound) && (time->upperBound <= t.upperBound)
        
                // Eliminates the linear lower bound tests in the beggining of the list 
                // by using binary search to find the lower limit in the list. 
                TemporalEntry* lowerLimitEntry = getPreviousTemporalEntry(t);
                if (lowerLimitEntry == NULL) {
                    lowerLimitEntry = sortedTemporalList;
                }
                // Now iterates on the list until finds the lower limit
                while (lowerLimitEntry != NULL) {
                    // printf("Checking lowerBound of %s ...", lowerLimitEntry->time->toString().c_str());
                    // printf(" (lowerBound of t = %ld) ", t.getLowerBound());
                    if (lowerLimitEntry->time->getLowerBound() >= t.getLowerBound()) {
                        //printf("ok\n");
                        break;                
                    }
                    //printf("failed\n");
                    lowerLimitEntry = lowerLimitEntry->next;
                }    
                // Here, we know that all entries from the lowerLimitEntry to the end satisfy (time->lowerBound >= t.lowerBound).
                // Now, check (time->upperBound <= t.upperBound) for each one of such entries
                TemporalEntry* currentEntry = lowerLimitEntry;
                while (currentEntry != NULL) {
                    // Check if current entry matches the interval by checking the upperBound of each entry.
                    if (currentEntry->time->getUpperBound() <= t.getUpperBound()) {
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
        } // INCLUDES
        */
        default: {
            TemporalEntry* te = sortedTemporalList;
            //printf("Default: te = %s\n", te->toString().c_str());
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
                        //printf("After removal: te = %s\n", te->toString().c_str());
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
//    logger().debug("TemporalTable::removeTemporal - end.");

    return result;
}

void TemporalTable::removeFromSortedEntries(const Temporal& t)
{

//    logger().debug("TemporalTable::removeFrom - init.");
    //printf("TemporalTable::removeFromSortedEntries(%s)\n", t.toString().c_str());
    // First check if corresponding te is in the index table
    int pos = getTemporalIndexTablePos(t);
    //printf("Index table pos = %d!\n", pos);
    if (pos >= 0 && TemporalEntry::compare(temporalIndexTable[pos]->time, &t) == 0) {
        TemporalEntry* entry = temporalIndexTable[pos];
        //printf("Temporal t found in index!\n");
        if (entry->next != NULL) {
            // TODO: THIS IS VERY BAD WHEN REMOVING TEMPORAL OBJECTS IN CRONOLOGICAL ASCENDENT ORDER: EACH REMOVAL ONE MORE REPLACEMENT IS NEEDED.
            //printf("TemporalEntry has next => replace all its ocurrences by the its next in index table!\n");
            pos = replaceIndexTablePosition(pos, entry->next);
        } else {
            //cprintf(NORMAL, "TemporalEntry has no next => So, just remove all positions with this entry from the index!\n");
            // Look backward for this entry
            pos--;
            while (pos >= 0 && temporalIndexTable[pos] == entry) {
                pos--;
            }
            indexTableCount = pos + 1;
            //printf("indexTableCount = pos+1 => %d\n", indexTableCount);
        }
    }
    //printf("Previous pos = %d!\n", pos);
    // Now removes the entry from the linked list
    TemporalEntry* previousTe;
    TemporalEntry* currentTe;
//    while(pos >= 0 && TemporalEntry::compare(temporalIndexTable[pos]->time,&t) >= 0) {
//       pos--;
//    }
    if (pos < 0) {
        previousTe = NULL;
        currentTe = sortedTemporalList;
    } else {
        previousTe = temporalIndexTable[pos];
        currentTe = previousTe->next;
    }
    //printf("Looking for TemporalEntry to be removed in %s!\n", currentTe->toString().c_str());
    while (currentTe != NULL) {
        int compare = TemporalEntry::compare(currentTe->time, &t);
        if (compare == 0) {
            //printf("Found TemporalEntry to be removed!\n");
            // Found the point to remove
            if (previousTe != NULL) {
                //printf("Has previous entry. Just update its next!\n");
                previousTe->next = currentTe->next;
            } else {
                //printf("Has no previous entry. Update sorted list!\n");
                sortedTemporalList = currentTe->next;
            }
            currentTe->next = NULL;
            //printf("TemporalEntry removed!\n");
            break;
        } else if (compare > 0) {
            //printf("TemporalEntry not found!\n");
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
            //cprintf(DEBUG, "indexTableCount-- => %d\n", indexTableCount);
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
//    logger().debug("TemporalTable::removeFrom - end.");
}

int TemporalTable::replaceIndexTablePosition(int pos, TemporalEntry* newEntry)
{

//    logger().debug("TemporalTable::replaceIndex - init.");

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
//    logger().debug("TemporalTable::replaceIndex - end.");
    return bPos;
}

void TemporalTable::addToMaps(Handle h, Temporal* t)
{
    // Add to handleMap
//    logger().debug("TemporalTable::addToMaps - init");

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
//      logger().debug("TemporalTable::addToMaps - using tail");
            TemporalEntry::add(tailTimeEntry, t);
            tailTimeEntry = tailTimeEntry->last();
        } else {
            timeEntry = TemporalEntry::add(timeEntry, t);
        }
    } else {
//     logger().fine("TemporalTable - addToMaps: set tail");
        timeEntry = TemporalEntry::add(timeEntry, t);
        tailTimeEntry = timeEntry->last();
    }

    handleMap->add(h, timeEntry);
    tailHandleMap->add(h, tailTimeEntry);


    // Add to temporalMap
    HandleSet* handleSet;
    if (temporalMap->contains(t)) {
        handleSet = temporalMap->remove(t);
    } else {
        handleSet = new HandleSet();
    }
    handleSet->add(h);
    temporalMap->add(t, handleSet);

//    logger().debug("TemporalTable::addToMaps - end");

}

void TemporalTable::updateIndexTable(int numEntries)
{

//    logger().debug("TemporalTable::updateIndex - init.");
    //cprintf(NORMAL, "TemporalTable::updateIndexTable(%d)\n", numEntries);
    if ((numEntries > indexTableSize) || (numEntries < indexTableSize / 4)) {
        // needs to allocate more memory or free unecessary memory
        delete[](temporalIndexTable);
        indexTableSize = numEntries * 2;
        //cprintf(NORMAL, "indexTableSize => %d\n", indexTableSize);
        temporalIndexTable = new TemporalEntry*[indexTableSize];
    }
    TemporalEntry* current = sortedTemporalList;
    int i = 0;
    while (current != NULL && i < numEntries) {
        temporalIndexTable[i++] = current;
        current = current->next;
    }
    indexTableCount = i;
    //printf("indexTableCount = i => %d\n", indexTableCount);
    if (i != numEntries) {
        logger().warn("WARN: Inconsistent sizes (sortedTemporalList => %d, numEntries => %d)\n", i, numEntries);
    }
    pendingUpdateCount = 0;
//    logger().debug("TemporalTable::updateIndex - end.");
}


int TemporalTable::getTemporalIndexTablePos(const Temporal& t)
{
//    logger().debug("TemporalTable::updateIndexPs - init.");

    // Look up at index table, using binary search
    int low = 0;
    int up = indexTableCount - 1;
    do {
        int pos = (up + low) / 2;
        //cprintf(NORMAL, "getTemporalIndexTablePos(): low = %d, up = %d, pos = %d\n", low, up, pos);
        int compare = TemporalEntry::compare(temporalIndexTable[pos]->time, &t);
        if (compare > 0) {
            up = pos - 1;
        } else if (compare < 0) {
            low = pos + 1;
        } else {
            //cprintf(NORMAL, "getTemporalIndexTablePos(): key found!\n");
            return pos;
        }
    } while (low <= up);
    //cprintf(NORMAL, "getTemporalIndexTablePos(): key not found!\n");
//    logger().debug("TemporalTable::updateIndexPs - end.");
    return up;
}

TemporalEntry* TemporalTable::getPreviousTemporalEntry(const Temporal& t)
{
//    logger().debug("TemporalTable::getPrevious - init.");

    int pos = getTemporalIndexTablePos(t);
    //cprintf(NORMAL, "Got pos = %d for Temporal %s\n", pos, t.toString().c_str());
    if (pos >= 0 && TemporalEntry::compare(temporalIndexTable[pos]->time, &t) == 0) {
        // key found. Get previous pos
        pos--;
    }
//    logger().debug("TemporalTable::getPrevious - end.");
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

