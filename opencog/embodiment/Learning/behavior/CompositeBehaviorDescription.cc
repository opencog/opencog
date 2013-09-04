/*
 * opencog/embodiment/Learning/behavior/CompositeBehaviorDescription.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#include <stdio.h>
#include "ElementaryBehaviorDescription.h"
#include "CompositeBehaviorDescription.h"
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/CogServer.h>
#include <algorithm>

#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>

using namespace behavior;

CompositeBehaviorDescription::CompositeBehaviorDescription(AtomSpace *_atomspace) : atomspace(_atomspace)
{
    timelineRepresentationIsValid = false;
}

CompositeBehaviorDescription::~CompositeBehaviorDescription()
{
}

bool CompositeBehaviorDescription::empty() const
{
    return entries.empty();
}

unsigned int CompositeBehaviorDescription::size() const
{
    return entries.size();
}

void CompositeBehaviorDescription::clear()
{

    if (entries.empty()) {
        return;
    }

    entries.clear();
    timelineSets.clear();
    timelineIntervals.clear();

    timelineRepresentationIsValid = false;
}

void CompositeBehaviorDescription::addPredicate(const ElementaryBehaviorDescription& ebd)
{
    entries.push_back(ebd);
    timelineRepresentationIsValid = false;
}

void CompositeBehaviorDescription::addPredicate(Handle handle, const Temporal &interval)
{
    ElementaryBehaviorDescription newEntry(handle, interval);
    entries.push_back(newEntry);
    timelineRepresentationIsValid = false;
}

void CompositeBehaviorDescription::addPredicate(Handle handle,
        unsigned long start_time,
        unsigned long end_time)
{

    addPredicate(handle, Temporal(start_time, end_time));
}



const std::vector<PredicateHandleSet> &CompositeBehaviorDescription::getTimelineSets() const
{

    if (! timelineRepresentationIsValid) {
        buildTimelineRepresentation();
    }

    return timelineSets;
}

unsigned long CompositeBehaviorDescription::getIndexStartTime(unsigned index) const
{
    if (! timelineRepresentationIsValid) {
        buildTimelineRepresentation();
    }
    unsigned long t = getStartTime();
    unsigned i = 0;
    for (std::vector<long>::const_iterator tli = timelineIntervals.begin();
            tli != timelineIntervals.end() && i < index; ++tli, i++) {
        t += (unsigned long) * tli;
    }
    return t;
}

const std::vector<long> &CompositeBehaviorDescription::getTimelineIntervals() const
{

    if (! timelineRepresentationIsValid) {
        buildTimelineRepresentation();
    }

    return timelineIntervals;
}

unsigned long CompositeBehaviorDescription::getStartTime() const
{
    if (entries.empty())
        return 0;
    else {
        unsigned long minT;
        std::vector<ElementaryBehaviorDescription>::iterator ei = entries.begin();
        minT = ei->temporal.getLowerBound();
        ++ei;
        for (; ei != entries.end(); ++ei)
            minT = std::min(minT,  ei->temporal.getLowerBound());
        return minT;
    }
}

unsigned long CompositeBehaviorDescription::getEndTime() const
{
    if (entries.empty())
        return 0;
    else {
        unsigned long maxT;
        std::vector<ElementaryBehaviorDescription>::iterator ei = entries.begin();
        maxT = ei->temporal.getUpperBound();
        ++ei;
        for (; ei != entries.end(); ++ei)
            maxT = std::max(maxT,  ei->temporal.getUpperBound());
        return maxT;
    }
}

Temporal CompositeBehaviorDescription::getTimeInterval() const
{
    if (entries.empty())
        return Temporal(0, 0);
    else {
        unsigned long minT, maxT;
        std::vector<ElementaryBehaviorDescription>::iterator ei = entries.begin();
        minT = ei->temporal.getLowerBound();
        maxT = ei->temporal.getUpperBound();
        ++ei;
        for (; ei != entries.end(); ++ei) {
            minT = std::min(minT,  ei->temporal.getLowerBound());
            maxT = std::max(maxT,  ei->temporal.getUpperBound());
        }
        return Temporal(minT, maxT);
    }
}

const std::vector<ElementaryBehaviorDescription> &CompositeBehaviorDescription::getEntries() const
{
    return entries;
}

// ********************************************************************************
// Private API

// Static to avoid linking conflicts with possible homonimous functions from
// anywhere else
static bool pairComparator(ElementaryBehaviorDescription a, ElementaryBehaviorDescription b)
{
    return a.temporal.getLowerBound() < b.temporal.getLowerBound();
}

std::vector<unsigned long> *CompositeBehaviorDescription::buildSeparatorsVector(const std::vector<ElementaryBehaviorDescription> &entries)
{

    std::vector<unsigned long> v(2 * entries.size());
    for (unsigned int i = 0; i < entries.size(); i++) {
        v[2 * i] = entries[i].temporal.getLowerBound();
        v[2 * i + 1] = entries[i].temporal.getUpperBound();
    }
    sort(v.begin(), v.end());

    std::vector<unsigned long> *answer = new std::vector<unsigned long>();
    answer->push_back(v[0]);
    unsigned long last = v[0];
    for (unsigned int j = 1; j < v.size(); j++) {
        if (v[j] != last) {
            answer->push_back(v[j]);
            last = v[j];
        }
    }

    if (answer->size() == 1) {
        answer->push_back(last);
    }

    return answer;
}

void CompositeBehaviorDescription::buildTimelineRepresentation() const
{
    buildTimelineRepresentation(timelineSets, timelineIntervals, entries);
}

void CompositeBehaviorDescription::buildTimelineRepresentation(std::vector<PredicateHandleSet> &timelineSets, std::vector<long> &timelineIntervals, std::vector<ElementaryBehaviorDescription> &entries) const
{

    if (entries.size() == 0) {
        timelineRepresentationIsValid = true;
        return;
    }

    if (! timelineSets.empty()) {
        timelineSets.clear();
    }
    if (! timelineIntervals.empty()) {
        timelineIntervals.clear();
    }

    std::vector<unsigned long> *separators = buildSeparatorsVector(entries);
    sort(entries.begin(), entries.end(), pairComparator);
    PredicateHandleSet currentSet;

    unsigned int cursor = 0;
    for (unsigned int i = 0; i < (separators->size() - 1); i++) {
        currentSet.clear();
        unsigned long intervalStart = separators->at(i);
        unsigned long intervalEnd = separators->at(i + 1);
        while ((cursor < entries.size()) && (entries[cursor].temporal.getUpperBound() < intervalStart)) {
            cursor++;
        }
        for (unsigned int j = cursor; j < entries.size(); j++) {
            unsigned long lower = entries[j].temporal.getLowerBound();
            unsigned long upper = entries[j].temporal.getUpperBound();
            Handle handle = entries[j].handle;
            if (((upper > intervalStart) && (lower < intervalEnd)) ||
                    ((upper == lower) && (upper == intervalStart)) ||
                    ((upper == lower) && (upper == intervalEnd))) {
                currentSet.insert(handle);
            }
        }
        timelineSets.push_back(currentSet);
        timelineIntervals.push_back(intervalEnd - intervalStart);
    }

    delete separators;
    OC_ASSERT(timelineSets.size() == timelineIntervals.size());
    timelineRepresentationIsValid = true;
}

std::string CompositeBehaviorDescription::toString() const
{
    //this is actually not fundamently required but forces to print
    //the CBD in chronological order
    if (! timelineRepresentationIsValid) {
        buildTimelineRepresentation();
    }

    std::string answer = "{";
    for (unsigned int i = 0; i < entries.size(); i++) {
        answer.append("(");
        answer.append(atomspace->atomAsString(entries[i].handle));
        answer.append(",");
        answer.append(entries[i].temporal.toString());
        answer.append(")");
        if (i < (entries.size() - 1)) {
            answer.append(",");
        }
    }
    answer.append("}");

    return answer;
}

// ********************************************************************************
// Test/debug



bool CompositeBehaviorDescription::equals(const CompositeBehaviorDescription &other) const
{
    if (entries.size() != other.entries.size()) {
        return false;
    }

    this->buildTimelineRepresentation();
    other.buildTimelineRepresentation();

    for (unsigned int i = 0; i < entries.size(); i++) {
        if (entries[i].handle != other.entries[i].handle)
            return false;
        else if (entries[i].temporal != other.entries[i].temporal)
            return false;
    }

    return true;
}

std::string CompositeBehaviorDescription::toStringHandles()
{

    std::string answer = "{";
    for (unsigned int i = 0; i < entries.size(); i++) {
        answer.append("(");
        answer.append(atomspace->getName(entries[i].handle));
        answer.append(",");
        answer.append(entries[i].temporal.toString());
        answer.append(")");
        if (i == (entries.size() - 1)) {
            answer.append("}");
        } else {
            answer.append(",");
        }
    }

    return answer;
}

std::string CompositeBehaviorDescription::toStringTimeline()
{
    if (! timelineRepresentationIsValid) {
        buildTimelineRepresentation();
    }

    return toStringTimeline(timelineSets, timelineIntervals);
}

std::string CompositeBehaviorDescription::toStringTimeline(
        std::vector<PredicateHandleSet> &timelineSets,
        std::vector<long> &timelineIntervals)
{

    //TODO: contigous equals sets should be merged
    std::string answer = "{";
    for (unsigned int i = 0; i < timelineSets.size(); i++) {
        answer.append("(");
        answer.append("{");
        std::vector<std::string> names;
        for (std::set<Handle>::iterator it = timelineSets[i].getSet().begin(); it != timelineSets[i].getSet().end(); ++it) {
            //the assert below is here to insure that the atom is a node
            OC_ASSERT(atomspace->isNode(*it));
            names.push_back(atomspace->getName(*it));
        }
        std::sort(names.begin(), names.end());
        for (std::vector<std::string>::iterator it = names.begin(); it != names.end(); ++it) {
            if (it != names.begin()) {
                answer.append(",");
            }
            answer.append(*it);
        }
        answer.append("},");
        char s[128];
        sprintf(s, "%ld", timelineIntervals[i]);
        answer.append(s);
        answer.append(")");
        if (i != (timelineSets.size() - 1)) {
            answer.append(",");
        }
    }
    answer.append("}");

    return answer;
}
