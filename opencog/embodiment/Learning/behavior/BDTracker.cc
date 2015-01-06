/*
 * opencog/embodiment/Learning/behavior/BDTracker.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>

#include <opencog/spacetime/HandleTemporalPair.h>
#include <opencog/spacetime/TemporalTable.h>
#include <opencog/spacetime/TimeServer.h>
#include <opencog/spacetime/SpaceTime.h>

#include "BDTracker.h"

using std::vector;
using std::map;

typedef unsigned long ulong;

namespace behavior
{

namespace param {
int MIN_TIME_TO_DENOTE_MOVEMENT_STOPPING = 1;
}

int getIntervalEnd(AtomSpace* atomspace, Handle h, const Temporal& now)
{
    vector<HandleTemporalPair> hhh;
    timeServer().getTimeInfo(back_inserter(hhh), h, now, TemporalTable::NEXT_AFTER_END_OF);


    return (hhh.size() == 1 ? hhh.rbegin()->getTemporal()->getUpperBound()
            : now.getUpperBound());

    // Assume that last interval returned is in fact the last one in temp. order
    //return (hhh.empty()
    //      ? 0
    //      : (hhh.rbegin()->getTemporal()->getUpperBound()));
}

int getIntervalStart(AtomSpace* atomspace, Handle h, const Temporal& now)
{
    vector<HandleTemporalPair> hhh;
    timeServer().getTimeInfo(back_inserter(hhh), h, now, TemporalTable::PREVIOUS_BEFORE_START_OF);

    return (hhh.size() == 1 ? hhh.rbegin()->getTemporal()->getLowerBound()
            : now.getLowerBound());

    // Assume that last interval returned is in fact the last one in temp. order
    //return (hhh.empty()
    //      ? 0
    //      : (hhh.rbegin()->getTemporal()->getLowerBound()));
}

Handle BDTracker::updateEndOfInterval(Handle perception, Handle bd, const Temporal& t_now, int old_interval_start)
{
    int old_start = (old_interval_start
                     ? old_interval_start
                     : getIntervalStart(atomspace, bd, t_now));
    int old_end = getIntervalEnd(atomspace, bd, t_now);
    int new_end = getIntervalEnd(atomspace, perception, t_now);
    // Create a new, wider interval
    Temporal new_interval(old_start, new_end);
    Temporal old_interval(old_start, old_end);

    // printf("%s %s: %d %d\n", perception->toString().c_str(),
    // bd->toString().c_str(), new_end, old_end);

    OC_ASSERT(new_end >= old_end, "new_end interval should be greater than old_end interval.");

    // Remove old interval
    timeServer().removeTimeInfo(bd, old_interval);

    // Replace with a new, wider interval
    Handle atTimeLink = timeServer().addTimeInfo(bd, new_interval);
    // TODO: check if the updateLatest bellow is really needed
    //AtomSpaceUtil::updateLatestBDInterval(atomSpace, atTimeLink, perception);

    return atTimeLink;
}

/// Give the BD-of-obj the temporal (start-of-obj, now)
Handle BDTracker::CreateBD(Handle perception, Handle obj, const Temporal& t_now, int old_interval_start)
{
    Handle ret = getBDHandle(obj, perception);

    // if (hhh.empty())
    //  printf("%s had no timestamp info\n", obj->toString().c_str());

    Temporal t(( old_interval_start ?  old_interval_start : getIntervalStart(atomspace, perception, t_now)),
               t_now.getUpperBound());

    Handle atTimeLink = timeServer().addTimeInfo(ret, t);
    // TODO: check if the updateLatest bellow is really needed
    //AtomSpaceUtil::updateLatestBDInterval(atomSpace, atTimeLink, perception, obj?);

    for (BDCreationListener* listener : BDCreationListeners)
    listener->onBD(ret, atTimeLink);

//puts("Added a BD:\n");
// puts(ret->toString().c_str());

    return atTimeLink;
}

void BDTracker::addMemberLink(Handle bdAtTimeLink, Handle trickExemplarAtTimeLink)
{
    HandleSeq memberLinkHS;
    memberLinkHS.push_back(bdAtTimeLink);
    memberLinkHS.push_back(trickExemplarAtTimeLink);
    atomspace->addLink(MEMBER_LINK, memberLinkHS);
}

bool MovementBDTracker::BDUpToDate(Handle bd, Handle percept) const
{
    HandleSeq hs1, hs11, hs2, hs21;
    hs1 = (atomspace->getOutgoing(bd)); //hs1 = (beh, eval(goto, list(pos
    if (hs1.size() < 2)
        return false;
    hs11 = atomspace->getOutgoing(hs1[1]); //hs11 = (goto, list(pos
    if (hs11.size() < 5)
        return false;

    hs2 = (atomspace->getOutgoing(percept)); //hs2 = (agisimpos, list(pos
    if (hs2.size() < 2)
        return false;
    hs21 = (atomspace->getOutgoing(hs2[1])); //hs2 = (agisimpos, list(pos
    if (hs21.size() < 2)
        return false;

    Handle x1 = hs11[2];
    Handle x2 = hs21[1];

    //printf("!!! %s / %s\n", x1->toString().c_str(),
    //      x2->toString().c_str());

    if (x1 != x2) return false;

    Handle y1 = hs11[3];
    Handle y2 = hs21[2];

    //printf("!!! %s / %s\n", y1->toString().c_str(),
    //      y2->toString().c_str());

    if (y1 != y2) return false;

    Handle z1 = hs11[4];
    Handle z2 = hs21[3];

    //printf("!!! %s / %s\n", z1->toString().c_str(),
    //      z2->toString().c_str());

    return z1 == z2;

//    printf("cmp: %s      %s\n", hs11[2]->toString().c_str(), hs2[1]->toString().c_str());
// return hs11[2]->equals(hs2[1]);
}

Handle MovementBDTracker::UpdateBD(Handle percept, Handle obj, const Temporal& now)
{
    // Remove the previous goto(x,y) of the ongoing movement, because the
    // previous assumption of the movement target is likely no longer valid.

    int old_interval_start = 0;

    map<Handle, Handle>::iterator it = obj2lastBD.find(obj);
    if (it != obj2lastBD.end()) {
        old_interval_start = getIntervalStart(atomspace, it->second, now);
        if (!BDUpToDate(it->second, percept)) {
            next_obj_has_moved = true;
//        printf("Removing old form: %s\n", it->second->toString().c_str());
            atomspace->removeAtom(it->second, false);
            Handle ret = CreateBD(percept, obj, now, old_interval_start);

//     printf("New form:%s \n",ret->toString().c_str());

            return ret;
        } else {
//      printf("Up to date: %s\n", it->second->toString().c_str());
            return updateEndOfInterval(percept, it->second, now, old_interval_start);
        }
    } else {
        Handle ret = getBDHandle(obj, percept);
        return updateEndOfInterval(percept, ret, now, old_interval_start);
    }
}

Handle MovementBDTracker::getBDHandle(Handle obj, Handle percept)
{
    HandleSeq hs2;
    hs2.push_back(atomspace->addNode(PREDICATE_NODE, "behaved"));
    HandleSeq listLinkHS;
    Handle perceptListLink = atomspace->getOutgoing(percept, 1);
    OC_ASSERT(atomspace->getType(perceptListLink) == LIST_LINK,
                     "Handle perceptListLink should be a 'LIST_LINK'.");
    HandleSeq perceptListLinkHS = atomspace->getOutgoing(perceptListLink);
    listLinkHS.push_back(perceptListLinkHS[0]);

    // next_obj_has_moved indicates that the obj in question has changed position.
    // Apologies for passing this information as a separate variable;
    // This should made more elegant when the BDTracker API is updated the next time.

// printf("getBDHandle: %s %s\n", (next_obj_has_moved ? "goto" : "standingAt"), obj->toString().c_str());
    listLinkHS.push_back(atomspace->addNode(NODE,
                                            (next_obj_has_moved ? "goto" : "standingAt")));
    next_obj_has_moved = false;

    std::copy(++(perceptListLinkHS.begin()), perceptListLinkHS.end(), back_inserter(listLinkHS));
    hs2.push_back(atomspace->addLink(LIST_LINK, listLinkHS));

    return obj2lastBD[obj] = atomspace->addLink(EVALUATION_LINK, hs2);
}

bool MovementBDTracker::isPerceptOfObj(Handle percept, Handle obj) const
{
    // TODO: Compare here for whether h is in fact a percept indicating for
    // object "obj"

    HandleSeq hs1;
    hs1 = (atomspace->getOutgoing(percept));
    if (hs1.size() < 2)
        return false;
    Handle p_obj = atomspace->getOutgoing(hs1[1], 0);

    return p_obj == obj;
}

Handle ActionBDTracker::UpdateBD(Handle percept, Handle obj, const Temporal& now)
{
    Handle ret = getBDHandle(obj, percept);
    return updateEndOfInterval(percept, ret, now);
}

Handle ActionBDTracker::getBDHandle(Handle obj, Handle percept)
{
    HandleSeq hs2;
    hs2.push_back(atomspace->addNode(PREDICATE_NODE, "behaved"));
    hs2.push_back( atomspace->getOutgoing(percept, 1) );

    return obj2lastBD[obj] = atomspace->addLink(EVALUATION_LINK, hs2);
}

bool ActionBDTracker::isPerceptOfObj(Handle percept, Handle obj) const
{
    // TODO: Compare here for whether h is in fact a percept indicating for
    // object "obj"
    // return true;

    // Eval(pred, List(obj, ...))
    HandleSeq hs1;
    hs1 = (atomspace->getOutgoing(percept));
    if (hs1.size() < 2)
        return false;
    Handle p_obj = atomspace->getOutgoing(hs1[1], 0);

    return p_obj == obj;
}

} //namespace behavior

