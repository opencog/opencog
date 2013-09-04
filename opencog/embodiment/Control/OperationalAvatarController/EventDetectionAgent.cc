/*
 * opencog/embodiment/Control/PerceptionActionInterface/EventDetector.cc
 *
 * Copyright (C) 2011 OpenCog Foundation
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Shujing ke (rainkekekeke@gmail.com)

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

/** EventDetector.cc
 *
 *  Detect events from continueously happening actions .
 */

#include "EventDetectionAgent.h"
#include "opencog/guile/SchemeSmob.h"
#include <iostream>
#include <fstream>
#include <iterator>
#include "OAC.h"

using namespace opencog::oac;
using namespace opencog;

map<unsigned long,Frame*> EventDetectionAgent::frames;


EventDetectionAgent::EventDetectionAgent(CogServer& cs) :
    Agent(cs), atomSpace(cs.getAtomSpace())
{
    this->cycleCount = 0;
    // dataProvider = new DataProvider(3, false);

    // ----------------pattern mining -------------------
    // NGramSkeletonArray = new Skeleton*[MAX_PATTERN_GRAM];
    // NGramPatternArray = new Pattern*[MAX_PATTERN_GRAM];

    iAtomSpace = new AtomSpace();

    rootSkeleton = new Skeleton();

    globalVarToInt.clear();
    globalIntToVar.clear();

    globalVarNum = 0;

    // ----------------pattern mining end-------------------
}

EventDetectionAgent::~EventDetectionAgent()
{
    destroy();
}

void EventDetectionAgent::destroy()
{
    /*
    if (config().get_bool("ENABLE_ACTION_COLLECT"))
        exportActionConcernedNodesToSCM();
        */

}

// First, we collect all the action corpora from PAI
void EventDetectionAgent::actionCorporaCollect(std::vector<Handle> actionConcernedHandles)
{
    std::vector<Handle>::iterator iter = actionConcernedHandles.begin();
    for (; iter != actionConcernedHandles.end(); iter ++)
    {
        insertNodeToScmMap((Handle)(*iter));
    }
}

void EventDetectionAgent::addAnEvent(Handle eventHandle, unsigned long timestamp, EVENT_TYPE eventType)
{
    map<unsigned long,Frame*>::iterator frameIt = frames.find(timestamp);
    Frame* frame;

    if (frameIt == frames.end())
    {
        // create a new frame
        frame = new Frame();
        frames.insert(map<unsigned long,Frame*>::value_type(timestamp,frame));
    }
    else
        frame = (Frame*)(frameIt->second);

    switch (eventType)
    {
    case EVENT_TYPE_ACTION:
        frame->Actions.push_back(eventHandle);
        break;
    case EVENT_TYPE_STATE_CHANGE:
        frame->StateChanges.push_back(eventHandle);
        break;
    case EVENT_TYPE_SPATIAL_PREDICATE:
        frame->SpatialPredicates.push_back(eventHandle);
        break;

    }
}

void EventDetectionAgent::run()
{
    this->cycleCount = _cogserver.getCycleCount();

    logger().debug( "EventDetectionAgent::%s - Executing run at %d cycles ",
                     __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");
}

bool Skeleton::isSameSkeletonToMe(HandleSeq _rootLinks)
{
    //TODO:
    foreach(Handle rh , _rootLinks)
    {

    }

    return false;
}

// insert a node into allNodesForScmActions
void EventDetectionAgent::insertNodeToScmMap(Handle node)
{
    if( allNodesForScmActions.find(node.value()) ==  allNodesForScmActions.end())
        allNodesForScmActions.insert(map<UUID, Handle>::value_type(node.value(),node));
}

void EventDetectionAgent::exportActionConcernedNodesToSCM()
{
#ifdef HAVE_GUILE
    std::fstream fscm;
    fscm.open(ActionsExportToScmFileName,ios::out);
    std::map<UUID,Handle>::iterator iter = allNodesForScmActions.begin();
    std::string nodeStr;

    for (; iter != allNodesForScmActions.end(); ++iter)
    {
        nodeStr = SchemeSmob::to_string(iter->second);
        fscm << nodeStr << endl;
    }
    fscm.close();
#endif /* HAVE_GUILE */
}

//--------------------------Pattern mining -------------------------------------
void EventDetectionAgent::extractPatternsBySharingSameVaiables(Handle oneLink)
{

}

void EventDetectionAgent::extractSkeleton(Handle oneLink)
{





}

void EventDetectionAgent::processOneInputLink(Handle rootLink)
{

}

void EventDetectionAgent::_processOneInputLink(Handle rootLink,std::map<Handle,Handle> varBindings)
{
    HandleSeq outGoings = atomSpace.getOutgoing(rootLink);

    map<Handle, int>::iterator varit;

    foreach(Handle h , outGoings)
    {
        if (atomSpace.isLink(h))
        {
            processOneInputLink(h);
        }
        else // the current atom is a node
        {
            // check if this node value already in the global var map
            varit = globalVarToInt.find(h);
            if ( varit == globalVarToInt.end())
            {
                // add new variable mapping
                globalVarToInt.insert(map<Handle, int>::value_type(h,++globalVarNum));
            }

            // this variable already exists in the global var map
         // TODO

        }
    }
}

bool EventDetectionAgent::isThisSkeletonAlreadyExisted(Skeleton* s, Skeleton** theExistedSkeleton)
{
 // TODO
    return true;
}

//--------------------------Pattern mining end-------------------------------------
