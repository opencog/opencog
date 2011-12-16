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

#include "EventDetector.h"
#include "opencog/guile/SchemeSmob.h"
#include <iostream>
#include <fstream>

using namespace opencog::pai;
using namespace opencog::control;
using namespace opencog;

EventDetector* EventDetector::instance = NULL;

EventDetector* EventDetector::getInstance()
{
    return EventDetector::instance;
}

EventDetector::EventDetector(PAI& _pai , AtomSpace& _atomSpace):
    pai(_pai),atomSpace(_atomSpace)
{
    // EventDetector is allowed only one in an oac
    if (EventDetector::instance != NULL)
    {
        logger().info("There is already an EventResponder, you cannot create another one!\n");
        return;
    }

    EventDetector::instance = this;

}

EventDetector::~EventDetector()
{

}

void EventDetector::destroy()
{
    exportActionConcernedNodesToSCM();

    if (EventDetector::instance != NULL)
        delete EventDetector::instance;
}

// First, we collect all the action corpra from PAI
void EventDetector::actionCorporaCollect(std::vector<Handle> actionConcernedHandles)
{
    std::vector<Handle>::iterator iter = actionConcernedHandles.begin();
    for (; iter != actionConcernedHandles.end(); iter ++)
    {
        insertNodeToScmMap((Handle)(*iter));
    }
}

// insert a node into allNodesForScmActions
void EventDetector::insertNodeToScmMap(Handle node)
{
    if( allNodesForScmActions.find(node.value()) ==  allNodesForScmActions.end())
        allNodesForScmActions.insert(map<UUID, Handle>::value_type(node.value(),node));
}

void EventDetector::exportActionConcernedNodesToSCM()
{
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
}
