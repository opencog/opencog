/*
 * opencog/embodiment/Control/PerceptionActionInterface/EventDetector.h
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

/** EventDetector.h
 *
 *  Detect events from continueously happening actions .
 */

#ifndef EventDetector_H
#define EventDetector_H

#include <opencog/atomspace/AtomSpace.h>
#include "PAI.h"
#include <map>

using namespace std;

namespace opencog { namespace pai {

#define ActionsExportToScmFileName "ActionsScmCorpus"

class EventDetector
{
public:
    static EventDetector* getInstance();
    EventDetector(PAI& _pai , AtomSpace& _atomSpace);
    void destroy();

    // collect action corpora from PAI
    // the actionConcernedHandles contains all the nodes concerned an action
    void actionCorporaCollect(std::vector<Handle> actionConcernedHandles);
    void exportActionConcernedNodesToSCM();

private:
    static EventDetector* instance;
    std::map<UUID,Handle> allNodesForScmActions;
    PAI& pai;
    AtomSpace& atomSpace;
    ~EventDetector();

    // insert a node into allNodesForScmActions
    void insertNodeToScmMap(Handle node);


};// class EventDetector

}} // namespace opencog::pai


#endif // EventDetector_H
