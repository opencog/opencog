/*
 * opencog/embodiment/Control/PerceptionActionInterface/EventResponder.h
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

/** EventResponder.h
 *
 *  Do responses to the external events .
 */

#ifndef EventResponder_H
#define EventResponder_H

#include <vector>
#include <map>
#include <exception>
#include <opencog/atomspace/AtomSpace.h>
#include "PAI.h"

using namespace opencog::control;
using namespace opencog::oac;

namespace opencog { namespace pai {

class EventResponder;

typedef Handle (EventResponder::*ParaFunc_Ptr) (std::string, Handle, Handle, Handle) ;

// define the fuzzy degree expressions
#define EXTREMELY_HIGH "extremely_high"
#define HIGH           "high"
#define MEDIUM         "medium"
#define LOW            "low"
#define EXTREMELY_LOW  "extremely_low"

class EventResponder
{
public:
    static EventResponder* getInstance();
    EventResponder(PAI& _pai , AtomSpace& _atomSpace);
    void destroy();

    void response(std::string actionName, Handle instanceNode, Handle actorNode, Handle targetNode, std::vector<Handle> actionparams, unsigned long timestamp);

private:
    static EventResponder* instance;
    std::map<std::string, ParaFunc_Ptr> paraFuncMap;
    PAI& pai;
    AtomSpace& atomSpace;
    ~EventResponder();

    void ActionParametersprocess(std::string actionName, Handle instanceNode, Handle actorNode, Handle targetNode, std::vector<Handle> actionparams);

    // parameter process functions
    Handle processForce(std::string actionName, Handle actorNode, Handle targetNode, Handle evalLink);


};// class EventResponder

}} // namespace opencog::pai

#endif // EventResponder_H
