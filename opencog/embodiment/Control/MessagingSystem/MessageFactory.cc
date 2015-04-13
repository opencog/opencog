/*
 * opencog/embodiment/Control/MessagingSystem/MessageFactory.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * Copyright (C) 2012 Linas Vepstas
 * All Rights Reserved
 * Author(s): Andre Senna
 *            Linas Vepstas <linasvepstas@gmail.com>
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


#include <map>
#include <stdio.h>

#include "Message.h"
#include "MessageFactory.h"
#include "MessagingSystemExceptions.h"
#include "StringMessage.h"
#include "RouterMessage.h"

#include <opencog/embodiment/Control/MessagingSystem/TickMessage.h>
#include <opencog/embodiment/Control/MessagingSystem/FeedbackMessage.h>
#include <opencog/embodiment/Control/MessagingSystem/RawMessage.h>

namespace opencog { namespace messaging {
    
using namespace std;

static std::map<int, factory_t> factory_map;


Message *messageFactory(const std::string &from, const std::string &to, int
        msgType, const std::string &msg)
    throw (opencog::InvalidParamException, std::bad_exception)
{
    switch (msgType) {
    case STRING: {
        return new StringMessage(from, to, msg);
        break;
    }
    case TICK: {
        return new TickMessage(from, to);
        break;
    }
    case FEEDBACK: {
        return new FeedbackMessage(from, to, msg);
        break;
    }
    case RAW: {
        return new RawMessage(from, to, msg);
        break;
    }
    default: {

        map<int,factory_t>::iterator fit = factory_map.find(msgType);

        if (fit == factory_map.end()) {
            throw opencog::InvalidParamException(TRACE_INFO,
                     "Message - Unknown message type id: '%d'.", msgType);
        }

        factory_t& fac = fit->second;
        return fac(from, to, msgType, msg);
    }
    }

    // to remove compilation warning, execution should never reach here.
    return NULL;
}

Message *routerMessageFactory(const std::string &from, const std::string &to,
                              int encapsulateMsgType, const std::string &msg)
{
    return new RouterMessage(from, to, encapsulateMsgType, msg);
} 

/// Dynamic run-time registrarion of a new message factory. 
///
/// This allows assorted modules to register new factories for whatever
/// kinds of messages that they want to use, without having to hard-code
/// them into the Mesage class.  The "desired_id" should be set to the 
/// desired numeric ID corresponding to this message.  This is used only
/// for backwards compatibility with old (broken) code that runs on Windows,
/// viz the Unity game engine.  New code should allow the numbers to be 
/// dynamically assigned.
int registerMessageFactory(factory_t fac, int desired_id)
{
    static int next_unissued_msg_id = 100;

    // If the desired ID is not being used, then use it. Otherwise, issue
    // a new ID.
    if (0 != desired_id) {
        map<int,factory_t>::iterator fit = factory_map.find(desired_id);
        if (fit != factory_map.end())
            desired_id = next_unissued_msg_id++;
    }

    factory_map.insert(pair<int, factory_t>(desired_id, fac));

    return desired_id;
}


} } // namespace opencog::messaging

