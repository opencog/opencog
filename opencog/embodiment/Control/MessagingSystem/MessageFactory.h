/*
 * opencog/embodiment/Control/MessagingSystem/MessageFactory.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * Copyright (C) 2012 Linas Vepstas
 * All Rights Reserved
 * Author(s): Andre Senna
 *            Linas Vepstas <linasvestas@gmail.com>
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


#ifndef MESSAGE_FACTORY_H
#define MESSAGE_FACTORY_H

#include "Message.h"

namespace opencog { namespace messaging {

    // Message types. The numbering and order of these must not be
    // changed; these are statically assigned in other (non-opencog)
    // code, such as the Unity game engine, and thus must remain
    // numerically compiatible with the list below.
    enum MsgId 
    {
        STRING = 1,
        LEARN  = 2,
        REWARD = 3,
        SCHEMA = 4,
        LS_CMD = 5,
        ROUTER = 6,
        CANDIDATE_SCHEMA = 7,
        TICK = 8,
        FEEDBACK = 9,
        TRY = 10,
        STOP_LEARNING = 11,
        RAW = 12,
    };

    /**
     * Builds Message object of given type
     *
     * @return A new Message of given type
     */
    Message* messageFactory(const std::string &from, const std::string &to,
                int msgType, const std::string &msg)
            throw (opencog::InvalidParamException, std::bad_exception);

    /**
     * Built a message object of RouterMessage type. This method should be
     * called ONLY by Router related classes.
     *
     * @return A new message
     */
    Message* routerMessageFactory(const std::string &from,
            const std::string &to, int encapsulateMsgType, const std::string &msg);


    /**
     * Register a new message factory
     */
    typedef Message* (*factory_t)(const std::string, const std::string, int, const std::string);
    int registerMessageFactory(factory_t, int desired_id = 0);

} } // namespace opencog::messaging

#endif
