/*
 * opencog/embodiment/Control/OperationalAvatarController/MessageSender.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#ifndef MESSAGESSENDER_H_
#define MESSAGESSENDER_H_

#include <string>

#include <opencog/atomspace/AtomSpace.h>

namespace opencog { namespace oac {

using namespace opencog;

class MessageSender
{

public:
    virtual ~MessageSender() {}

    /**
     *
     * @return a boolean value that indicates the success (true) or failure (false) of this sending operation.
     */
    virtual bool sendReward(const std::string &schema, const std::vector<std::string> & schemaArguments, const std::string &triedSchema, const double reward) = 0;

    /**
     *
     */
    virtual bool sendExemplar(const std::string &schema, const std::vector<std::string> & schemaArguments, const std::string &ownerId, const std::string &avatarId, AtomSpace& atomSpace) = 0;

    /**
     *
     */
    virtual bool sendCommand(const std::string &command, const std::string &schema) = 0;

    /**
     *
     */
    virtual bool sendFeedback(const std::string &ownerId, const std::string &feedback) = 0;

    /**
     *
     */
    virtual bool sendTrySchema(const std::string &schemaName, const std::vector<std::string> & schemaArgs) = 0;

    /**
     *
     */
    virtual bool sendStopLearning(const std::string &schemaName, const std::vector<std::string> & schemaArgs) = 0;

}; // class

} } // namespace opencog::oac
#endif
