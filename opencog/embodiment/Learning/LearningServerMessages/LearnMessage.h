/*
 * opencog/embodiment/Learning/LearningServerMessages/LearnMessage.h
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

#ifndef LEARNMESSAGE_H_
#define LEARNMESSAGE_H_

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/Control/MessagingSystem/Message.h>

using namespace opencog;

namespace LearningServerMessages
{

class LearnMessage : public opencog::messaging::Message
{

private:
    std::string schema;
    std::string ownerId;
    std::string avatarId;
    std::string behaviorDescriptions;
    std::vector<std::string> schemaArguments;
    std::vector<std::string> spaceMaps;

    // the full message to be sent
    std::string message;

public:

    ~LearnMessage();
    LearnMessage(const std::string &from, const std::string &to);
    LearnMessage(const std::string &from, const std::string &to,
                 const std::string &msg);
    LearnMessage(const std::string &from, const std::string &to,
                 const std::string &schema,
                 const std::vector<std::string> &argumentsList,
                 const std::string &ownerId,
                 const std::string &avatarId, AtomSpace &atomSpace)
    throw (opencog::InvalidParamException, std::bad_exception);

    /**
     * Return A (char *) representation of the message, a c-style string terminated with '\0'.
     * Returned string is a const pointer hence it shaw not be modified and there is no need to
     * free/delete it.
     *
     * @return A (char *) representation of the message, a c-style string terminated with '\0'
     */
    const char *getPlainTextRepresentation();

    /**
     * Factory a message using a c-style (char *) string terminated with `\0`.
     *
     * @param strMessage (char *) representation of the message to be built.
     */
    void loadPlainTextRepresentation(const char *strimessage);

    /**
     * Schema getter and setter
     */
    void setSchema(const std::string &schema);
    const std::string & getSchema();

    /**
     * Owner Id getter and setter
     */
    void setOwnerId(const std::string &ownerId);
    const std::string & getOwnerId();

    /**
     * Avatar Id getter and setter
     */
    void setAvatarId(const std::string &avatarId);
    const std::string & getAvatarId();

    /**
     * Return a string with the NMXml representation of behaviors descriptors.
     *
     * @return string with the NMXml representation of of behaviors descriptors.
     */
    const std::string & getBehaviorDescriptions();

    /**
     * Return a string vector representation of the maps that span over the exemplars from space server.
     *
     * @return a string vector representation of the maps that span over the exemplars from space server.
     */
    const std::vector<std::string> & getSpaceMaps();

    /**
     * Populates the AtomSpace data disseminated within the LearMessage: behavior
     * descriptons and the latest map from space server.
     *
     * @param atomSpace A pointe to the atomSpace to be filled with data
     * from the LearnMessage.
     * @return True if atomSpace was succesfully populated and false
     * otherwise
     */
    bool populateAtomSpace(AtomSpace &atomSpace);

    /**
     * Schema arguments get and set methods.
     */
    const std::vector<std::string> &getSchemaArguments();
    void setSchemaArguments(const std::vector<std::string> &argumentsList);

}; // class
}  // namespace
#endif
