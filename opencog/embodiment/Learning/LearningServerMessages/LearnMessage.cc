/*
 * opencog/embodiment/Learning/LearningServerMessages/LearnMessage.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * Copyright (C) 2012 Linas Vepstas
 * All Rights Reserved
 * Author(s): Carlos Lopes, Linas Vepstas <linasvepstas@gmail.com>
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

#include <vector>
#include <boost/lexical_cast.hpp>

#include <opencog/util/foreach.h>
#include <opencog/util/StringTokenizer.h>

#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spacetime/SpaceTime.h>
#include <opencog/spacetime/TimeServer.h>
#include <opencog/embodiment/Control/MessagingSystem/MessageFactory.h>

#include "LearnMessage.h"

using namespace opencog::learningserver::messages;
using namespace opencog::messaging;

int LearnMessage::_learnMsgType = init();

static Message* learnFactory(const std::string &from, const std::string &to,
                              int msgType, const std::string &msg)
{
    return new LearnMessage(from, to, msg);
}

int LearnMessage::init()
{
   _learnMsgType = registerMessageFactory((factory_t) learnFactory, LEARN);
   return _learnMsgType;
}

/**
 * Constructor and destructor
 */
LearnMessage::~LearnMessage()
{
}

LearnMessage::LearnMessage(const std::string &from, const std::string &to) :
        Message(from, to, _learnMsgType)
{
    schema.assign("");
    //spaceMap.assign("");
    behaviorDescriptions.assign("");
}

LearnMessage::LearnMessage(const std::string &from, const std::string &to,
                           const std::string &msg) :
        Message(from, to, _learnMsgType)
{

    loadPlainTextRepresentation(msg.c_str());
}

LearnMessage::LearnMessage(const std::string &from, const std::string &to,
                           const std::string &schm,  const std::vector<std::string> &argumentsList,
                           const std::string &owId,
                           const std::string &avId, AtomSpace &atomSpace)
throw (opencog::InvalidParamException, std::bad_exception):
        Message(from, to, _learnMsgType)
{


    // NMXmlExporter exporter(&atomSpace);

    schema.assign(schm);
    ownerId.assign(owId);
    avatarId.assign(avId);
    setSchemaArguments(argumentsList);

    //look for the exemplars time intervals to fill spaceMaps
    Handle trick_h = atomSpace.getHandle(CONCEPT_NODE, schm);
    if (trick_h == Handle::UNDEFINED) {
        throw opencog::InvalidParamException(TRACE_INFO,
                                             "LearnMessage - AtomSpace does not contain '%s' concept node.",
                                             schm.c_str());
    }

    std::list<HandleTemporalPair> htp_seq;
    timeServer().getTimeInfo(back_inserter(htp_seq), trick_h);

#ifdef USE_MAP_HANDLE_SET
    // TODO: THIS DOES NOT WORK BECAUSE MAPS GETS WRONG ORDER (Suggestion: to use a set of timestamps instead)
    // So, for now, we may send duplicate maps, which is inneficient but should not cause any error
    std::set<Handle> mapsToSend;
#endif
    const SpaceServer& spacServer = spaceServer();
    foreach(HandleTemporalPair htp, htp_seq) {
        Temporal t = *htp.getTemporal();
        HandleSeq sm_seq;
        timeServer().getMapHandles(back_inserter(sm_seq), t.getLowerBound(), t.getUpperBound());

        foreach(Handle sm_h, sm_seq) {
            try {
                if (spacServer.containsMap(sm_h)) {
#ifdef USE_MAP_HANDLE_SET
                    mapsToSend.insert(sm_h);
#else
                    logger().debug(
                                 "LearnMessage - adding space map (%s) to the message.",
                                 atomSpace.atomAsString(sm_h).c_str());
                    spaceMaps.push_back(spacServer.mapToString(sm_h));
#endif
                } else {
                    logger().error(
                                 "LearnMessage - SpaceServer has no space map (%s) to be added to the message!",
                                 atomSpace.atomAsString(sm_h).c_str());
                    // TODO: is this really needed? If not, we can get a const
                    // AtomSpace reference instead. Check if this atom will be
                    // removed by forgetting mechanism later anyway
                    atomSpace.removeAtom(sm_h, true);
                }
            } catch (opencog::AssertionException& e) {
                logger().error(
                             "LearnMessage - Failed to add map (%s) into LearnMessage.",
                             atomSpace.atomAsString(sm_h).c_str());
            }
        }
    }

#ifdef USE_MAP_HANDLE_SET
    foreach(Handle sm_h, mapsToSend) {
        logger().debug(
                     "LearnMessage - adding space map (%s) to the message.",
                     atomSpace.atomAsString(sm_h).c_str());
        spaceMaps.push_back(spacServer.MapToString(sm_h));
    }
#endif

    // why are we assigning the entire atomspace?
    HandleSeq hs;
    atomSpace.getHandlesByType(back_inserter(hs), ATOM, true);
    // behaviorDescriptions.assign(exporter.toXML(hs));
    logger().debug("LearnMessage - finished creating message (behavior descriptors just added.");
}

/**
 * Public methods
 */
const char * LearnMessage::getPlainTextRepresentation()
{
    message.assign("");

    //schema id
    message.append(schema);
    message.append(END_TOKEN);
    //ownerId
    message.append(ownerId);
    message.append(END_TOKEN);
    //avatarId to imitate
    message.append(avatarId);
    message.append(END_TOKEN);
    //behavior description
    message.append(behaviorDescriptions);
    message.append(END_TOKEN);
    //schema arguments
    message.append(boost::lexical_cast<std::string>(schemaArguments.size()));
    foreach(std::string s, schemaArguments) {
        message.append(END_TOKEN);
        message.append(s);
    }
    //spaceMaps
    foreach(std::string s, spaceMaps) {
        message.append(END_TOKEN);
        message.append(s);
    }
    return message.c_str();
}

void LearnMessage::loadPlainTextRepresentation(const char *strMessage)
{
    opencog::StringTokenizer stringTokenizer((std::string)strMessage,
            (std::string)END_TOKEN);
    //schema id
    schema = stringTokenizer.nextToken();
    //ownerId
    ownerId = stringTokenizer.nextToken();
    //avatarId to imitate
    avatarId = stringTokenizer.nextToken();
    //behavior description
    behaviorDescriptions = stringTokenizer.nextToken();
    //schema arguments
    schemaArguments.clear();
    unsigned int arity = boost::lexical_cast<unsigned int>(stringTokenizer.nextToken());
    for (unsigned int i = 0; i < arity; i++)
        schemaArguments.push_back(stringTokenizer.nextToken());
    //spaceMaps
    spaceMaps.clear();
    std::string s = stringTokenizer.nextToken();
    while (!s.empty()) {
        spaceMaps.push_back(s);
        s = stringTokenizer.nextToken();
    }
}

/**
 * Getters and setters
 */
void LearnMessage::setSchema(const std::string &schm)
{
    schema.assign(schm);
}

const std::string & LearnMessage::getSchema()
{
    return schema;
}

void LearnMessage::setAvatarId(const std::string &avId)
{
    avatarId.assign(avId);
}

const std::string & LearnMessage::getAvatarId()
{
    return avatarId;
}

void LearnMessage::setOwnerId(const std::string &owId)
{
    ownerId.assign(owId);
}

const std::string & LearnMessage::getOwnerId()
{
    return ownerId;
}

const std::string & LearnMessage::getBehaviorDescriptions()
{
    return behaviorDescriptions;
}

const std::vector<std::string> & LearnMessage::getSpaceMaps()
{
    return spaceMaps;
}

const std::vector<std::string> & LearnMessage::getSchemaArguments()
{
    return schemaArguments;
}

void LearnMessage::setSchemaArguments(const std::vector<std::string> &argumentsList)
{
    schemaArguments = argumentsList;
}


bool LearnMessage::populateAtomSpace(AtomSpace &atomSpace)
{
/*
    // load AtomSpace
    std::vector<XMLBufferReader *> reader(1, new StringXMLBufferReader(behaviorDescriptions.c_str()));
    try {
        //NMXmlParser::loadXML(reader, atomSpace, true, true);
        //NMXmlParser::loadXML(reader, &atomSpace, false, false);

        // load SpaceMap into AtomSpace
        foreach(std::string s, spaceMaps) {
            SpaceServer::TimestampMap timestampMap = SpaceServer::mapFromString(s);
            atomSpace.getSpaceServer().addSpaceMap(timestampMap.first, timestampMap.second);
        }

        // catch all RuntimeExceptions descendents, specially
        // InconsistenceException
    } catch (opencog::RuntimeException& e) {
        delete reader[0];
        return false;
    }

    delete reader[0];*/
    return true;
}

