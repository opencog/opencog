/*
 * opencog/embodiment/Learning/LearningServer/LS.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller, Carlos Lopes
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

#ifndef LS_H
#define LS_H

#include <opencog/comboreduct/combo/vertex.h>

#include <opencog/util/Logger.h>

#include <opencog/embodiment/Learning/behavior/WorldProvider.h>
#include <opencog/embodiment/Control/EmbodimentConfig.h>

#include <opencog/embodiment/Control/MessagingSystem/MessageCogServer.h>

#include <opencog/embodiment/Learning/LearningServerMessages/LSCmdMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/LearnMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/RewardMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/TrySchemaMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/StopLearningMessage.h>

#include "ImitationLearningAgent.h"


namespace opencog { namespace learningserver {

using namespace opencog::messaging;

class LS : public MessageCogServer
{

public:

    static opencog::BaseServer* derivedCreateInstance();

    /**
     * Constructor and Destructor
     */
    LS(const std::string &myId, const std::string &ip, int portNumber);
    LS();
    ~LS();

    void init(const std::string &myId, const std::string &ip, int portNumber);

    /**
     * Method inherited from network element
     */
    bool processNextMessage(opencog::messaging::Message *msg);

    /**
     * Informs whenever the LS is busy performing a learning action or not.
     *
     * @return True if LS is busy, false otherwise.
     */
    bool isBusy();

    /**
     * Try a candidate schema of the schema being learned. This schema is
     * send to OAC in order to be evaluated by the pet owner. In order to
     * send a candidate schema, the learning algorithm is pause and waits
     * for a feedback from the user (a reward message)
     *
     * @param schema The combo combo_tree representation of the schema
     */
    void sendCandidateSchema(const combo::combo_tree & schema);

    /**
     * Send the best schema so far to be stored in the OAC procedure repository
     *
     * @param schema The combo combo_tree representation of the schema
     */
    void sendBestSchema(const combo::combo_tree & schema);

private:

    WorldProvider *wp;   // store behavior descriptors and space server
    // with latest map

    bool busy;      // LS busy learning trick?

    std::string learningPet;  // the id of the pet using the LS
    std::string ownerID;                    // the id of the owner of the pet
    std::string avatarID;                   // the id of the avatar to imitate
    std::string learningSchema;             // the trick being learned

    int candidateSchemaCnt;

    //imitation learning task (can plug hillclimbing or MOSES)
    ImitationLearningAgentPtr ILAgent;

    Factory<ImitationLearningAgent, Agent> factory;

    /**
     * Return the candidate schema name according to the acctual candidate
     * schema count.
     *
     * @return A std::string with the candidate schema name
     */
    const std::string getCandidateSchemaName();

    /**
     * Set the learning algorithm environment with data passed within the
     * learn message (schema to learn, world map, example actions)
     *
     * @param msg Learning message with enviroment data
     */
    void initLearn(learningserver::messages::LearnMessage * msg);

    /**
     * Add new data into the learning algorithm enviroment. This data represents
     * new schema examples and new world map data. This data is merged with
     * previous received data.
     *
     * @param msg Learning message with enviroment data
     */
    void addLearnExample(learningserver::messages::LearnMessage * msg);

    /**
     * Insert user feedback into the learning process (via RewardMessage).
     *
     * @param msg A RewardMessage with user feedback about a candidate schema
     */
    void rewardCandidateSchema(learningserver::messages::RewardMessage * msg);

    /**
     * Clear all LS structures used during learning process.
     */
    void resetLearningServer();

    /**
     * Encapsulate the combo combo_tree representation of the learned schema, its
     * name and possible its candidate name (if trying a candidate schema).
     *
     * @param schema A combo combo_tree representation of the schema
     * @param schemaName The name of the schema
     * @param candidateName The name of the candidate schema of the schema
     *       being learned (optional).
     */
    void sendSchema(const combo::combo_tree & schema,
                    std::string schemaName,
                    std::string candidateName = "");

    /**
     * Finishs the learning process, get the best schema so far and send it to
     * the OAC to be added into pet's procedure repository.
     */
    void stopLearn();

    /**
     * Get the best schema so far during learning process and send it to
     * the OAC in order to be evaluated by the owner of the pet. Note: the
     * learning process is paused when a candidate schema is sent to evaluation.
     */
    void trySchema();


}; // class
} } // namespace opencog::learningserver

#endif
