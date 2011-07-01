/*
 * opencog/embodiment/Learning/LearningServer/LS.cc
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

#include "LS.h"
#include <opencog/embodiment/Learning/LearningServerMessages/SchemaMessage.h>
#include <sstream>
#include <opencog/embodiment/Learning/LearningServer/AtomSpaceWorldProvider.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAIUtils.h>
#include <opencog/embodiment/WorldWrapper/WorldWrapperUtil.h>
#include <opencog/util/Config.h>

using namespace opencog::learningserver;
using namespace opencog::world;
using namespace opencog;

BaseServer* LS::derivedCreateInstance()
{
    return new LS;
}

/**
 * Constructor and destructor
 */
LS::LS(const std::string &myId, const std::string &ip,
       int portNumber)
{
    init(myId, ip, portNumber);
}

LS::LS()
{
}

LS::~LS()
{
}

void LS::init(const std::string &myId, const std::string &ip,
              int portNumber)
{
    setNetworkElement(new NetworkElement(myId, ip, portNumber));

    this->busy = false;

    this->registerAgent(ImitationLearningAgent::info().id, &factory);
    ILAgent = static_cast<ImitationLearningAgent*>(this->createAgent(ImitationLearningAgent::info().id, true));
}

bool LS::processNextMessage(opencog::messaging::Message *msg)
{
    learningserver::messages::LearnMessage  * lm;
    learningserver::messages::RewardMessage * rm;
    learningserver::messages::LSCmdMessage  * cm;

    switch (msg->getType()) {

    case opencog::messaging::Message::LS_CMD:
        cm = (learningserver::messages::LSCmdMessage *)msg;

        if (learningPet == cm->getFrom() &&
                learningSchema == cm->getSchema()) {

            if (cm->getCommand() == config().get("STOP_LEARNING_CMD")) {
                stopLearn();
                return false;
            }

            if (cm->getCommand() == config().get("TRY_SCHEMA_CMD")) {
                trySchema();
                return false;
            }
        }
        break;

    case opencog::messaging::Message::LEARN:
        lm = (learningserver::messages::LearnMessage *)msg;

        ownerID = lm->getOwnerId();

        avatarID = lm->getAvatarId();

        // no learning in progress... start a new one
        if (!isBusy()) {
            busy = true;

            delete(atomSpace); //it may be null expect perhaps the first time
            atomSpace = new AtomSpace(); //atomSpace is a protective member of
            //BaseServer
            wp = new AtomSpaceWorldProvider(*atomSpace);
            learningPet = lm->getFrom();
            learningSchema = lm->getSchema();

            logger().info("LS - Starting new learning: (%s, %s).", learningPet.c_str(), learningSchema.c_str());
            initLearn(lm);
            break;
        }

        // learning in progress, message from the pet currently using LS
        // and the currently trick being learned (that is, it's a new
        // example)
        if (learningPet == lm->getFrom() &&
                learningSchema == lm->getSchema()) {

            logger().info("LS - Adding example: (%s, %s).", learningPet.c_str(), learningSchema.c_str());

            // TODO verify if commented change has some effect
            //wp = new AtomSpaceWorldProvider(new AtomSpace());
            addLearnExample(lm);
            break;
        }

        // currently the LS do not have a queue of tricks to learn or execute
        // more than one learning process (no concurrency) soh just return
        logger().warn("LS - LS does not support concurent learning (LS busy right now).");
        break;

    case opencog::messaging::Message::REWARD:
        rm = (learningserver::messages::RewardMessage *)msg;

        if (!isBusy()) {
            logger().warn("LS - LS should be learning when receive a reward message.");
            return false;
        }

        // message from the learning pet and rewarding the learning schema

        if (learningPet == rm->getFrom() &&
                learningSchema == rm->getCandidateSchema()) {
            rewardCandidateSchema(rm);
        }
        break;
    case opencog::messaging::Message::TRY:
        learningserver::messages::TrySchemaMessage  * tryMsg;
        tryMsg = (learningserver::messages::TrySchemaMessage  *)msg;

        // TODO: verify if the arguments are the same?
        if (learningPet == tryMsg->getFrom() &&  learningSchema == tryMsg->getSchema()) {
            trySchema();
        }
        break;

    case opencog::messaging::Message::STOP_LEARNING:
        learningserver::messages::StopLearningMessage  * stopLearningMsg;
        stopLearningMsg = (learningserver::messages::StopLearningMessage  *)msg;

        // TODO: verify if the arguments are the same?
        if (learningPet == stopLearningMsg->getFrom() &&  learningSchema == stopLearningMsg->getSchema()) {
            stopLearn();
        }
        break;

    default:
        logger().error("LS - Unknown message type.");
    }
    return false;
}

/**
 * Public methods
 */
bool LS::isBusy()
{
    return busy;
}

void LS::sendCandidateSchema(const combo::combo_tree & schema)
{
    candidateSchemaCnt++;
    sendSchema(schema, learningSchema, getCandidateSchemaName());
}

void LS::sendBestSchema(const combo::combo_tree& schema)
{
    OC_ASSERT(isBusy());
    sendSchema(schema, learningSchema);
    resetLearningServer();
    //inform imitation learning task to stop learning
    ILAgent->stopLearning();
}

/**
 * Private methods
 */
void LS::sendSchema(const combo::combo_tree & schema, std::string schemaName, std::string candidateName)
{
    stringstream ss;
    // TODO reenable the following line
    // ss << schema;
    logger().info("LS - Sending schema: (%s, %s, %s).", learningPet.c_str(), schemaName.c_str(), ss.str().c_str());
    learningserver::messages::SchemaMessage msg(this->getID(), learningPet, schema, schemaName, candidateName);
    sendMessage(msg);
}

const std::string LS::getCandidateSchemaName()
{
    if (learningSchema == "") {
        logger().warn("LS - Trying to get a candidate name from a nameless schema.");
        return ("");
    }

    std::string candidateName;
    candidateName.append(learningSchema);
    candidateName.append("_");
    candidateName.append(toString(candidateSchemaCnt));

    return candidateName;
}

void LS::resetLearningServer()
{
    busy = false;
    candidateSchemaCnt = 0;

    learningPet.assign("");
    learningSchema.assign("");
    ownerID.assign("");
    avatarID.assign("");
}

void LS::initLearn(learningserver::messages::LearnMessage * msg)
{
    logger().debug("LS - Getting data from LearnMessage (populating atomSpace).");

    bool result = msg->populateAtomSpace(wp->getAtomSpace());
    if (!result) {
        // TODO: do something when fails
        logger().warn("LS - initLearn():  failed to populate AtomSpace.");
    }
    logger().debug("LS - Data from LearnMessage gotten.");


    //debug print
    //wp->getAtomSpace()->print();
    //~debug print

    // TODO: use atoms from atomSpace

    // For now the Learning Server only uses hillclimbing - getting definite objects from all maps covering the exemplars

    logger().debug("LS - Initiating Learning Process.");

    combo::argument_list al;
    std::vector<std::string> stral = msg->getSchemaArguments();
    //convert string list in vertex list
    for (std::vector<std::string>::iterator ai = stral.begin();
            ai != stral.end(); ++ai) {
        //since the argument are given as atom name
        //we first convert them to be definite_object
        //here self correspond to avatarID because
        //the pet goes under the skin of the avatar to imitate
        combo::definite_object cdo =
            WorldWrapperUtil::atom_name_to_definite_object(*ai,
                                                           avatarID, ownerID);
        al.push_back(cdo);
    }

    bool initLearningSucceeds =
        ILAgent->initLearning(config().get_int("NUMBER_OF_ESTIMATIONS_PER_CYCLE"),
                              wp,
                              al,
                              opencog::pai::PAIUtils::getInternalId(learningPet.c_str()),
                              ownerID,
                              avatarID,
                              learningSchema);
    if (initLearningSucceeds)
        logger().debug("LS - Initiating Learning Process - Done.");
    else {
        resetLearningServer();
        logger().debug("LS - Initiating Learning Process - Failed.");
    }
}

void LS::addLearnExample(learningserver::messages::LearnMessage * msg)
{
    logger().debug("LS - Getting data from LearnMessage (populating atomSpace).");

    // TODO: do something when resul equals false
    bool result = msg->populateAtomSpace(wp->getAtomSpace());
    if (!result) {
        // TODO: do something when fails
        logger().warn("LS - addLearnExample(): failed to populate AtomSpace.");
    }
    logger().debug("LS - Data from LearnMessage gotten.");

    logger().debug("LS - Adding exemplar to Learning Process.");
    //get world map and atomSpace from message
    //and update learning algorithm environment
    combo::argument_list al;
    std::vector<std::string> stral = msg->getSchemaArguments();
    //convert string list in vertex list
    for (std::vector<std::string>::iterator ai = stral.begin();
            ai != stral.end(); ++ai) {
        //since the argument are given as atom name
        //we first convert them to be definite_object
        //here self correspond to avatarID because
        //the pet goes under the skin of the avatar to imitate
        combo::definite_object cdo =
            WorldWrapperUtil::atom_name_to_definite_object(*ai,
                    avatarID, ownerID);
        al.push_back(cdo);
    }
    ILAgent->addLearningExample(wp, al);
    logger().debug("LS - Adding exemplar to Learning Process - done.");
}

void LS::rewardCandidateSchema(learningserver::messages::RewardMessage * msg)
{
    logger().info("LS - Receive Reward: %f.",
                 msg->getReward());
    // use RewardMessage data to adjust learning algorithm
    double f = msg->getReward();
    ILAgent->setFitness(f);
}

void LS::stopLearn()
{
    logger().debug("LS - Stopping learn process.");
    combo::combo_tree bestSchema;

    if (isBusy()) {
        // finish the learning process and get best schema soh far as the
        // learned schema.
        bestSchema = ILAgent->getBestSchema();
        //no need because sendBestSchema reset the task
        //ILAgent->stopLearning();
        stringstream ss;
        // TODO reenable the following line
        // ss << bestSchema;
        string s = ss.str();
        logger().debug("LS - Send the final learned schema : %s", s.c_str());
        sendBestSchema(bestSchema);

        //delete wp
        delete(wp);
    } else {
        logger().debug(
                     "LS - Send no schema because LS is not busy");
    }
}

void LS::trySchema()
{
    logger().debug(
                 "LS - Trying schema");
    combo::combo_tree bestSchema;

    if (isBusy()) {
        // get a candidate schema to execute.
        //the learning algorithm will pause until
        // a reward message is received.
        bestSchema = ILAgent->getBestSchemaEstimated();
        stringstream ss;
        // TODO reenable the following line
        // ss << bestSchema;
        string s = ss.str();
        logger().debug(
                     "LS - Trying the following schema : %s", s.c_str());
        ILAgent->waitForReward();
        sendCandidateSchema(bestSchema);
    } else {
        logger().debug(
                     "LS - Trying no schema because LS is not busy");
    }
}
