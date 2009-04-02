/*
 * opencog/embodiment/Learning/LearningServer/LS.cc
 *
 * Copyright (C) 2007-2008 Nil Geisweiller, Carlos Lopes
 * All Rights Reserved
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
#include "SchemaMessage.h"
#include <sstream>
#include "AtomSpaceWorldProvider.h"
#include "PAIUtils.h"
#include "WorldWrapperUtil.h"
#include <atom_types_init.h>
#include <opencog/util/Config.h>

using namespace LearningServer;
using namespace WorldWrapper;
using namespace opencog;

BaseServer* LS::derivedCreateInstance()
{
    return new LS;
}

/**
 * Constructor and destructor
 */
LS::LS(const std::string &myId, const std::string &ip,
       int portNumber, Control::SystemParameters & parameters)
{
    init(myId, ip, portNumber, parameters);
}

LS::LS()
{
}

LS::~LS()
{
}

void LS::init(const std::string &myId, const std::string &ip,
              int portNumber, Control::SystemParameters & parameters)
{
    setNetworkElement(new NetworkElement(parameters, myId, ip, portNumber));

    // OpenCog-related initialization
    opencog::atom_types_init::init();
    opencog::config().set("MIN_STI",
                          getParameters().get("ATOM_TABLE_LOWER_STI_VALUE"));

    this->busy = false;

    this->registerAgent(ImitationLearningAgent::info().id, &factory);
    ILAgent = static_cast<ImitationLearningAgent*>(this->createAgent(ImitationLearningAgent::info().id, true));
}

bool LS::processNextMessage(MessagingSystem::Message *msg)
{
    LearningServerMessages::LearnMessage  * lm;
    LearningServerMessages::RewardMessage * rm;
    LearningServerMessages::LSCmdMessage  * cm;

    switch (msg->getType()) {

    case MessagingSystem::Message::LS_CMD:
        cm = (LearningServerMessages::LSCmdMessage *)msg;

        if (learningPet == cm->getFrom() &&
                learningSchema == cm->getSchema()) {

            if (cm->getCommand() == getParameters().get("STOP_LEARNING_CMD")) {
                stopLearn();
                return false;
            }

            if (cm->getCommand() == getParameters().get("TRY_SCHEMA_CMD")) {
                trySchema();
                return false;
            }
        }
        break;

    case MessagingSystem::Message::LEARN:
        lm = (LearningServerMessages::LearnMessage *)msg;

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

            logger().log(opencog::Logger::INFO, "LS - Starting new learning: (%s, %s).", learningPet.c_str(), learningSchema.c_str());
            initLearn(lm);
            break;
        }

        // learning in progress, message from the pet currently using LS
        // and the currently trick being learned (that is, it's a new
        // example)
        if (learningPet == lm->getFrom() &&
                learningSchema == lm->getSchema()) {

            logger().log(opencog::Logger::INFO, "LS - Adding example: (%s, %s).", learningPet.c_str(), learningSchema.c_str());

            // TODO verify if commented change has some effect
            //wp = new AtomSpaceWorldProvider(new AtomSpace());
            addLearnExample(lm);
            break;
        }

        // currently the LS do not have a queue of tricks to learn or execute
        // more than one learning process (no concurrency) soh just return
        logger().log(opencog::Logger::WARN, "LS - LS does not support concurent learning (LS busy right now).");
        break;

    case MessagingSystem::Message::REWARD:
        rm = (LearningServerMessages::RewardMessage *)msg;

        if (!isBusy()) {
            logger().log(opencog::Logger::WARN, "LS - LS should be learning when receive a reward message.");
            return false;
        }

        // message from the learning pet and rewarding the learning schema

        if (learningPet == rm->getFrom() &&
            learningSchema == rm->getCandidateSchema()) {
            rewardCandidateSchema(rm);
        }
        break;
    case MessagingSystem::Message::TRY:
        LearningServerMessages::TrySchemaMessage  * tryMsg;
        tryMsg = (LearningServerMessages::TrySchemaMessage  *)msg;

        // TODO: verify if the arguments are the same?
        if (learningPet == tryMsg->getFrom() &&  learningSchema == tryMsg->getSchema()) {
            trySchema();
        }
        break;

    case MessagingSystem::Message::STOP_LEARNING:
        LearningServerMessages::StopLearningMessage  * stopLearningMsg;
        stopLearningMsg = (LearningServerMessages::StopLearningMessage  *)msg;

        // TODO: verify if the arguments are the same?
        if (learningPet == stopLearningMsg->getFrom() &&  learningSchema == stopLearningMsg->getSchema()) {
            stopLearn();
        }
        break;

    default:
        logger().log(opencog::Logger::ERROR, "LS - Unknown message type.");
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
    opencog::cassert(TRACE_INFO, isBusy());
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
    ss << schema;
    logger().log(opencog::Logger::INFO, "LS - Sending schema: (%s, %s, %s).", learningPet.c_str(), schemaName.c_str(), ss.str().c_str());
    LearningServerMessages::SchemaMessage msg(this->getID(), learningPet, schema, schemaName, candidateName);
    sendMessage(msg);
}

const std::string LS::getCandidateSchemaName()
{
    if (learningSchema == "") {
        logger().log(opencog::Logger::WARN, "LS - Trying to get a candidate name from a nameless schema.");
        return ("");
    }

    std::string candidateName;
    candidateName.append(learningSchema);
    candidateName.append("_");
    candidateName.append(opencog::toString(candidateSchemaCnt));

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

void LS::initLearn(LearningServerMessages::LearnMessage * msg)
{
    logger().log(opencog::Logger::DEBUG, "LS - Getting data from LearnMessage (populating atomSpace).");

    bool result = msg->populateAtomSpace(wp->getAtomSpace());
    if (!result) {
        // TODO: do something when fails
        logger().log(opencog::Logger::WARN, "LS - initLearn():  failed to populate AtomSpace.");
    }
    logger().log(opencog::Logger::DEBUG, "LS - Data from LearnMessage gotten.");


    //debug print
    //wp->getAtomSpace()->print();
    //~debug print

    // TODO: use atoms from atomSpace

    // For now the Learning Server only uses hillclimbing - getting definite objects from all maps covering the exemplars

    logger().log(opencog::Logger::DEBUG, "LS - Initiating Learning Process.");

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
        ILAgent->initLearning(atoi(getParameters().get("NUMBER_OF_ESTIMATIONS_PER_CYCLE").c_str()),
                              wp,
                              al,
                              PerceptionActionInterface::PAIUtils::getInternalId(learningPet.c_str()),
                              ownerID,
                              avatarID,
                              learningSchema);
    if (initLearningSucceeds)
        logger().log(opencog::Logger::DEBUG, "LS - Initiating Learning Process - Done.");
    else {
        resetLearningServer();
        logger().log(opencog::Logger::DEBUG, "LS - Initiating Learning Process - Failed.");
    }
}

void LS::addLearnExample(LearningServerMessages::LearnMessage * msg)
{
    logger().log(opencog::Logger::DEBUG, "LS - Getting data from LearnMessage (populating atomSpace).");

    // TODO: do something when resul equals false
    bool result = msg->populateAtomSpace(wp->getAtomSpace());
    if (!result) {
        // TODO: do something when fails
        logger().log(opencog::Logger::WARN, "LS - addLearnExample(): failed to populate AtomSpace.");
    }
    logger().log(opencog::Logger::DEBUG, "LS - Data from LearnMessage gotten.");

    logger().log(opencog::Logger::DEBUG, "LS - Adding exemplar to Learning Process.");
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
    logger().log(opencog::Logger::DEBUG, "LS - Adding exemplar to Learning Process - done.");
}

void LS::rewardCandidateSchema(LearningServerMessages::RewardMessage * msg)
{
    logger().log(opencog::Logger::INFO, "LS - Receive Reward: %f.",
                 msg->getReward());
    // use RewardMessage data to adjust learning algorithm
    double f = msg->getReward();
    ILAgent->setFitness(f);
}

void LS::stopLearn()
{
    logger().log(opencog::Logger::DEBUG, "LS - Stopping learn process.");
    combo::combo_tree bestSchema;

    if (isBusy()) {
        // finish the learning process and get best schema soh far as the
        // learned schema.
        bestSchema = ILAgent->getBestSchema();
        //no need because sendBestSchema reset the task
        //ILAgent->stopLearning();
        stringstream ss;
        ss << bestSchema;
        string s = ss.str();
        logger().log(opencog::Logger::DEBUG,
                     "LS - Send the final learned schema : %s", s.c_str());
        sendBestSchema(bestSchema);

        //delete wp
        delete(wp);
    } else {
        logger().log(opencog::Logger::DEBUG,
                     "LS - Send no schema because LS is not busy");
    }
}

void LS::trySchema()
{
    logger().log(opencog::Logger::DEBUG,
                 "LS - Trying schema");
    combo::combo_tree bestSchema;

    if (isBusy()) {
        // get a candidate schema to execute.
        //the learning algorithm will pause until
        // a reward message is received.
        bestSchema = ILAgent->getBestSchemaEstimated();
        stringstream ss;
        ss << bestSchema;
        string s = ss.str();
        logger().log(opencog::Logger::DEBUG,
                     "LS - Trying the following schema : %s", s.c_str());
        ILAgent->waitForReward();
        sendCandidateSchema(bestSchema);
    } else {
        logger().log(opencog::Logger::DEBUG,
                     "LS - Trying no schema because LS is not busy");
    }
}
