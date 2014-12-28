/*
 * opencog/embodiment/Control/OperationalAvatarController/Pet.cc
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

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <boost/format.hpp>

#include <opencog/util/files.h>
#include <opencog/util/Logger.h>
#include <opencog/util/mt19937ar.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/TimeServer.h>

#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>

#include <opencog/embodiment/Learning/behavior/BE.h>
#include <opencog/embodiment/Learning/behavior/BDTracker.h>
#include <opencog/embodiment/Learning/behavior/PAIWorldProvider.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h>
#include <opencog/embodiment/WorldWrapper/WorldWrapperUtil.h>

#include "LearningAgentModeHandler.h"
#include "DefaultAgentModeHandler.h"
#include "Pet.h"
#include "OAC.h"


using namespace behavior;
using namespace opencog::world;
using namespace opencog;
using namespace opencog::oac;
using namespace opencog::control;

const unsigned long Pet::UNDEFINED_TIMESTAMP = 0;

/* ------------------------------------
 * Constructor and destructor
 * ------------------------------------
 */
Pet::Pet(const std::string& petId, const std::string& petName, const
        std::string& agentType, const std::string& agentTraits, const
        std::string& ownerId, AtomSpace* atomSpace, MessageSender* sender)
{

    this->pai = NULL;
    this->latestRewardTimestamp = UNDEFINED_TIMESTAMP;
    this->latestPunishmentTimestamp = UNDEFINED_TIMESTAMP;
    this->sender = sender;
    this->atomSpace = atomSpace;

    this->petId.assign(petId);
    this->petName.assign(petName);
    this->visualDebuggerServer = NULL;
    setMode(PLAYING);

    // lower case agent type so that there is no problem with loading files
    this->agentType.assign(agentType);
    std::transform(this->agentType.begin(), this->agentType.end(),
            this->agentType.begin(), ::tolower);

    // lower case pet traits soh that there is no problem with loading files
    this->agentTraits.assign(agentTraits);
    std::transform(this->agentTraits.begin(), this->agentTraits.end(), this->agentTraits.begin(), ::tolower);

    this->ownerId.assign(ownerId);
    this->rayOfVicinity = 7; // TODO: put this in the constructor

    this->triedSchema.assign("");
    this->learningSchema.clear();

    this->grabbedObjId.assign("");

    this->exemplarStartTimestamp = Pet::UNDEFINED_TIMESTAMP;
    this->exemplarEndTimestamp   = Pet::UNDEFINED_TIMESTAMP;

    this->candidateSchemaExecuted = true;

    //initialize the random generator
    unsigned long rand_seed;
    if (config().get_bool("AUTOMATED_SYSTEM_TESTS")) {
        rand_seed = 0;
    } else {
        rand_seed = time(NULL);
    }
    this->rng = new MT19937RandGen(rand_seed);
    logger().info("Pet - Created random number generator (%p) for Pet with "
            "seed %lu", this->rng, rand_seed);

    this->modeHandler[ LEARNING ] = new LearningAgentModeHandler(this);
    this->modeHandler[ PLAYING ] = new DefaultAgentModeHandler(this);
}

Pet::~Pet()
{
    delete(rng);
    std::map<PetMode, AgentModeHandler*>::iterator it;
    for ( it = this->modeHandler.begin( );
            it != this->modeHandler.end( ); ++it ) {
        AgentModeHandler* handler = it->second;
        it->second = 0;
        delete handler;
    } // for

    stopVisualDebuggerServer( );
}

void Pet::initTraitsAndFeelings()
{
    //make sure there is a node for the pet and owner
    Handle petHandle;
    AtomSpaceUtil::addNode(*atomSpace, AVATAR_NODE, getOwnerId(), true);
    if ( this->agentType == "pet" ) {
        petHandle = AtomSpaceUtil::addNode(*atomSpace, PET_NODE,
                getPetId(), true);
    } else if ( this->agentType == "humanoid" ) {
        petHandle = AtomSpaceUtil::addNode(*atomSpace, HUMANOID_NODE,
                getPetId(), true);
    }

    // feelings
    TruthValuePtr tv = SimpleTruthValue::createTV(0.5f, 0.0f);
    atomSpace->setLTI(AtomSpaceUtil::setPredicateValue(*atomSpace,
                std::string("fear"), tv, petHandle), 1);
    atomSpace->setLTI(AtomSpaceUtil::setPredicateValue(*atomSpace,
                std::string("pride"), tv, petHandle), 1);
    atomSpace->setLTI(AtomSpaceUtil::setPredicateValue(*atomSpace,
                std::string("love"), tv, petHandle), 1);
    atomSpace->setLTI(AtomSpaceUtil::setPredicateValue(*atomSpace,
                std::string("hate"), tv, petHandle), 1);
    atomSpace->setLTI(AtomSpaceUtil::setPredicateValue(*atomSpace,
                std::string("anger"), tv, petHandle), 1);
    atomSpace->setLTI(AtomSpaceUtil::setPredicateValue(*atomSpace,
                std::string("gratitude"), tv, petHandle), 1);

    tv = SimpleTruthValue::createTV(0.51f, 0.0f);
    atomSpace->setLTI(AtomSpaceUtil::setPredicateValue(*atomSpace,
                std::string("happiness"), tv, petHandle), 1);
    atomSpace->setLTI(AtomSpaceUtil::setPredicateValue(*atomSpace,
                std::string("sadness"), tv, petHandle), 1);
    atomSpace->setLTI(AtomSpaceUtil::setPredicateValue(*atomSpace,
                std::string("excitement"), tv, petHandle), 1);
}

void Pet::setPAI(PAI *pai)
{
    this->pai = pai;
}


/* ------------------------------------
 * Acessor Methods
 * ------------------------------------
 */
const std::string& Pet::getName() const
{
    return (this->petName);
}

void Pet::setName(const std::string& petName)
{
    this->petName = petName;
}

const std::string& Pet::getPetId() const
{
    return (this->petId);
}

const std::string& Pet::getType() const
{
    return (this->agentType);
}

const std::string& Pet::getOwnerId() const
{
    return (this->ownerId);
}

const std::string & Pet::getDemandName() const
{
    return this->demandName; 
}

void Pet::setDemandName(const std::string & demandName) 
{
    this->demandName = demandName;
}

const std::string& Pet::getExemplarAvatarId() const
{
    return (this->exemplarAvatarId);
}

void Pet::setOwnerId(const std::string& ownerId)
{
    this->ownerId.assign(ownerId);
}

void Pet::adjustIsExemplarAvatarPredicate(bool active) throw (RuntimeException)
{

    if (this->exemplarAvatarId != "") {
        std::vector<Handle> exemplarAvatarSet;
        atomSpace->getHandlesByName(back_inserter(exemplarAvatarSet),
                this->exemplarAvatarId, OBJECT_NODE, true);

        if (exemplarAvatarSet.size() != 1) {
            throw RuntimeException(TRACE_INFO,
                    "Pet - Found '%d' node(s) with name '%s'. "
                    "Expected exactly one node.",
                    exemplarAvatarSet.size(), this->exemplarAvatarId.c_str());
        }

        TruthValuePtr tv = SimpleTruthValue::createTV(0.0, 1.0);
        if (active) {
            tv = SimpleTruthValue::createTV(1.0, 1.0);
        }

        Handle atTimeLink = AtomSpaceUtil::addPropertyPredicate(*atomSpace,
                "is_exemplar_avatar", exemplarAvatarSet[0], getMyHandle(), tv,
                Temporal(pai->getLatestSimWorldTimestamp()));
        AtomSpaceUtil::updateLatestIsExemplarAvatar(*atomSpace, atTimeLink);
    }
}

void Pet::setExemplarAvatarId(const std::string& exemplarAvatarId)
{
    adjustIsExemplarAvatarPredicate(false);
    this->exemplarAvatarId.assign(exemplarAvatarId);
    adjustIsExemplarAvatarPredicate(true);
}

const PetMode Pet::getMode() const
{
    return (this->mode);
}

void Pet::setMode(PetMode  mode)
{
    this->mode = mode;

    std::string feedback = "";
    switch (this->mode) {

    case LEARNING: {
        logger().info(
                 "Pet - '%s' entering LEARNING mode. Trick: '%s', exemplar"
                 "avatar: '%s'.",
                 this->petName.c_str(),
                 learningSchema.empty() ? "" : learningSchema.front().c_str(),
                 exemplarAvatarId.c_str());

        feedback.append(petName);
        feedback.append(" entering \"Learning Mode\"");

    }
    break;

    case PLAYING: {
        logger().info("Pet - '%s' entering PLAYING mode.",
                this->petName.c_str());

        // remove previous info realated to  exemplar avatar id,
        // learning schema and tried schema
        setExemplarAvatarId(std::string(""));
        learningSchema.clear();
        triedSchema.assign("");

        // exemplar times also should be reset
        exemplarStartTimestamp = Pet::UNDEFINED_TIMESTAMP;
        exemplarEndTimestamp  = Pet::UNDEFINED_TIMESTAMP;

        feedback.append(petName);
        feedback.append(" entering \"Playing Mode\"");

    }
    break;

    default:
        break;
    }

    // sending feedback (only if petName is already known -- this prevents
    // unnnecessary and bad feedback message at the startup)
    if (petName != config().get("UNKNOWN_PET_NAME")) { 
        logger().info("Pet - setMode - PetId '%s' sending feedback '%s'.",
                this->petId.c_str(), feedback.c_str());
        sender->sendFeedback(petId, feedback);
    }
}

const std::vector<std::string>& Pet::getLearningSchema()
{
    return (this->learningSchema);
}

const std::string& Pet::getTriedSchema()
{
    return (this->triedSchema);
}

void Pet::setTriedSchema(const std::string & triedSchema)
{
    this->triedSchema = triedSchema;
    this->candidateSchemaExecuted = false;
}

void Pet::schemaSelectedToExecute(const std::string & schemaName)
{
    logger().debug("Pet - schemaSelectedToExecute(%s)" , schemaName.c_str());
    if (this->triedSchema == schemaName) {
        this->candidateSchemaExecuted = true;

        std::string feedback;
        feedback.append(petName);
        feedback.append(" trying schema \" ");
        feedback.append(schemaName);
        feedback.append("\"");

        sender->sendFeedback(petId, feedback);

    } else {
        logger().debug("Pet - schemaSelectedToExecute: schemaName (%s) is"
                "different from triedSchema (%s)", schemaName.c_str(),
                triedSchema.c_str()); }
}

unsigned long Pet::getExemplarStartTimestamp()
{
    return (this->exemplarStartTimestamp);
}

unsigned long Pet::getExemplarEndTimestamp()
{
    return (this->exemplarEndTimestamp);
}

/* ------------------------------------
 * Public Methods
 * ------------------------------------
 */
Pet* Pet::importFromFile(const std::string& filename, const std::string& petId,
        AtomSpace* atomSpace, MessageSender* sender)
{
    Pet* pet;
    unsigned int petMode;
    std::string ownerId;
    std::string petName;
    std::string agentType;
    std::string agentTraits;

    std::ifstream petFile(filename.c_str());
    petFile.exceptions(std::ifstream::eofbit | std::ifstream::failbit | std::ifstream::badbit);
    try {
        std::getline(petFile, petName);
        std::getline(petFile, agentType);
        std::getline(petFile, agentTraits);
        std::getline(petFile, ownerId);
        petFile >> petMode;
    } catch (std::ifstream::failure e) {
        logger().error("Pet - Unable to load pet metadata.");
        return NULL;
    }
    petFile.close();

    pet = new Pet(petId, petName, agentType, agentTraits, ownerId, atomSpace,
            sender);
    pet->setMode((PetMode)petMode);

    return pet;
}

void Pet::exportToFile(const std::string& filename, Pet & pet) throw
    (IOException, std::bad_exception)
{
    // remove previous saved dumps
    remove(filename.c_str());

    std::ofstream petFile(filename.c_str());
    petFile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    try {
        petFile << pet.getName()    << endl;
        petFile << pet.getType()    << endl;
        petFile << pet.getTraits()  << endl;
        petFile << pet.getOwnerId() << endl;
        petFile << pet.getMode()    << endl;
    } catch (std::ofstream::failure e) {
        petFile.close();
        throw IOException(TRACE_INFO, "Pet - Unable to save pet metadata.");
    }

    petFile.close();
}

/* ---------------------------------------------------
 * IMPLEMENTATION OF METHODS OF AvatarInterface
 * - getPetId() is already defined above
 * ---------------------------------------------------
 */
AtomSpace& Pet::getAtomSpace()
{
    return *atomSpace;
}

void Pet::stopExecuting(const std::vector<std::string> &commandStatement,
        unsigned long timestamp)
{
    logger().debug("Pet - Stop executing '%s' at %lu.",
                 commandStatement.front().c_str(), timestamp);
    // TODO:
    //  Cancel a Pet command instruction that was given before:
    // * If it is not running yet: decrease the importance of the corresponding
    // GoalSchemaImplicationLink
    // * If it is alreday running: try to abort the execution of the
    // corresponding GroundedSchema
}

bool Pet::isInLearningMode() const
{
    return mode == LEARNING;
}

void Pet::startLearning(const std::vector<std::string> &commandStatement,
        unsigned long timestamp)
{
    logger().debug("Pet - Start learning '%s' trick at %lu with '%s'", commandStatement.front().c_str(), timestamp, getExemplarAvatarId().c_str());

    if (isInLearningMode()) {
        logger().warn("Pet - Already in LEARNING mode. Canceling learning "
                "to '%s' with '%s'.", learningSchema.front().c_str(),
                exemplarAvatarId.c_str());
        std::string newExemplarAvatarId = exemplarAvatarId;
        stopLearning(learningSchema, timestamp);
        setExemplarAvatarId(newExemplarAvatarId);
        // TODO: check if this timestamp is ok
    }

    // change the Pet to learning mode
    learningSchema = commandStatement;

    startLearningSessionTimestamp = timestamp;
    setMode(LEARNING);

    // TODO: Perhaps the "PayAttention" stuff should be here, instead of in PredaveseActions.cc
}

void Pet::stopLearning(const std::vector<std::string> &commandStatement,
        unsigned long timestamp)
{
    logger().debug("Pet - Stop learning '%s' trick at %lu.",
                 commandStatement.front().c_str(), timestamp);

    // reset all exemplar timestamps to avoid storing more maps than necessary
    exemplarStartTimestamp = Pet::UNDEFINED_TIMESTAMP;
    exemplarEndTimestamp  = Pet::UNDEFINED_TIMESTAMP;

    // check if stop learning corresponds to currently learning schema
    if (learningSchema != commandStatement) {
        logger().warn("Pet - Stop learn, trick command statement registered in"
                "learning is different from trick command statement provided.");
        // TODO: Send a feedback message to the user about this problem so that
        // he/she enter the right command
        return;
    }

    endLearningSessionTimestamp = timestamp;
    Temporal learningTimeInterval(startLearningSessionTimestamp,
            endLearningSessionTimestamp);
    Handle trickConceptNode = AtomSpaceUtil::addNode(*atomSpace, CONCEPT_NODE,
            learningSchema.front());
    Handle atTimeLink = timeServer().addTimeInfo(trickConceptNode,
            learningTimeInterval);
    // TODO: check if the updateLatest bellow is really needed
    //AtomSpaceUtil::updateLatestLearningSession(atomSpace, atTimeLink);

    Handle learningConceptNode = atomSpace->addNode(CONCEPT_NODE, "trick");
    HandleSeq inhLinkHS;
    inhLinkHS.push_back(trickConceptNode);
    inhLinkHS.push_back(learningConceptNode);
    atomSpace->addLink(INHERITANCE_LINK, inhLinkHS);

    //sender->sendCommand(config().get("STOP_LEARNING_CMD"),
    //      commandStatement.front());
    std::vector<std::string> args;
    std::vector<std::string>::iterator it = learningSchema.begin();
    ++it;
    while (it != learningSchema.end()) {
        args.push_back(*it);
        ++it;
    }

//  std::copy(learningSchema.begin()+1, learningSchema.end(), args.begin());
    sender->sendStopLearning(learningSchema.front(), args);

    // NOTE: Pet will return to playing mode only when the learned schema is
    // stored in ProcedureRepository
    // FOR NOW, put it in playing mode, so that OAC does not stay in Learning
    // state forever if LS crashes or become unavailable for any reason ...
    // (later, we could create a intermediate state/mode that implements a
    // timeout waiting LS meSsage.
    setMode(PLAYING);
}

bool Pet::isExemplarInProgress() const
{
    return (isInLearningMode() &&
            exemplarStartTimestamp != Pet::UNDEFINED_TIMESTAMP &&
            exemplarEndTimestamp == Pet::UNDEFINED_TIMESTAMP);
}

void Pet::startExemplar(const std::vector<std::string> &commandStatement,
        unsigned long timestamp)
{
    logger().debug("Pet - Exemplars for '%s' trick started at %lu with '%s'.",
                 (commandStatement.size() > 0 ? commandStatement.front().c_str()
                  : learningSchema.front().c_str()), timestamp,
                 getExemplarAvatarId().c_str());

    if (!isInLearningMode()) {
        logger().warn("Pet - Unable to start exemplar. Not in LEARNING mode.");
        return;
    }

    if (learningSchema != commandStatement && commandStatement.size() > 0) {
        logger().warn("Pet - Start exemplar, trick command statement "
                "registered in learning is different from trick command "
                "statement provided.");
        // TODO: Send a feedback message to the user about this problem so that
        // he/she enter the right command
        return;
    }

    this->candidateSchemaExecuted = true;
    exemplarStartTimestamp = timestamp;
}

void Pet::endExemplar(const std::vector<std::string> &commandStatement, unsigned
        long timestamp)
{
    logger().debug("Pet - Exemplars for '%s' trick ended at %lu.",
                 (commandStatement.size() > 0 ? commandStatement.front().c_str()
                  : learningSchema.front().c_str()), timestamp);

    if (!isInLearningMode() ||
            exemplarStartTimestamp == Pet::UNDEFINED_TIMESTAMP) {
        logger().warn("Pet - Unable to end exemplar. Not in LEARNING mode or "
                "StartExemplar message not received.");
        // TODO: Send a feedback message to the user about this problem so that
        // he/she enter the right command
        return;
    }

    if (learningSchema != commandStatement && commandStatement.size() > 0) {
        logger().warn("Pet - End exemplar, trick command statement registered "
                "in learning is different from trick command statement "
                "provided.");
        // TODO: Send a feedback message to the user about this problem so that
        // he/she enter the right command
        return;
    }

    exemplarEndTimestamp = timestamp;

    // Behavior enconder and persistence of relevant SpaceMaps
    executeBehaviorEncoder();
    updatePersistentSpaceMaps();

    // send the whole atomSpace to the LS
    std::vector<std::string> args;

    std::vector<std::string>::iterator it = learningSchema.begin();
    ++it;
    while (it != learningSchema.end()) {
        args.push_back(*it);
        ++it;
    }

    for (std::vector<std::string>::iterator it = args.begin();
            it != args.end(); ++it)
        logger().debug(" args: %s", (*it).c_str());
    sender->sendExemplar(learningSchema.front(), args, ownerId,
            exemplarAvatarId, *atomSpace);

    // after sending LearnMessage
    exemplarStartTimestamp = Pet::UNDEFINED_TIMESTAMP;
    exemplarEndTimestamp  = Pet::UNDEFINED_TIMESTAMP;
}

//static WorldProvider* haxxWorldProvider = (WorldProvider*)NULL;

void Pet::executeBehaviorEncoder()
{

    //if (!haxxWorldProvider)
    //haxxWorldProvider = new PAIWorldProvider(atomSpace);

    // Define the behavior interval
    Temporal exemplarTimeInterval(exemplarStartTimestamp, exemplarEndTimestamp);

    //debug print
    //std::cout << "EXECUTE BEHAVIOR ENCODER EXEMPLAR TIME INTERVAL : " << exemplarTimeInterval << std::endl;
    //~debug print

    // TODO: the command parameters:
    // (commandStatement[1], commandStatement[2], ...)  are beging ignored
    Handle trickConceptNode = atomSpace->addNode(CONCEPT_NODE,
            learningSchema.front());
    Handle trickExemplarAtTimeLink = timeServer().addTimeInfo(trickConceptNode,
            exemplarTimeInterval);
    // TODO: check if the updateLatest bellow is really needed
    //AtomSpaceUtil::updateLatestTrickExemplar(atomSpace,
    //  trickExemplarAtTimeLink);

    //BehaviorEncoder encoder(haxxWorldProvider, trickExemplarAtTimeLink, 1);
    BehaviorEncoder encoder(new PAIWorldProvider(this->pai), petId,
            trickExemplarAtTimeLink, 1);

    // Adds the inheritance link as Ari asked
    Handle exemplarConceptNode = atomSpace->addNode(CONCEPT_NODE, "exemplar");
    HandleSeq inhLinkHS;
    inhLinkHS.push_back(trickConceptNode);
    inhLinkHS.push_back(exemplarConceptNode);
    atomSpace->addLink(INHERITANCE_LINK, inhLinkHS);

    //Note from Nil : I comment the POSITION tracker for now because
    //hillclimbing does not deal with positions anyway
    // position tracker
    /*
      atom_tree *positionTemplate =
          makeVirtualAtom(EVALUATION_LINK,
              makeVirtualAtom(atomSpace->addNode(PREDICATE_NODE, AGISIM_POSITION_PREDICATE_NAME), NULL),
              makeVirtualAtom(LIST_LINK,
                  makeVirtualAtom(AVATAR_NODE, NULL),
                  NULL
              ),
              NULL
          );
      encoder.addBETracker(*positionTemplate, new MovementBDTracker (atomSpace));
    */

    // action tracker
    atom_tree *actionTemplate =
        makeVirtualAtom(EVALUATION_LINK,
            makeVirtualAtom(atomSpace->addNode(PREDICATE_NODE,
                    ACTION_DONE_PREDICATE_NAME), NULL),
            makeVirtualAtom(LIST_LINK,
                    makeVirtualAtom(
                        atomSpace->addNode(AVATAR_NODE, exemplarAvatarId),
                        NULL),
                    NULL),
            NULL);

    encoder.addBETracker(*actionTemplate, new ActionBDTracker(atomSpace));

    // TODO: Use the exemplarEndTimestamp as well -- the current BehaviorEncoder
    // considers the "NOW" as end of the examplar interval.
    Temporal startTime(exemplarStartTimestamp);
    encoder.tempUpdateRec(exemplarTimeInterval);

    //debug print
    //std::cout << "PRINT ATOMSPACE :" << std::endl;
    //atomSpace->print();
    //~debug print

}

void Pet::trySchema(const std::vector<std::string> &commandStatement, unsigned
        long timestamp)
{
    logger().debug("Pet - Try '%s' trick at %lu.",
             (commandStatement.size() > 0 ? commandStatement.front().c_str() :
              learningSchema.front().c_str()), timestamp);

    if (learningSchema != commandStatement && commandStatement.size() > 0) {
        logger().warn("Pet - Try schema, trick differs");
        return;
    }

    if (this->candidateSchemaExecuted) {
        //sender->sendCommand(config().get("TRY_SCHEMA_CMD"),
        //learningSchema.front().c_str());
        std::vector<std::string> args;

        std::vector<std::string>::iterator it = learningSchema.begin();
        ++it;
        while (it != learningSchema.end()) {
            args.push_back(*it);
            ++it;
        }

//    std::copy(learningSchema.begin()+1, learningSchema.end(), args.begin());
        sender->sendTrySchema(learningSchema.front(),  args);
    } else {
        logger().warn("Pet - Did not executed the last received candidate yet!");
        // Force a new attempt of executing the candidate schema.
//        ruleEngine->tryExecuteSchema(learningSchema.front());
    }
}

void Pet::reward(unsigned long timestamp)
{
    logger().debug("Pet - Reward at %lu.",  timestamp);
    this->latestRewardTimestamp = timestamp;

    if (isInLearningMode()) {
        if (learningSchema.empty() ||  learningSchema.front() == "" ||
                triedSchema == "") {
            logger().warn("Pet - Trying to reward a non-tried schema.");
            // TODO: Send a feedback message to the user about this problem so
            // that he/she enter the right command
            return;
        }
        this->candidateSchemaExecuted = true;
        // TODO: the command parameters
        // (commandStatement[1], commandStatement[2], ...)
        // are beging ignored
        std::vector<std::string> args;
        std::vector<std::string>::iterator it = learningSchema.begin();
        ++it;
        while (it != learningSchema.end()) {
            args.push_back(*it);
            ++it;
        }


//      std::copy(learningSchema.begin()+1, learningSchema.end(), args.begin());
        sender->sendReward(learningSchema.front(), args, triedSchema, config().get_double("POSITIVE_REWARD"));

    } else {
        // call rule engine to reward implication links for latest selected
        // rules
//        ruleEngine->rewardRule(timestamp);
    }
}

void Pet::punish(unsigned long timestamp)
{
    logger().debug("Pet - Punishment at %lu.",  timestamp);
    this->latestPunishmentTimestamp = timestamp;

    if (isInLearningMode()) {
        if (learningSchema.empty() || learningSchema.front() == "" ||
                triedSchema == "") {
            logger().warn("Pet - Trying to punish a non-tried schema.");
            // TODO: Send a feedback message to the user about this problem so
            // that he/she enter the right command
            return;
        }
        this->candidateSchemaExecuted = true;
        // TODO: the command parameters
        // (commandStatement[1], commandStatement[2], ...)
        // are beging ignored
        std::vector<std::string> args;
        std::vector<std::string>::iterator it = learningSchema.begin();
        ++it;
        while (it != learningSchema.end()) {
            args.push_back(*it);
            ++it;
        }

//      std::copy(learningSchema.begin()+1, learningSchema.end(), args.begin());
        sender->sendReward(learningSchema.front(), args, triedSchema, config().get_double("NEGATIVE_REWARD"));

    } else {
        // call rule engine to punish implication links for latest selected rules
//        ruleEngine->punishRule(timestamp);
    }
}

AgentModeHandler& Pet::getCurrentModeHandler( void )
{
    return *this->modeHandler[ this->mode ];
}

float Pet::computeWalkingSpeed() const
{
    float speed =  config().get_double("PET_WALKING_SPEED");
    if (speed <= 0) {
        // get a random speed to be used (just for tests)
        speed = 0.5 + 3.0 * rng->randfloat();
    }
    return speed;
}

const std::string& Pet::getTraits( void ) const
{
    return this->agentTraits;
}

unsigned long Pet::getLatestRewardTimestamp( void )
{
    return this->latestRewardTimestamp;
}

unsigned long Pet::getLatestPunishmentTimestamp( void )
{
    return this->latestPunishmentTimestamp;
}


void Pet::setGrabbedObj(const string& id)
{
    if ( grabbedObjId == id ) {
        logger().debug(
             "Pet - Pet is already holding '%s', ignoring...",
             grabbedObjId.c_str() );
        return;
    } // if

    grabbedObjId = id;
}

const std::string& Pet::getGrabbedObj()
{
    return grabbedObjId;
}

bool Pet::hasGrabbedObj()
{
    // true if has a id, i.e. not empty
    return (!grabbedObjId.empty());
}

void Pet::updatePersistentSpaceMaps() throw (RuntimeException, std::bad_exception)
{

    // sanity checks
    if (exemplarStartTimestamp == Pet::UNDEFINED_TIMESTAMP ||
            exemplarEndTimestamp   == Pet::UNDEFINED_TIMESTAMP) {
        logger().warn(
             "Pet - Exemplar start/end should be set to update "
             "SpaceMapsToHold.");
        return;
    }

    // sanity checks
    if (exemplarStartTimestamp > exemplarEndTimestamp) {
        logger().warn(
             "Pet - Exemplar start should be smaller than exemplar end.");
        return;
    }
/*
    Handle spaceMapNode = atomSpace->getSpaceServer().getLatestMapHandle();

    // getting all HandleTemporalPairs associated with the SpaceMap
    // concept node within the exemplar timestamped sections
    std::vector<HandleTemporalPair> pairs;
    atomSpace->getTimeServer().getTimeInfo(back_inserter(pairs), spaceMapNode,
           Temporal(exemplarStartTimestamp, exemplarEndTimestamp),
           TemporalTable::STARTS_WITHIN);

    logger().fine("Pet - %d candidate maps to be checked.", pairs.size());

    foreach(HandleTemporalPair pair, pairs) {
        // mark any still existing spaceMap in this period as persistent
        Handle mapHandle = atomSpace->getTimeServer().getAtTimeLink(pair);
        if (atomSpace->getSpaceServer().containsMap(mapHandle)) {
            logger().debug("Pet - Marking map (%s) as persistent.",
                         atomSpace->atomAsString(mapHandle).c_str());
            atomSpace->getSpaceServer().markMapAsPersistent(mapHandle);
        } else {
            // TODO: This should not be needed here. Remove it when a solution
            // for that is implemented.
            logger().debug("Pet - Removing map handle (%s) from AtomSpace. "
                    "Map already removed from SpaceServer.",
                     atomSpace->atomAsString(mapHandle).c_str());
            atomSpace->removeAtom(mapHandle, true);
        }
    }
    */
}
bool Pet::isNear(const Handle& objectHandle)
{
    return AtomSpaceUtil::isPredicateTrue(*atomSpace, "is_near", objectHandle,
            getMyHandle() );
}

Handle Pet::getMyHandle() const
{
    Handle h = atomSpace->getHandle( PET_NODE, petId );
    if ( h == Handle::UNDEFINED ) {
        h = atomSpace->getHandle( HUMANOID_NODE, petId );
    } // if
    return h;
}

/*
bool Pet::getVicinityAtTime(unsigned long timestamp, HandleSeq& petVicinity)
{
    vector<string> entitiesInVicinity;
    Handle spaceMapHandle =
        AtomSpaceUtil::getSpaceMapHandleAtTimestamp(*atomSpace, timestamp);

    if (spaceMapHandle != Handle::UNDEFINED) {
        const SpaceServer::SpaceMap& spaceMap =
            atomSpace->getSpaceServer().getMap(spaceMapHandle);
        spatial::Point petLoc = WorldWrapperUtil::getLocation(spaceMap,
                *atomSpace, this->petId);
        spaceMap.findEntities( spaceMap.snap(petLoc), rayOfVicinity,
                back_inserter(entitiesInVicinity) );
    }

    // get the handle for each entity
    foreach(string entity, entitiesInVicinity) {
        HandleSeq objHandle;
        atomSpace->getHandlesByName(back_inserter(objHandle), entity, OBJECT_NODE, true);
        if (objHandle.size() == 1) {
            petVicinity.push_back(*objHandle.begin());
        } else {
            logger().error("Could not find handle of object with id \"%s\".",
                    entity.c_str() );
            petVicinity.clear();
            return false;
        }
    }
    return true;
}
*/
void Pet::getHighLTIObjects(HandleSeq& highLTIObjects)
{
    atomSpace->getHandlesByType(back_inserter(highLTIObjects), OBJECT_NODE, true);

    HandleSeq::iterator it = highLTIObjects.begin();
    while (it != highLTIObjects.end()) {
        AttentionValuePtr attentionValue = atomSpace->getAV(*it);
        if (attentionValue->getLTI() < AtomSpaceUtil::highLongTermImportance)
            it = highLTIObjects.erase(it);
        else ++it;
    }
}


void Pet::getAllObservedActionsDoneAtTime(const Temporal& time, HandleSeq& actionsDone)
{
    logger().debug("Pet::getAllActionsDoneObservedAtTime");

    std::vector<HandleTemporalPair> everyEventThatHappened;
    timeServer().getTimeInfo(back_inserter(everyEventThatHappened),
            Handle::UNDEFINED, time, TemporalTable::OVERLAPS);
    foreach(HandleTemporalPair event, everyEventThatHappened) {
        Handle eventAtTime = timeServer().getAtTimeLink(event);
        if (atomSpace->getArity(eventAtTime) >= 2) {
            Handle evaluationLink = atomSpace->getOutgoing(eventAtTime, 1);
            if (atomSpace->getType(evaluationLink) == EVALUATION_LINK) {
                if (atomSpace->getName(
                            atomSpace->getOutgoing(evaluationLink, 0))
                        == ACTION_DONE_PREDICATE_NAME) {
                    actionsDone.push_back(evaluationLink);
                }
            }
        }
    }
}

void Pet::getAllActionsDoneInATrickAtTime(const Temporal& time, HandleSeq& actionsDone)
{
    logger().debug("Pet::getAllActionsDoneInATrickAtTime");

    HandleSeq patternToSearchLearningSession;
    Handle conceptNode = atomSpace->getHandle(CONCEPT_NODE, "learningSession");
    if (conceptNode != Handle::UNDEFINED) {
        patternToSearchLearningSession.push_back(conceptNode);
        // get the handles of all trick
        HandleSeq learningSessionHandles;
        atomSpace->getHandlesByOutgoing(back_inserter(learningSessionHandles),
                patternToSearchLearningSession, NULL, NULL, 2, INHERITANCE_LINK,
                true);
        foreach(Handle learningSessionHandle, learningSessionHandles) {
            if (atomSpace->getArity(learningSessionHandle) > 1)  {
                std::vector<HandleTemporalPair> learningSessionIntervals;
                // Get temporal info for all the Handles that pertain to this
                // trick
                timeServer().getTimeInfo(back_inserter(learningSessionIntervals),
                        learningSessionHandle, time, TemporalTable::OVERLAPS);
                // get all action that occurred during each interval
                foreach(HandleTemporalPair learningSessionInterval,
                        learningSessionIntervals) {
                    Temporal *learningSessionIntervalTemporal =
                        learningSessionInterval.getTemporal();
                    std::vector<HandleTemporalPair> actionsInLearningSession;
                    Handle learningSessionIntervalHandle =
                        timeServer().getAtTimeLink(learningSessionInterval);
                    timeServer().getTimeInfo(back_inserter(actionsInLearningSession),
                            learningSessionIntervalHandle,
                            *learningSessionIntervalTemporal,
                            TemporalTable::OVERLAPS);

                    foreach(HandleTemporalPair action,
                            actionsInLearningSession) {
                        Handle evaluationLink = timeServer().getAtTimeLink(action);
                        Handle predicateNode =
                            atomSpace->getOutgoing(evaluationLink)[1];
                        if (atomSpace->getName(predicateNode) ==
                                ACTION_DONE_PREDICATE_NAME) {
                            actionsDone.push_back(evaluationLink);
                        }
                    }
                }
            }
        }
    }
}

void Pet::restartLearning() throw (RuntimeException, std::bad_exception)
{
    // sanity checks
    if (learningSchema.empty()) {
        throw RuntimeException(TRACE_INFO, "Pet - No learning schema set when "
                "restarting learning..");
        return;
    }

    if (exemplarAvatarId == "") {
        throw RuntimeException(TRACE_INFO, "Pet - No exemplar avatar id set "
                "when restarting learning..");
        return;
    }

    std::vector<std::string> args;
    std::vector<std::string>::iterator it = learningSchema.begin();
    ++it;
    while (it != learningSchema.end()) {
        args.push_back(*it);
        ++it;
    }

//  std::copy(learningSchema.begin()+1, learningSchema.end(), args.begin());
    sender->sendExemplar(learningSchema.front(), args, ownerId,
            exemplarAvatarId, *atomSpace);
}

void Pet::setRequestedCommand(string command, vector<string> parameters)
{
    this->lastRequestedCommand.name = command;
    this->lastRequestedCommand.arguments = parameters;
    this->lastRequestedCommand.readed = false;
}

void Pet::startVisualDebuggerServer( const std::string& host, const std::string& port )
{
    if ( this->visualDebuggerServer == NULL ) {
        this->visualDebuggerServer = new opencog::spatial::MapExplorerServer( host, port );
    } // if
    if ( !this->visualDebuggerServer->isRunning( ) ) {
        this->visualDebuggerServer->start( );
    } // if
}

void Pet::stopVisualDebuggerServer( void )
{
    if ( this->visualDebuggerServer != NULL ) {
        if ( this->visualDebuggerServer->isRunning( ) ) {
            this->visualDebuggerServer->stop( );
        } // if
        delete this->visualDebuggerServer;
        this->visualDebuggerServer = NULL;
    } // if
}

void Pet::sendMapToVisualDebuggerClients( const SpaceServer::SpaceMap & map )
{
    if ( this->visualDebuggerServer != NULL && this->visualDebuggerServer->isRunning( ) ) {
        this->visualDebuggerServer->sendMap( map );
    } // if
}
