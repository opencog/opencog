/*
 * opencog/embodiment/Control/MessagingSystem/OPC.cc
 *
 * Copyright (C) 2007-2008 Carlos Lopes
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

#include "comboreduct/combo/type_tree.h"

#include "OPC.h"
#include "SchemaMessage.h"

#include "util/files.h"

#include <fstream>
#include <iostream>

#include "PetComboVocabulary.h"
#include <atom_types_init.h>
#include <opencog/util/Config.h>


/**
 * Uncoment the following define in order to delete atomSpace content inside OPC
 * destructor
 */
//#define DELELTE_ATOMSPACE

using namespace OperationalPetController;
using namespace Procedure;
using namespace PetCombo;
using namespace opencog;

opencog::BaseServer* OPC::createInstance() {
    return new OPC;
}

OPC::OPC() {}

void OPC::init(const std::string & myId, const std::string & ip, int portNumber,
         const std::string& petId, const std::string& ownerId,
         const std::string& agentType, const std::string& agentTraits,
         Control::SystemParameters & parameters) {

    setNetworkElement(new NetworkElement(parameters, myId, ip, portNumber));

    // OpenCog-related initialization
    atom_types_init::init();
    opencog::config().set("MIN_STI",
                          parameters.get("ATOM_TABLE_LOWER_STI_VALUE"));

    std::string aType = (agentType == "pet" || agentType == "humanoid") ? 
                         agentType : parameters.get( "RULE_ENGINE_DEFAULT_AGENT_TYPE" );


    this->atomSpace  = new AtomSpace();
    this->spaceServer = new SpaceServer(*atomSpace);
    this->planSender = new PVPActionPlanSender(petId, &(getNetworkElement()));
    this->petMessageSender = new PetMessageSender(&(getNetworkElement()));

    // load pet
    if(fileExists(getPath(petId, parameters.get("PET_DUMP")).c_str())){
      loadPet(petId);
    } else {
      this->pet = new Pet(petId, parameters.get("UNKNOWN_PET_NAME"), 
                          aType, agentTraits, ownerId, 
                          spaceServer, petMessageSender);
    }

    this->pai = new PerceptionActionInterface::PAI(*spaceServer, *planSender, *pet,
                                                   parameters);

    this->procedureRepository = new ProcedureRepository(*pai);
    this->procedureInterpreter = new ProcedureInterpreter(*pai);

    this->pet->setPAI(pai);

    // adds all savable repositories for further calls to save/load methods.
    savingLoading.addSavableRepository(spaceServer);
    savingLoading.addSavableRepository(procedureRepository);

    if(fileExists(getPath(petId, parameters.get("ATOM_SPACE_DUMP")).c_str())){
        loadAtomSpace(petId);
    } else {
        pet->initTraitsAndFeelings();

        logger().log(opencog::Logger::INFO, "OPC - Loading initial Combo stdlib file '%s', RulesPreconditions '%s' and ActionSchemataPreconditions '%s'.",
                        parameters.get("COMBO_STDLIB_REPOSITORY_FILE").c_str(),
                        parameters.get("RULES_PRECONDITIONS_REPOSITORY_FILE").c_str(),
                        parameters.get("RULES_ACTION_SCHEMATA_REPOSITORY_FILE").c_str());

        int cnt = 0;
        ifstream fin(parameters.get("COMBO_STDLIB_REPOSITORY_FILE").c_str());
        if (fin.good()) {
            cnt = procedureRepository->loadComboFromStream(fin);
        } else {
            logger().log(opencog::Logger::ERROR,
                            "OPC - Unable to load Combo stdlib.");
        }
        fin.close();
        logger().log(opencog::Logger::INFO,
                        "OPC - %d Combo functions loaded.", cnt);
        
        fin.open(parameters.get("COMBO_RULES_PRECONDITIONS_REPOSITORY_FILE").c_str());
        if (fin.good()) {
        	cnt = procedureRepository->loadComboFromStream(fin);
        } else {
        	logger().log(opencog::Logger::ERROR,
                                "OPC - Unable to load RulePreconditions combo.");
        }
        fin.close();
        logger().log(opencog::Logger::INFO,
                        "OPC - RulesPreconditions combo functions loaded.");   

        fin.open(parameters.get("COMBO_SELECT_RULES_PRECONDITIONS_REPOSITORY_FILE").c_str());
        if (fin.good()) {
        	cnt = procedureRepository->loadComboSelectFromStream(fin);
        } else {
        	logger().log(opencog::Logger::ERROR,
                                "OPC - Unable to load RulePreconditions combo select.");
        }
        fin.close();
        logger().log(opencog::Logger::INFO,
                        "OPC - RulesPreconditions combo select functions loaded.");   

        fin.open(parameters.get("COMBO_RULES_ACTION_SCHEMATA_REPOSITORY_FILE").c_str());
        if (fin.good()) {
        	cnt = procedureRepository->loadComboFromStream(fin);
        } else {
        	logger().log(opencog::Logger::ERROR,
                                "OPC - Unable to load RulesActionSchemata combo.");
        }
        fin.close();
        logger().log(opencog::Logger::INFO,
                        "OPC - RulesActionSchemata combo functions loaded.");   
    }
 

    // warning: it must be called after register the agent and it's owner nodes
    this->ruleEngine = new RuleEngine( this, petId, parameters );
    this->pet->setRuleEngine(ruleEngine);

    predicatesUpdater = new PredicatesUpdater(*spaceServer, pet->getPetId());

    // TODO: remove component reference from component constructors

    // Register and create agents 
    // IMPORTANT: the order the agents are created matters 
    //            if they must be executed sequencially.

    this->registerAgent(ProcedureInterpreterAgent::info().id, &procedureInterpreterAgentFactory);
    procedureInterpreterAgent = static_cast<ProcedureInterpreterAgent*>(
            this->createAgent(ProcedureInterpreterAgent::info().id, false));
    procedureInterpreterAgent->setInterpreter(procedureInterpreter);

    this->registerAgent(ActionSelectionAgent::info().id, &actionSelectionAgentFactory);
    actionSelectionAgent = static_cast<ActionSelectionAgent*>(
            this->createAgent(ActionSelectionAgent::info().id, false));

    this->registerAgent(ImportanceDecayAgent::info().id, &importanceDecayAgentFactory);
    importanceDecayAgent = static_cast<ImportanceDecayAgent*>(
            this->createAgent(ImportanceDecayAgent::info().id, false));

    this->registerAgent(PetInterfaceUpdaterAgent::info().id, &petInterfaceUpdaterAgentFactory);
    petInterfaceUpdaterAgent = static_cast<PetInterfaceUpdaterAgent*>(
            this->createAgent(PetInterfaceUpdaterAgent::info().id, false));

    if (atoi(getParameters().get("PROCEDURE_INTERPRETER_ENABLED").c_str())) {
        this->startAgent(procedureInterpreterAgent);
    }

    if (atoi(getParameters().get("ACTION_SELECTION_ENABLED").c_str())) {
        actionSelectionAgent->setFrequency(atoi(getParameters().get("RE_CYCLE_PERIOD").c_str()));
        this->startAgent(actionSelectionAgent);
    }

    if (atoi(getParameters().get("PROCEDURE_INTERPRETER_ENABLED").c_str())) {
        // adds the same procedure interpreter agent to schedule again
        this->startAgent(procedureInterpreterAgent);
    }

    if (atoi(getParameters().get("IMPORTANCE_DECAY_ENABLED").c_str())) {
        // TODO: Create a config parameter to control this frequency
        importanceDecayAgent->setFrequency(15);
        this->startAgent(importanceDecayAgent);
    }

    // local map 2D interface
    if (atoi(getParameters().get("PET_INTERFACE_ENABLED").c_str())) {
//        PetInterfaceUpdaterTask::startGUI();
//        petInterfaceUpdaterAgent->setFrequency(atoi(getParameters().get("PET_INTERFACE_UPDATE_PERIOD").c_str()));
//        this->startAgent(petInterfaceUpdaterAgent);
    }

    // TODO: This should be done only after NetworkElement is initialized
    // (i.e., handshake with router is done)
    // Send SUCCESS_LOAD to PROXY, so that it can start sending perception messages
    char str[100];
    sprintf(str, "SUCCESS LOAD %s %s", myId.c_str(), PerceptionActionInterface::PAIUtils::getExternalId(petId.c_str()).c_str());
    StringMessage successLoad(myId, parameters.get("PROXY_ID"), str);
    logger().log(opencog::Logger::INFO, "OPC spawned. Acking requestor");
    if (!sendMessage(successLoad)) {
        logger().log(opencog::Logger::ERROR, "Could not send SUCCESS LOAD to PROXY!");
	exit(-1);
    }
 
}

OPC::~OPC(){
    
    // WARNIG: free memory should be implemented if there are more than one opc
    // per process 
    
    delete (planSender);
    delete (petMessageSender);
    delete (predicatesUpdater);
    delete (pai);
    delete (procedureRepository);
    delete (pet);

    // agents
    delete (procedureInterpreterAgent);
    delete (importanceDecayAgent);
    delete (actionSelectionAgent);
    delete (petInterfaceUpdaterAgent);

    // TODO: It takes too much time to delete atomspace 
    // TODO: This is a hack to allow valgrind tests to work fine. When atomSpace
    // removal if fast enough remove this hack and make delete operation
    // permanent
    if(atoi(getParameters().get("CHECK_OPC_MEMORY_LEAKS").c_str())){
        logger().log(opencog::Logger::DEBUG, "OPC - Starting AtomSpace removal.");
        delete (atomSpace);
        logger().log(opencog::Logger::DEBUG, "OPC - Finished AtomSpace removal.");
    }
}

/* --------------------------------------
 * Private Methods
 * --------------------------------------
 */

void OPC::loadPet(const std::string& petId){
    // load pet metadata
    std::string file = getPath(petId, getParameters().get("PET_DUMP"));
    this->pet = Pet::importFromFile(file, petId, spaceServer, petMessageSender);
}

void OPC::loadAtomSpace(const std::string& petId){
    // load atom space and other repositories
    std::string file = getPath(petId, getParameters().get("ATOM_SPACE_DUMP"));
    savingLoading.load(file.c_str(), spaceServer->getAtomSpace());
}

void OPC::saveState(){

    if(!createDirectory(getPath(pet->getPetId()).c_str())){
        logger().log(opencog::Logger::ERROR, "OPC - Cannot create directory '%s'.",
                        getPath(pet->getPetId()).c_str());
        return;
    }

    // save atom space and othe repositories
    std::string file = getPath(pet->getPetId(), getParameters().get("ATOM_SPACE_DUMP"));
    remove(file.c_str());
    savingLoading.save(file.c_str(), spaceServer->getAtomSpace());

    // save pet metadata
    file = getPath(pet->getPetId(), getParameters().get("PET_DUMP"));
    Pet::exportToFile(file, getPet());

    // Pet state saved, send success unload message to proxy
    char str[100];
    sprintf(str, "SUCCESS UNLOAD %s %s", getID().c_str(), PerceptionActionInterface::PAIUtils::getExternalId(pet->getPetId().c_str()).c_str());
    StringMessage successUnload(getID(), getParameters().get("PROXY_ID"), str);
    logger().log(opencog::Logger::INFO, "OPC - OPC despawned (state saved). Acking requestor");
    if (!sendMessage(successUnload)) {
        logger().log(opencog::Logger::ERROR, "Could not send SUCCESS UNLOAD to PROXY!");
    }
}

void OPC::adjustPetToBePersisted(){

    // drop grabbed object, if any
    if(pet->hasGrabbedObj()){
        AtomSpaceUtil::setupHoldingObject(*atomSpace, pet->getPetId(), "", pai->getLatestSimWorldTimestamp());
        pet->setGrabbedObj("");
    }

    // put pet in playing mode stopping currently learning process
    if(pet->getMode() == LEARNING){
        pet->stopLearning(pet->getLearningSchema(), pai->getLatestSimWorldTimestamp());
    }
}

bool OPC::processSpawnerMessage(const std::string & spawnerMessage){
    logger().log(opencog::Logger::INFO, "OPC::processSpawnerMessage: msg = %s", spawnerMessage.c_str()); 
    if(spawnerMessage == "SAVE_AND_EXIT"){
        adjustPetToBePersisted();
        saveState();
        logoutFromRouter();
        return true;
    }
    return false;
}

/* --------------------------------------
 * Public Methods
 * --------------------------------------
 */
AtomSpace & OPC::getAtomSpace() {
    return *atomSpace;
}
const AtomSpace& OPC::getAtomSpace() const {
    return *atomSpace;
}

const SpaceServer & OPC::getSpaceServer() const{
    return *spaceServer;
}

PerceptionActionInterface::PAI & OPC::getPAI(){
    return *pai;
}

Pet & OPC::getPet(){
    return *pet;
}

RuleEngine & OPC::getRuleEngine(){
    return *ruleEngine;
}

ProcedureInterpreter & OPC::getProcedureInterpreter(){
    return *procedureInterpreter;
}

ProcedureRepository & OPC::getProcedureRepository(){
    return *procedureRepository;
}

PVPActionPlanSender & OPC::getPlanSender() {
    return *planSender;
}

bool OPC::processNextMessage(MessagingSystem::Message *msg){
    using namespace combo;

    logger().log(opencog::Logger::FINE, "OPC - Processing next message.");
    bool result;

    // message not for the OPC
    if (msg->getTo() != getID()){
        logger().log(opencog::Logger::WARN, "OPC - Wrong destination. Message to: %s",
                        msg->getTo().c_str());
        return false;
    }

    // message from petaverse proxy - send to PAI
    if(msg->getFrom() == getParameters().get("PROXY_ID")){
        HandleSeq toUpdateHandles;
        result = pai->processPVPMessage(msg->getPlainTextRepresentation(), toUpdateHandles);

        if(!result){
            logger().log(opencog::Logger::ERROR, "OPC - Unable to process XML message.");
        } else {

            // PVP message processed, update predicates for the
            // added/updated atoms
            predicatesUpdater->update(toUpdateHandles, pai->getLatestSimWorldTimestamp());
                    logger().log(opencog::Logger::DEBUG, "OPC - Message successfully  processed.");
        }
        return false;
    }

    // message from spawner - probabily a SAVE_AND_EXIT
    if(msg->getFrom() == getParameters().get("SPAWNER_ID")){
        result = processSpawnerMessage((std::string)msg->getPlainTextRepresentation());

        // Message correctly processed, just exit
        if(result){
            // TODO: Save status...
            logger().log(opencog::Logger::INFO, "OPC - Exiting...");
            return true;
        }
    }

    // message from the combo shell to execute a schema
    if (msg->getFrom() == getParameters().get("COMBO_SHELL_ID")) {
      std::string str(msg->getPlainTextRepresentation());
      logger().log(opencog::Logger::ERROR, "OPC - Got combo shell msg: '%s'",str.c_str());

      if (str.empty())
        return false; //a timing error, maybe?

      std::stringstream ss(str);
      combo_tree tr;
      ss >> tr;
      ComboProcedure cp("",0,tr);
      std::vector<vertex> args; //an expression, not a function - no args
      procedureInterpreter->runProcedure(cp,args);
      logger().log(opencog::Logger::ERROR,
		      "OPC - Called runProcedure(" + ss.str() + ")");
    }

    // message from learning server
    if(msg->getFrom() == getParameters().get("LS_ID")){
        LearningServerMessages::SchemaMessage * sm = (LearningServerMessages::SchemaMessage *)msg;
        logger().log(opencog::Logger::DEBUG, "OPC - Got msg from LS: '%s'", msg->getPlainTextRepresentation());
        
        // sanity check to see if LS does not return an empty
        // ComboSchema
        if(sm->getComboSchema().empty()){
            
            logger().log(opencog::Logger::WARN, 
                    "OPC - Received an empty ComboSchema fom LS. Discarding it.");
            return false;

        } else {

            // add schema to combo repository 
	    //check first if a procedure of that name already exists an remove it
	    //if so
	    //WARNING : if there was dependencies involving that procedure
	    //then removing it is going to generate an invalid procedure_call pointer
	    if(procedureRepository->contains(sm->getSchemaName()))
	      procedureRepository->remove(sm->getSchemaName());

	    //note that if the type is infered and checked
	    //it doesn't matter what arity is specified (here 0)
	    //because it is going to be overwrite with at the type check
	    bool tc = static_cast<bool>(atoi(getParameters().get("TYPE_CHECK_LOADING_PROCEDURES").c_str()));
	    arity_t a = infer_arity(sm->getComboSchema());
            procedureRepository->add(ComboProcedure(sm->getSchemaName(),
						    a, sm->getComboSchema(),
						    tc));
        }

        switch(sm->getType()){
            // note: assuming arity==0 for now - Moshe

            case MessagingSystem::Message::SCHEMA:
                {
                    // learning is finished, set pet to PLAYING state. This
                    // design ensure that the learning info will not be lost
                    // until a learned schema is received
		    // NOTE: transfered back to when "stop learning" instruction 
		    // is processed so that OPC does not stay in Lerning mode 
		    // forever if LS crashes... 
                    //pet->setMode(PLAYING);
                   
                    // Add schema to RuleEngine learned schemata
		    ruleEngine->addLearnedSchema( sm->getSchemaName( ) );
                }
                break;

            case MessagingSystem::Message::CANDIDATE_SCHEMA:
                {
                    // Add schema to RuleEngine learned schemata ...
		            ruleEngine->addLearnedSchema( sm->getSchemaName( ) );
                    
                    // .. and execute it
                    pet->setTriedSchema(sm->getSchemaName());
        			ruleEngine->tryExecuteSchema( sm->getSchemaName( ) );
                    
                }
                break;

            default:
                {
                    logger().log(opencog::Logger::ERROR,
                        "Not a SCHEMA or CANDIDATE_SCHEMA message!!!");
                }
                break;
        }
    }
    return false;
}

void OPC::schemaSelection() {
  logger().log(opencog::Logger::FINE, "OPC - Executing selectSchemaToExecute().");

  this->pet->getCurrentModeHandler( ).update( );
  this->ruleEngine->processNextAction( );
  this->ruleEngine->runSchemaForCurrentAction( );

//  if ( pet->getMode( ) != PLAYING && pet->getMode( ) != LEARNING ) {
//    pet->setMode( PLAYING );
//  } // if
}

void OPC::decayShortTermImportance() {
    atomSpace->decayShortTermImportance();
}


/* TODO: OPC does not extend NetworkElement anymore. 
 * So, a better design must be created to figure out LS has just recovered from a failure:
void OPC::markAsAvailableElement(const std::string& id){
    logger().log(opencog::Logger::DEBUG, "OPC - markAsAvailableElement(%s).", id.c_str());

    // remove element from unavailable list and handshake if router
    NetworkElement::markAsAvailableElement(id);

    if (isInitialized()) {
        // OPC-specific actions
        PetMode mode = pet->getMode();
        if(mode == OperationalPetController::LEARNING){
            // Pet still in learning mode and LS has recovered from a crash. Restart
            // the learning process from the begining
	    // TODO: Review this. Perhaps the best thing to do is exiting Learning mode and give feedback to the user (via Feedback messages)
            if(id == getParameters().get("LS_ID")){
                pet->restartLearning();
            }
        }
    }
}
*/

const std::string OPC::getPath(const std::string& petId, const std::string& filename){
    std::string path;

    std::string base = getParameters().get("PET_DATABASE");
    expandPath(base);

    logger().log(opencog::Logger::DEBUG, "OPC - Pet database directory: %s", base.c_str());
    path.append(base);
    path.append("/pet_");
    path.append(petId);

    // no empty string
    if(filename.size() > 0){
        path.append("/");
        path.append(filename);
    }

    logger().log(opencog::Logger::DEBUG, "OPC - getPath: " + path);

    return path;
}

