/*
 * opencog/embodiment/Control/OperationalAvatarController/OAC.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
 *
 * Updated: By Jinhua Chua <JinhuaChua@gmail.com>, on 2011-12-19
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
#include <fstream>
#include <iostream>

#include <boost/format.hpp>

#include <opencog/util/files.h>
#include <opencog/util/Config.h>

#include <opencog/comboreduct/type_checker/type_tree.h>

// For loading Scheme scripts by C++ code
#include <opencog/guile/load-file.h>

#include <opencog/embodiment/Control/MessagingSystem/MessageFactory.h>
#include <opencog/embodiment/Learning/LearningServerMessages/SchemaMessage.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>
#include <opencog/spacetime/SpaceTime.h>

#include "OAC.h"
#include "Inquery.h"

/**
 * Uncoment the following define in order to delete atomSpace content inside OAC
 * destructor
 */
//#define DELETE_ATOMSPACE

namespace opencog { 
namespace oac {

using namespace Procedure;
using namespace AvatarCombo;
using namespace pai;
using learningserver::messages::SchemaMessage;

BaseServer* OAC::createInstance()
{
    return new OAC;
}

OAC::OAC()
{
}

bool OAC::customLoopRun(void)
{
#ifdef HAVE_ZMQ
    this->plaza->forwardMessages();
#endif
    return MessageCogServer::customLoopRun();
}

void OAC::init(const std::string & myId, const std::string & ip, int portNumber,
               const std::string & zmqPublishPort,
               const std::string& petId, const std::string& ownerId,
               const std::string& agentType, const std::string& agentTraits)
{
    setNetworkElement(new NetworkElement(myId, ip, portNumber));

    // Initialize ZeroMQ
#ifdef HAVE_ZMQ
    this->plaza = new Plaza(config().get("ZMQ_PUBLISH_IP"), zmqPublishPort);
#endif

    std::string aType = (agentType == "pet" || agentType == "humanoid") ?
                        agentType : config().get( "RULE_ENGINE_DEFAULT_AGENT_TYPE" );


    this->planSender = new PVPActionPlanSender(petId, &(getNetworkElement()));
    this->petMessageSender = new PetMessageSender(&(getNetworkElement()));

    // load pet
// Note: Disable reload pet from external file temporally. It crashes. 
//    if (fileExists(getPath(petId, config().get("PET_DUMP")).c_str())) {
//        loadPet(petId);
//    } else {
        this->pet = new Pet(petId, config().get("UNKNOWN_PET_NAME"),
                            aType, agentTraits, ownerId,
                            atomSpace, petMessageSender);
//    }

    this->pai = new PAI(*atomSpace, *planSender, *pet);
    new EventResponder(*pai , *atomSpace);
    //new EventDetector(*pai , *atomSpace);
    this->procedureRepository = new ProcedureRepository(*pai);
    this->procedureInterpreter = new ProcedureInterpreter(*pai);

    this->pet->setPAI(pai);

    // adds all savable repositories for further calls to save/load methods.
    savingLoading.addSavableRepository(procedureRepository);

//    if (fileExists(getPath(petId, config().get("ATOM_SPACE_DUMP")).c_str())) {
//        loadAtomSpace(petId);
//    } else {
        pet->initTraitsAndFeelings();

        logger().info( "OAC::%s Loading initial Combo stdlib file '%s', ActionSchemataPreconditions '%s'",
                       __FUNCTION__, 
                       config().get("COMBO_STDLIB_REPOSITORY_FILE").c_str(),
                       config().get("COMBO_RULES_ACTION_SCHEMATA_REPOSITORY_FILE").c_str() 
                     );

        int cnt = 0;
        ifstream fin(config().get("COMBO_STDLIB_REPOSITORY_FILE").c_str());
        if (fin.good()) {
            cnt = procedureRepository->loadComboFromStream(fin);
        } else {
            logger().error("OAC::%s Unable to load Combo stdlib.", __FUNCTION__);
        }
        fin.close();
        logger().info("OAC::%s %d combo stdlib functions loaded.",
                      __FUNCTION__,  
                      cnt
                     );

        std::string action_schema_name; 
        if ( config().get_bool("ENABLE_UNITY_CONNECTOR") ) 
            action_schema_name = "unity_action_schema.combo";
        else
            action_schema_name = "multiverse_action_schema.combo"; 

        fin.open( action_schema_name.c_str() );
        if (fin.good()) {
            cnt = procedureRepository->loadComboFromStream(fin);
        } else {
            logger().error("OAC::%s Unable to load action schema combo.", __FUNCTION__);
        }
        fin.close();
        logger().info("OAC::%s %d action schema combo functions loaded.", 
                      __FUNCTION__, 
                      cnt
                     );

//    }// if

    // warning: it must be called after register the agent and it's owner nodes
    predicatesUpdater = new PredicatesUpdater(*atomSpace, pet->getPetId());

    // Load Psi Rules ('xxx_rules.scm') to AtomSpace
    //
    // Psi Rules should be loaded before running PsixxxAgent.
    // And before loading Psi Rules, make sure 'rules_core.scm' has been loaded.
    // Since 'rules_core.scm' is loaded as a SCM module, 
    // the correct sequence of initialization is 
    // OAC::loadSCMModules, OAC::addRulesToAtomSpace and PsiXxxAgent finally.
    this->addRulesToAtomSpace();

    // TODO: remove component reference from component constructors

    // Register and create agents
    // IMPORTANT: the order the agents are created matters
    //            if they must be executed sequencially.

//    this->registerAgent(ActionSelectionAgent::info().id, &actionSelectionAgentFactory);
//    actionSelectionAgent = 
//                               this->createAgent(ActionSelectionAgent::info().id, false));

//    this->registerAgent(ImportanceDecayAgent::info().id, &importanceDecayAgentFactory);
//    importanceDecayAgent = 
//                               this->createAgent(ImportanceDecayAgent::info().id, false));
//    importanceDecayAgent->connectSignals(*atomSpace);

//    this->registerAgent(EntityExperienceAgent::info().id, &entityExperienceAgentFactory);
//    entityExperienceAgent = 
//                               this->createAgent(EntityExperienceAgent::info().id, false));

    // Three steps to run a MindAgent
    // registerAgent, createAgent and startAgent
    registerAgent(PsiDemandUpdaterAgent::info().id, 
                  &psiDemandUpdaterAgentFactory);
    psiDemandUpdaterAgent = createAgent<PsiDemandUpdaterAgent>();

    registerAgent(PsiModulatorUpdaterAgent::info().id, 
                  &psiModulatorUpdaterAgentFactory);
    psiModulatorUpdaterAgent = createAgent<PsiModulatorUpdaterAgent>();

    // OCPlanningAgent and PsiActionSelectionAgent cannot be enabled at the same time
    if (config().get_bool("OCPLANNING_AGENT_ENABLED"))
    {
        registerAgent(OCPlanningAgent::info().id,
                             &ocPlanningAgentAgentFactory);
        ocPlanningAgent = createAgent<OCPlanningAgent>();

        psiActionSelectionAgent = 0;
    }
    else if (config().get_bool("PSI_ACTION_SELECTION_ENABLED"))
    {
        registerAgent(PsiActionSelectionAgent::info().id,
                             &psiActionSelectionAgentFactory);
        psiActionSelectionAgent = createAgent<PsiActionSelectionAgent>();
        ocPlanningAgent = 0;
    }
    else
    {
        psiActionSelectionAgent = 0;
        ocPlanningAgent = 0;
    }

    registerAgent(ProcedureInterpreterAgent::info().id, &procedureInterpreterAgentFactory);
    procedureInterpreterAgent = createAgent<ProcedureInterpreterAgent>();
    procedureInterpreterAgent->setInterpreter(procedureInterpreter);

    registerAgent( PsiRelationUpdaterAgent::info().id, 
                         &psiRelationUpdaterAgentFactory);
    psiRelationUpdaterAgent = createAgent<PsiRelationUpdaterAgent>();

    registerAgent( PsiFeelingUpdaterAgent::info().id, 
                         &psiFeelingUpdaterAgentFactory);
    psiFeelingUpdaterAgent = createAgent<PsiFeelingUpdaterAgent>();

    registerAgent( StimulusUpdaterAgent::info().id, 
                         &stimulusUpdaterAgentFactory);
    stimulusUpdaterAgent = createAgent<StimulusUpdaterAgent>();

    registerAgent( ForgettingAgent::info().id, 
                         &forgettingAgentFactory);
    forgettingAgent = createAgent<ForgettingAgent>();

    registerAgent( HebbianUpdatingAgent::info().id, 
                         &hebbianUpdatingAgentFactory);
    hebbianUpdatingAgent = createAgent<HebbianUpdatingAgent>();

//    registerAgent( ImportanceDiffusionAgent::info().id, 
//                         &importanceDiffusionAgentFactory);
//    importanceDiffusionAgent = createAgent<ImportanceDiffusionAgent>();

    registerAgent( ImportanceSpreadingAgent::info().id, 
                         &importanceSpreadingAgentFactory);
    importanceSpreadingAgent = createAgent<ImportanceSpreadingAgent>();

    registerAgent( ImportanceUpdatingAgent::info().id, 
                         &importanceUpdatingAgentFactory);
    importanceUpdatingAgent = createAgent<ImportanceUpdatingAgent>();

//    if (config().get_bool("PROCEDURE_INTERPRETER_ENABLED")) {
        // adds the same procedure interpreter agent to schedule again
//        this->startAgent(procedureInterpreterAgent);
//    }

//    if (config().get_bool("IMPORTANCE_DECAY_ENABLED")) {
//        importanceDecayAgent->setFrequency(
//            config().get_int("IMPORTANCE_DECAY_CYCLE_PERIOD"));
//        this->startAgent(importanceDecayAgent);
//    }

//    if (config().get_bool("ENTITY_EXPERIENCE_ENABLED")) {
//        this->entityExperienceAgent->setFrequency(
//           config().get_int( "ENTITY_EXPERIENCE_MOMENT_CYCLE_PERIOD" ) );
//        this->startAgent(entityExperienceAgent);
//    }

    if (config().get_bool("PSI_MODULATOR_UPDATER_ENABLED")) {
        this->psiModulatorUpdaterAgent->setFrequency(
           config().get_int( "PSI_MODULATOR_UPDATER_CYCLE_PERIOD" ) );
        this->startAgent(psiModulatorUpdaterAgent);
    }

    if (config().get_bool("PSI_DEMAND_UPDATER_ENABLED")) {
        this->psiDemandUpdaterAgent->setFrequency(
           config().get_int( "PSI_DEMAND_UPDATER_CYCLE_PERIOD" ) );
        this->startAgent(psiDemandUpdaterAgent);
    }
  
    // OCPlanningAgent and PsiActionSelectionAgent cannot be enabled at the same time
    if (config().get_bool("OCPLANNING_AGENT_ENABLED"))
    {
        this->ocPlanningAgent->setFrequency(
                    config().get_int( "OCPLANNING_AGENT_CYCLE_PERIOD" ) );
        this->startAgent(ocPlanningAgent);

    }
    else if (config().get_bool("PSI_ACTION_SELECTION_ENABLED")) {
        this->psiActionSelectionAgent->setFrequency(
           config().get_int( "PSI_ACTION_SELECTION_CYCLE_PERIOD" ) );
        this->startAgent(psiActionSelectionAgent);
    }

    if (config().get_bool("PROCEDURE_INTERPRETER_ENABLED")) {
        procedureInterpreterAgent->setFrequency(1); 
        this->startAgent(procedureInterpreterAgent);
    }

    if (config().get_bool("PSI_RELATION_UPDATER_ENABLED")) {
        this->psiRelationUpdaterAgent->setFrequency(
           config().get_int( "PSI_RELATION_UPDATER_CYCLE_PERIOD" ) );
        this->startAgent(psiRelationUpdaterAgent);
    }

    if (config().get_bool("PSI_FEELING_UPDATER_ENABLED")) {
        this->psiFeelingUpdaterAgent->setFrequency(
           config().get_int( "PSI_FEELING_UPDATER_CYCLE_PERIOD" ) );
        this->startAgent(psiFeelingUpdaterAgent);
    }

    if (config().get_bool("STIMULUS_UPDATER_ENABLED")) {
        this->psiFeelingUpdaterAgent->setFrequency(
           config().get_int( "STIMULUS_UPDATER_CYCLE_PERIOD" ) );
        this->startAgent(stimulusUpdaterAgent);
    }

    if (config().get_bool("FORGETTING_ENABLED")) {
        this->forgettingAgent->setFrequency(
           config().get_int( "FORGETTING_CYCLE_PERIOD" ) );
        this->startAgent(forgettingAgent);
    }

    if (config().get_bool("HEBBIAN_UPDATING_ENABLED")) {
        this->hebbianUpdatingAgent->setFrequency(
           config().get_int( "HEBBIAN_UPDATING_CYCLE_PERIOD" ) );
        this->startAgent(hebbianUpdatingAgent);
    }

//    if (config().get_bool("IMPORTANCE_DIFFUSION_ENABLED")) {
//        this->importanceDiffusionAgent->setFrequency(
//           config().get_int( "IMPORTANCE_DIFFUSION_CYCLE_PERIOD" ) );
//        this->startAgent(importanceDiffusionAgent);
//    }

    if (config().get_bool("IMPORTANCE_SPREADING_ENABLED")) {
        this->importanceSpreadingAgent->setFrequency(
           config().get_int( "IMPORTANCE_SPREADING_CYCLE_PERIOD" ) );
        this->startAgent(importanceSpreadingAgent);
    }

    if (config().get_bool("IMPORTANCE_UPDATING_ENABLED")) {
        this->importanceUpdatingAgent->setFrequency(
           config().get_int( "IMPORTANCE_UPDATING_CYCLE_PERIOD" ) );
        this->startAgent(importanceUpdatingAgent);
    }

#ifdef HAVE_CYTHON
    if ( config().get_bool("FISHGRAM_ENABLED") ) {
        this->fishgramAgent = PyMindAgentPtr(new PyMindAgent(*this, "fishgram", "FishgramMindAgent"));
        this->fishgramAgent->setFrequency( config().get_int("FISHGRAM_CYCLE_PERIOD") ); 
        this->startAgent(this->fishgramAgent); 
    }
    else 
        this->fishgramAgent = NULL; 

    if ( config().get_bool("MONITOR_CHANGES_ENABLED") ) {
        this->monitorChangesAgent = PyMindAgentPtr(new PyMindAgent(*this, "monitor_changes", "MonitorChangesMindAgent"));
        this->monitorChangesAgent->setFrequency( config().get_int("MONITOR_CHANGES_CYCLE_PERIOD") ); 
        this->startAgent(this->monitorChangesAgent); 
    }
    else
        this->monitorChangesAgent = NULL; 
#endif

    if ( config().get_bool("ENABLE_PATTERN_MINER"))
    {
        if ( load_scm_file( *(this->atomSpace), "pm_test_corpus.scm" ) == 0  )
            logger().info( "OAC::%s - Loaded pattern miner test corpus file: '%s'",
                            __FUNCTION__,
                           "pm_test_corpus.scm"
                         );
        else
            logger().error( "OAC::%s - Failed to load pattern miner test corpus file: '%s'",
                             __FUNCTION__,
                            "pm_test_corpus.scm"
                          );


        this->patternMiningAgent = PatternMiningAgentPtr(new PatternMiningAgent(*this));
        this->startAgent(this->patternMiningAgent);
    }
    else
        this->patternMiningAgent = NULL;

    // TODO: This should be done only after NetworkElement is initialized
    // (i.e., handshake with router is done)
    // Send SUCCESS_LOAD to PROXY, so that it can start sending perception messages
    char str[100];
    sprintf(str, "SUCCESS LOAD %s %s", myId.c_str(), PAIUtils::getExternalId(petId.c_str()).c_str());
    StringMessage successLoad(myId, config().get("PROXY_ID"), str);
    logger().info("OAC spawned. Acking requestor");
    if (!sendMessage(successLoad)) {
        logger().error("OAC - Could not send SUCCESS LOAD to PROXY!");
    }

    if ( config().get("VISUAL_DEBUGGER_ACTIVE") == "true" ) {
        int minPort = boost::lexical_cast<unsigned int>
            ( config().get("MIN_OAC_PORT") );

        std::string visualDebuggerHost = 
            config().get("VISUAL_DEBUGGER_HOST");

        std::string visualDebuggerStartPort =
            config().get("VISUAL_DEBUGGER_PORT");

        unsigned int visualDebuggerPort = static_cast<unsigned int>
            ( boost::lexical_cast<unsigned int>( visualDebuggerStartPort ) + ( portNumber - minPort ) );
        
        this->pet->startVisualDebuggerServer( visualDebuggerHost, 
            boost::lexical_cast<std::string>( visualDebuggerPort ) );

    } // if


    Inquery::init(atomSpace);

    // Run demand/ feeling updater agents as soon as possible, then virtual
    // world (say unity) will not wait too much time to get the initial values
    //
    // TODO: This is a temporally solution. We should reduce the time of oac 
    //       initialization. For example, don't send all the map info at initialization. 
    //
//    this->psiDemandUpdaterAgent->run(this); 
//    this->psiFeelingUpdaterAgent->run(this); 
//    this->psiModulatorUpdaterAgent->run(this); 

    // TODO: multi-threading doesn't work. 
//    this->thread_attention_allocation = boost::thread( boost::bind(&attention_allocation, this) ); 
}

void OAC::attention_allocation(OAC * oac)
{
    for (int i=0; i>=0; i++) {
        usleep(500000); 
        oac->runAgent(oac->forgettingAgent);
        std::cout<<"forgettingAgent "<<i<<std::endl; 
        oac->runAgent(oac->hebbianUpdatingAgent); 
        std::cout<<"hebbianUpdatingAgent "<<i<<std::endl; 
//    ImportanceDiffusionAgent * importanceDiffusionAgent; 
        oac->runAgent(oac->importanceSpreadingAgent);     
        std::cout<<"importanceSpreadingAgent "<<i<<std::endl; 
        oac->runAgent(oac->importanceUpdatingAgent); 
        std::cout<<"importanceUpdatingAgent "<<i<<std::endl; 
        std::cout<<"attention_allocation Done: "<<i<<std::endl; 
    }
}

int OAC::addRulesToAtomSpace()
{
    // Load core file
    //
    // There are two ways to load "rules_core.scm", 
    // one is adding "rules_core.scm" to the end of SCM_PRELOAD in "embodiment.conf",
    // the other is using the code commented below.
    // 
    // If we choose the latter solution, then we can not use the functions/variables 
    // defined in "rules_core.scm" in OpenCog shell. So we adopt the first method.
    //
/*
    std::string psi_rules_core_file_name = config().get("PSI_RULES_CORE_FILE");

    if ( load_scm_file( *(this->atomSpace), re_core_file_name.c_str() ) == 0  ) 
        logger().info( "OAC::%s - Loaded psi rules core file: '%s'", 
                        __FUNCTION__, 
                        psi_rules_core_file_name)c_str() 
                     );
    else 
        logger().error( "OAC::%s - Failed to load psi rules core file: '%s'", 
                         __FUNCTION__, 
                        psi_rules_core_file_name.c_str() 
                      );
    
*/
#ifdef HAVE_GUILE
    // Set PET_HANDLE and OWNER_HANDLE for the Scheme shell before loading rules file
    SchemeEval* evaluator = new SchemeEval(atomSpace);
    std::string scheme_expression, scheme_return_value;

    scheme_expression =  "(set! PET_HANDLE (get_agent_handle \"" + 
                                            this->getPet().getPetId() + 
                                            "\") )";

    scheme_expression += "(define agentSemeNode (SemeNode \"" +
                                                 this->getPet().getPetId() +
                                                 "\") )";

    scheme_expression += "(set! OWNER_HANDLE (get_owner_handle \"" + 
                                              this->getPet().getOwnerId() + 
                                             "\") )";

    scheme_return_value = evaluator->eval(scheme_expression);

    if ( evaluator->eval_error() ) 
        logger().error("OAC::%s - Failed to set PET_HANDLE and OWNER_HANDLE",
                       __FUNCTION__
                      );
    else 
        logger().info("OAC::%s - Set PET_HANDLE and OWNER_HANDLE for Scheme shell",
                      __FUNCTION__
                     );
    delete evaluator;
    evaluator = NULL;
            
    // Load the psi rules file, including Modulators, DemandGoals and Rules 
    std::string psi_rules_file_name; 

    if ( config().get_bool("ENABLE_UNITY_CONNECTOR") ) 
        psi_rules_file_name = "unity_rules.scm"; 
    else
        psi_rules_file_name = "multiverse_rules.scm"; 

    if ( load_scm_file( *(this->atomSpace), psi_rules_file_name.c_str() ) == 0  ) 
        logger().info( "OAC::%s - Loaded psi rules file: '%s'", 
                        __FUNCTION__, 
                       psi_rules_file_name.c_str() 
                     );
    else
        logger().error( "OAC::%s - Failed to load psi rules file: '%s'", 
                         __FUNCTION__, 
                        psi_rules_file_name.c_str() 
                      );

    // Load the dialog system rules file
    std::string dialog_system_rules_file_name = "dialog_system.scm"; 

    if ( load_scm_file( *(this->atomSpace), dialog_system_rules_file_name.c_str() ) == 0  ) 
        logger().info( "OAC::%s - Loaded dialog system rules file: '%s'", 
                        __FUNCTION__, 
                       dialog_system_rules_file_name.c_str() 
                     );
    else
        logger().error( "OAC::%s - Failed to load dialog system rules file: '%s'", 
                         __FUNCTION__, 
                        dialog_system_rules_file_name.c_str() 
                      );

    // Load the speech act schema file
    std::string speech_act_schema_file_name; 

    if ( config().get_bool("ENABLE_UNITY_CONNECTOR") ) 
        speech_act_schema_file_name = "unity_speech_act_schema.scm"; 
    else
        speech_act_schema_file_name = "multiverse_speech_act_schema.scm"; 

    if ( load_scm_file( *(this->atomSpace), speech_act_schema_file_name.c_str() ) == 0  ) 
        logger().info( "OAC::%s - Loaded speech act schema file: '%s'", 
                        __FUNCTION__, 
                       speech_act_schema_file_name.c_str() 
                     );
    else
        logger().error( "OAC::%s - Failed to load speech act schema file: '%s'", 
                         __FUNCTION__, 
                        speech_act_schema_file_name.c_str() 
                      );

    // Load the event-driven rules file for unity.
    if ( config().get_bool("ENABLE_UNITY_CONNECTOR") ) {
        std::string unity_stimulus_rules_file_name = "unity_stimulus_rules.scm";

        if ( load_scm_file( *(this->atomSpace), unity_stimulus_rules_file_name.c_str() ) == 0  ) {
            logger().info( "OAC::%s - Loaded stimulus rules file: '%s'", 
                            __FUNCTION__, 
                           unity_stimulus_rules_file_name.c_str() 
                         );
        } else {
            logger().error( "OAC::%s - Failed to load stimulus rules file: '%s'", 
                             __FUNCTION__, 
                            unity_stimulus_rules_file_name.c_str() 
                          );
        }
        
        std::string unity_attitude_processor_file_name = "unity_attitude_processor.scm";

        if ( load_scm_file( *(this->atomSpace), unity_attitude_processor_file_name.c_str() ) == 0  ) {
            logger().info( "OAC::%s - Loaded attitude processor module file: '%s'", 
                           __FUNCTION__, 
                           unity_attitude_processor_file_name.c_str() 
                         );
        } else {
            logger().error( "OAC::%s - Failed to load attitude processor module file: '%s'", 
                            __FUNCTION__, 
                            unity_attitude_processor_file_name.c_str() 
                          );
        }
    }
#endif /* HAVE_GUILE */


    return 0;
}

OAC::~OAC()
{

    // WARNIG: free memory should be implemented if there are more than one oac
    // per process

    delete (planSender);
    delete (petMessageSender);
    delete (predicatesUpdater);
    EventResponder::getInstance()->destroy();
    //EventDetector::getInstance()->destroy();
    delete (pai);
    delete (procedureRepository);
    delete (pet);

    // agents
#if 0
    delete (procedureInterpreterAgent);
//    delete (importanceDecayAgent);
    delete (psiModulatorUpdaterAgent);

    if (psiActionSelectionAgent)
        delete (psiActionSelectionAgent);

    if (ocPlanningAgent)
        delete (ocPlanningAgent);

    delete (psiRelationUpdaterAgent); 
    delete (psiFeelingUpdaterAgent); 

    delete (stimulusUpdaterAgent);

    delete (forgettingAgent); 
    delete (hebbianUpdatingAgent); 
//    delete (importanceDiffusionAgent); 
    delete (importanceSpreadingAgent); 
    delete (importanceUpdatingAgent); 

#ifdef HAVE_CYTHON
    delete (fishgramAgent); 
    delete (monitorChangesAgent); 
#endif
#endif

    // ZeroMQ 
#ifdef HAVE_ZMQ    
    delete plaza;
#endif    

#ifndef DELETE_ATOMSPACE 
    // TODO: It takes too much time to delete atomspace. So, atomspace removal
    // is currently disable. This is a hack to allow valgrind tests to work fine. 
    // When atomSpace removal is fast enough, remove this hack and enable delete
    // operation again.
    if (config().get_bool("CHECK_OAC_MEMORY_LEAKS")) {
#endif
    logger().debug("OAC - Starting AtomSpace removal.");
    printf("OAC - Starting AtomSpace removal.\n");
    int t1 = time(NULL);
    delete (atomSpace);
    int t2 = time(NULL);
    logger().debug("OAC - Finished AtomSpace removal. t1 = %d, t2=%d, elapsed time =%d seconds", t1, t2, t2-t1);
    printf("OAC - Finished AtomSpace removal. t1 = %d, t2=%d, diff=%d\n", t1, t2, t2-t1);
#ifndef DELETE_ATOMSPACE 
    }
#endif
}

/* --------------------------------------
 * Private Methods
 * --------------------------------------
 */

void OAC::loadPet(const std::string& petId)
{
    // load pet metadata
    std::string file = getPath(petId, config().get("PET_DUMP"));
    this->pet = Pet::importFromFile(file, petId, atomSpace, petMessageSender);
}

void OAC::loadAtomSpace(const std::string& petId)
{
    // load atom space and other repositories
    std::string file = getPath(petId, config().get("ATOM_SPACE_DUMP"));
    savingLoading.load(file.c_str(), *atomSpace, spaceServer(), timeServer());
}

void OAC::saveState()
{
    // ensure directory for pet db exists
    if (!createDirectory(getPath(pet->getPetId()).c_str())) {
        logger().error("OAC - Cannot create directory '%s'.",
                     getPath(pet->getPetId()).c_str());
        return;
    }

    // save atom space and othe repositories
    std::string file = getPath(pet->getPetId(), config().get("ATOM_SPACE_DUMP"));

    remove(file.c_str());
    savingLoading.save(file.c_str(), *atomSpace, spaceServer(), timeServer());

    // save pet metadata
    file = getPath(pet->getPetId(), config().get("PET_DUMP"));
    Pet::exportToFile(file, getPet());

    // Pet state saved, send success unload message to proxy
    char str[100];
    sprintf(str, "SUCCESS UNLOAD %s %s", getID().c_str(), PAIUtils::getExternalId(pet->getPetId().c_str()).c_str());
    StringMessage successUnload(getID(), config().get("PROXY_ID"), str);
    logger().info("OAC - OAC despawned (state saved). Acking requestor");
    if (!sendMessage(successUnload)) {
        logger().error("Could not send SUCCESS UNLOAD to PROXY!");
    }
}

void OAC::adjustPetToBePersisted()
{

    // drop grabbed object, if any
    if (pet->hasGrabbedObj()) {
        AtomSpaceUtil::setupHoldingObject(*atomSpace, pet->getPetId(), "", pai->getLatestSimWorldTimestamp());
        pet->setGrabbedObj("");
    }

    // put pet in playing mode stopping currently learning process
    if (pet->getMode() == LEARNING) {
        pet->stopLearning(pet->getLearningSchema(), pai->getLatestSimWorldTimestamp());
    }
}

bool OAC::processSpawnerMessage(const std::string & spawnerMessage)
{
    logger().info("OAC::processSpawnerMessage: msg = %s", spawnerMessage.c_str());
    if (spawnerMessage == "SAVE_AND_EXIT") {
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

PAI & OAC::getPAI()
{
    return *pai;
}

Pet & OAC::getPet()
{
    return *pet;
}

ProcedureInterpreter & OAC::getProcedureInterpreter()
{
    return *procedureInterpreter;
}

ProcedureRepository & OAC::getProcedureRepository()
{
    return *procedureRepository;
}

PVPActionPlanSender & OAC::getPlanSender()
{
    return *planSender;
}

bool OAC::processNextMessage(messaging::Message *msg)
{
    using namespace combo;

    logger().fine("OAC - Processing next message.");
    bool result;

    // message not for the OAC
    if (msg->getTo() != getID()) {
        logger().warn("OAC::%s - This message is not for OAC. Its destination is %s. Message content: %s",
                       __FUNCTION__, 
                       msg->getTo().c_str(), 
                       msg->getPlainTextRepresentation()
                     );
        return false;
    }

    // message that has been parsed by RelEx server
    if(msg->getFrom() == config().get("RELEX_SERVER_ID")) {
        HandleSeq toUpdateHandles;
        result = pai->processPVPMessage(msg->getPlainTextRepresentation(), toUpdateHandles);

        if (!result) {
            logger().error("OAC::%s - Unable to process XML message.", __FUNCTION__);
        } else {
            // PVP message processed, update predicates for the
            // added/updated atoms
            predicatesUpdater->update(toUpdateHandles, pai->getLatestSimWorldTimestamp());
            logger().debug("OAC::%s - Message successfully  processed.", __FUNCTION__);
        }
        return false;
    }

    // message from embodiment proxy - send to PAI
    if (msg->getFrom() == config().get("PROXY_ID")) {
        // @note:
        // The message type RAW is used for unity environment to handle dialog.
		// If you use multiverse, just ignore this.
        if(msg->getType() == messaging::RAW) {
			// message from OC Avatar, forward it to RelEx server.
            StringMessage rawMessage( getID(),
                                      config().get("RELEX_SERVER_ID"), 
                                      msg->getPlainTextRepresentation()
                                    );


            if ( !sendMessage(rawMessage) ) {
                logger().error("OAC::%s - Failed to forward raw message to RelEx server. Message content: %s", 
                                __FUNCTION__, 
                                msg->getPlainTextRepresentation()
                              );
            }
            else {
                logger().debug("OAC::%s - Forward raw message to RelEx server successfully. Message content: %s", 
                               __FUNCTION__, 
                               msg->getPlainTextRepresentation()
                              );
            }
        } 
        else {
            HandleSeq toUpdateHandles;
            result = pai->processPVPMessage(msg->getPlainTextRepresentation(), toUpdateHandles);

            if (!result) {
                logger().error("OAC::%s - Unable to process XML message.", __FUNCTION__);
            } else {
                // PVP message processed, update predicates for the
                // added/updated atoms
                predicatesUpdater->update(toUpdateHandles, pai->getLatestSimWorldTimestamp());

                logger().debug("OAC::%s - Message successfully  processed.", __FUNCTION__);
            }
        }
        return false;
    }

    // message from spawner - probably a SAVE_AND_EXIT
    if (msg->getFrom() == config().get("SPAWNER_ID")) {
        result = processSpawnerMessage((std::string)msg->getPlainTextRepresentation());

        // Message correctly processed, just exit
        if (result) {
            // TODO: Save status...
            logger().info("OAC::%s - Exiting...",__FUNCTION__);
            return true;
        }
    }

    // message from the combo shell to execute a schema
    if (msg->getFrom() == config().get("COMBO_SHELL_ID")) {
        std::string str(msg->getPlainTextRepresentation());
        logger().info("OAC::%s - Got combo shell msg: '%s'", __FUNCTION__, str.c_str());

        if (str.empty())
            return false; //a timing error, maybe?

        std::stringstream ss(str);
        combo_tree tr;
        AvatarCombo::operator>>(ss, tr);
        ComboProcedure cp("", 0, tr);
        std::vector<vertex> args; //an expression, not a function - no args
        procedureInterpreter->runProcedure(cp, args);
        logger().info("OAC - Called runProcedure(" + ss.str() + ")");
    }

    // message from learning server
    if (msg->getFrom() == config().get("LS_ID")) {
        SchemaMessage * sm = dynamic_cast<SchemaMessage*>(msg);

        logger().debug("OAC::%s - Got msg from LS: '%s'", __FUNCTION__, msg->getPlainTextRepresentation());

        // sanity check to see if LS does not return an empty
        // ComboSchema
        if (sm->getComboSchema().empty()) {

            logger().warn(
                         "OAC - Received an empty ComboSchema fom LS. Discarding it.");
            return false;

        } else {

            // add schema to combo repository
            //check first if a procedure of that name already exists an remove it
            //if so
            //WARNING : if there was dependencies involving that procedure
            //then removing it is going to generate an invalid procedure_call pointer
            if (procedureRepository->contains(sm->getSchemaName()))
                procedureRepository->remove(sm->getSchemaName());

            //note that if the type is infered and checked
            //it doesn't matter what arity is specified (here 0)
            //because it is going to be overwrite with at the type check
            bool tc = config().get_bool("TYPE_CHECK_LOADING_PROCEDURES");
            arity_t a = infer_arity(sm->getComboSchema());
            procedureRepository->add(ComboProcedure(sm->getSchemaName(),
                                                    a, sm->getComboSchema(),
                                                    tc));
        }

        if (sm->getType() == SchemaMessage::_schemaMsgType)  {
            // learning is finished, set pet to PLAYING state. This
            // design ensure that the learning info will not be lost
            // until a learned schema is received
            // NOTE: transfered back to when "stop learning" instruction
            // is processed so that OAC does not stay in Lerning mode
            // forever if LS crashes...
            //pet->setMode(PLAYING);

            // Add schema to RuleEngine learned schemata
//            ruleEngine->addLearnedSchema( sm->getSchemaName( ) );
        }
        else if (sm->getType() == SchemaMessage::_schemaCandMsgType)  {
            // Add schema to RuleEngine learned schemata ...
//            ruleEngine->addLearnedSchema( sm->getSchemaName( ) );

            // .. and execute it
            pet->setTriedSchema(sm->getSchemaName());
//            ruleEngine->tryExecuteSchema( sm->getSchemaName( ) );

        }
        else {
            logger().error(
                         "Not a SCHEMA or CANDIDATE_SCHEMA message!!!");
        }
    }
    return false;
}

void OAC::schemaSelection()
{
    logger().fine("OAC - Executing selectSchemaToExecute().");

    this->pet->getCurrentModeHandler( ).update( );

//  if ( pet->getMode( ) != PLAYING && pet->getMode( ) != LEARNING ) {
//    pet->setMode( PLAYING );
//  } // if
}

const std::string OAC::getPath(const std::string& petId, const std::string& filename)
{
    std::string path;

    std::string base = config().get("PET_DATABASE");
    expandPath(base);

    logger().debug("OAC - Pet database directory: %s", base.c_str());
    path.append(base);
    path.append("/pet_");
    path.append(petId);

    // no empty string
    if (filename.size() > 0) {
        path.append("/");
        path.append(filename);
    }

    logger().debug("OAC - getPath: " + path);

    return path;
}

}} // ~namespace opencog::oac
