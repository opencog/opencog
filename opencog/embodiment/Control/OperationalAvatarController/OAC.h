/*
 * opencog/embodiment/Control/OperationalAvatarController/OAC.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
 *
 * Updated: By Zhenhua Cai, on 2011-06-01
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

#ifndef OAC_H
#define OAC_H

#include <string>
#include <opencog/persist/file/SavingLoading.h>
#include <opencog/util/RandGen.h>
#include <opencog/embodiment/Control/MessagingSystem/StringMessage.h>
#include <opencog/embodiment/Control/MessagingSystem/EmbodimentCogServer.h>
#include <opencog/embodiment/Control/PredicateUpdaters/PredicatesUpdater.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <opencog/embodiment/Control/Procedure/ProcedureRepository.h>
#include <opencog/embodiment/Control/Procedure/ProcedureInterpreter.h>

#include "Pet.h"
#include "PetMessageSender.h"
#include "PVPActionPlanSender.h"
#include "Plaza.h"

#include <opencog/embodiment/Control/OperationalAvatarController/ProcedureInterpreterAgent.h>
#include <opencog/embodiment/Control/OperationalAvatarController/ActionSelectionAgent.h>
#include <opencog/embodiment/Control/OperationalAvatarController/ImportanceDecayAgent.h>
#include <opencog/embodiment/Control/OperationalAvatarController/EntityExperienceAgent.h>

#include <opencog/embodiment/Control/OperationalAvatarController/PsiModulatorUpdaterAgent.h>
#include <opencog/embodiment/Control/OperationalAvatarController/PsiDemandUpdaterAgent.h>
#include <opencog/embodiment/Control/OperationalAvatarController/PsiActionSelectionAgent.h>
#include <opencog/embodiment/Control/OperationalAvatarController/PsiRelationUpdaterAgent.h>
#include <opencog/embodiment/Control/OperationalAvatarController/PsiFeelingUpdaterAgent.h>

#include "RuleEngine.h"

class PsiModulatorUpdaterAgentUTest; 
class PsiDemandUpdaterAgentUTest;
class PsiFeelingUpdaterAgentUTest; 
class PsiActionSelectionAgentUTest; 

namespace OperationalAvatarController
{

/* Defines a single factory template to allow insert a same agent
 * multiple times in the Cogserver schedule */
template< typename _Type, typename _BaseType >
class SingletonFactory : public Factory<_Type, _BaseType>
{
public:
    explicit SingletonFactory() : Factory<_Type, _BaseType>() {}
    virtual ~SingletonFactory() {}
    virtual _BaseType* create() const {
        static _BaseType* inst =  new _Type;
        return inst;
    }
};

class OAC : public EmbodimentCogServer
{
    friend class::PsiModulatorUpdaterAgentUTest; 
    friend class::PsiDemandUpdaterAgentUTest;
    friend class::PsiFeelingUpdaterAgentUTest; 
    friend class::PsiActionSelectionAgentUTest; 

private:

    /**
     * Object used to save/load atomSpace dumps. Other componentes that
     * need persistence should extend SavableRepository interface and
     * add thenselfs to this savingLoading object.
     */
    SavingLoading savingLoading;

    /**
     * A repository to all combo and built-ins procedures
     */
    Procedure::ProcedureRepository* procedureRepository;

    /**
     * Component to deal with perceptions and actions from PVP proxy.
     */
    PerceptionActionInterface::PAI * pai;

    /**
     * Store metadata concerning a pet controled by the OAC. Also implements
     * an interface to comunicate with the LS.
     */
    Pet * pet;

#ifdef HAVE_ZMQ    
    /**
     * Customized ZeroMQ device used by mind agents to publish messages
     */
    Plaza * plaza;
#endif // HAVE_ZMQ

    /*
     * Random generator
     */
    static boost::shared_ptr<RandGen> rngPtr; 

    /**
     * Interpreter for the procedures to be executed by the OAC. Can
     * deal with combo and builtin procedures
     */
    Procedure::ProcedureInterpreter * procedureInterpreter;

    /**
     * Message sender for action plans. Used by PAI component
     */
    PVPActionPlanSender * planSender;

    /**
     * Message sender for learning comands, tricks. Used by Pet component
     */
    PetMessageSender * petMessageSender;

    /**
     * Update predicates necessary to the execution of built-ins procedures.
     */
    PredicatesUpdater * predicatesUpdater;

    /** opencog Agents */
    ProcedureInterpreterAgent* procedureInterpreterAgent;
    ImportanceDecayAgent* importanceDecayAgent;
//    ActionSelectionAgent* actionSelectionAgent;
    EntityExperienceAgent* entityExperienceAgent;

    PsiModulatorUpdaterAgent * psiModulatorUpdaterAgent;
    PsiDemandUpdaterAgent * psiDemandUpdaterAgent;
    PsiActionSelectionAgent * psiActionSelectionAgent;
    PsiRelationUpdaterAgent * psiRelationUpdaterAgent; 
    PsiFeelingUpdaterAgent * psiFeelingUpdaterAgent; 

    RuleEngine* ruleEngine;

    /**
     * Load pet metadata for a given pet.
     *
     * IMPORTANT: This method should be invoke only in OAC start up, i.e.
     * in the constructor.
     */
    void loadPet(const std::string & petId);

    /**
     * Load the AtomSpace and other repositories (TimeServer, SpaceServer,
     * ProcedureRepository, etc).  Before invoking this method the
     * SavableRepository objects must be created and registered
     * (this may be done at the constructor).
     *
     * IMPORTANT: This method should be invoke only in OAC start up, i.e.
     * in the constructor.
     */
    void loadAtomSpace(const std::string & petId);

    /**
     * Process messages from the spwaner componente. Usually a SAVE_AND_EXIT
     * one.
     *
     * @param spawnerMessage The spawner message plain text format to be
     *                          processed
     *
     * @return True if the message was correctly processed
     */
    bool processSpawnerMessage(const std::string & spawnerMessage);

    /**
     * Configure the pet objet to be persisted. Actions executed includes:
     * - dropping grabbed itens, if any;
     * - finishing learning sessions, if started
     */
    void adjustPetToBePersisted();

public:

    static opencog::BaseServer* createInstance();

    /**
     * Constructor and destructor
     */
    OAC();
    ~OAC();

    virtual bool customLoopRun(void);

    void init(const std::string &myId, const std::string &ip, int portNumber,
              const std::string & zmqPublishPort,
              const std::string& petId, const std::string& ownerId,
              const std::string& agentType, const std::string& agentTraits);

    /**
     * Add Rules, including Modulators, DemandGoals etc. into AtomSpace. 
     *
     * For details about formats of Modulators, DemandGoals and Rules,
     * please refer to file "rules_core.scm", 
     * and "pet_rules.scm" is a good example of using them.     
     */ 
    int addRulesToAtomSpace();

    /**
     * Save the OCP state.
     *
     * IMPORTANT: This method should be invoke only on OAC shutdown,
     *            that is, when it receive a shutdown message from the
     *            spawner. Or when an exception has occured and the state
     *            should be persisted.
     */
    void saveState();

    /**
     * @return The Percpetion/Action Interface object used to exchange
     *            (send and receive) data with virtual world.
     */
    PerceptionActionInterface::PAI & getPAI();

    /**
     * Return the pet's cognitive component. This component is responsible
     * for updating the pet AttentionValues and for choosing a schema to
     * be executed.
     *
     * @return The pet's cognitive component.
     */
    Pet & getPet();

#ifdef HAVE_ZMQ    
    /**
     * Return the customized ZeroMQ device used by mind agents to publish messages
     */
    Plaza & getPlaza() {
        return * plaza;
    }
#endif // HAVE_ZMQ

    /**
     * Return the reference to rand generator
     */ 
    RandGen & getRandGen();

    /**
     * Get the RuleEngine associated with the OAC.
     *
     * @return The RuleEngine associated with the OAC.
     */
    RuleEngine & getRuleEngine();

    /**
     * Get the Procedure Interpreter associated with the OAC.
     *
     * @return The ProcedureInterpreter associated with the OAC.
     */
    Procedure::ProcedureInterpreter & getProcedureInterpreter();


    /**
     * Get the Procedure Repository associated with the OAC.
     *
     * @return The ProcedureRepository associated with the OAC.
     */
    Procedure::ProcedureRepository & getProcedureRepository( );


    /**
      * @return A reference to the PVPActionPlanSender of this OAC
      */
    PVPActionPlanSender & getPlanSender();

    /**
     * Method inherited from EmbodimentCogServer 
     * @return True if the server should exit.
     */
    bool processNextMessage(MessagingSystem::Message *msg);

    /**
     * Method inherited from EmbodimentCogServer
     */
    void setUp();

    /**
      * Selects a pet schema to be executed. This schema will be interpreted
      * and sent to Proxy via PAI.
      */
    void schemaSelection();

    /**
      * Decays short term importance of all atoms in local AtomSpace
      */
    void decayShortTermImportance();

    /**
     * Return the path to a file or a directory
     *
     * @param petId The id of the pet used to produce the file path
     * @param filename The name of the file to get the path
     *
     * @return The string representing the path to the given file
     */
    const std::string getPath(const std::string & petId, const std::string & filename = "");

    SingletonFactory<ProcedureInterpreterAgent, Agent> procedureInterpreterAgentFactory;
    SingletonFactory<ImportanceDecayAgent, Agent> importanceDecayAgentFactory;
//    SingletonFactory<ActionSelectionAgent, Agent> actionSelectionAgentFactory;
    SingletonFactory<EntityExperienceAgent, Agent> entityExperienceAgentFactory;

    SingletonFactory <PsiModulatorUpdaterAgent, Agent> psiModulatorUpdaterAgentFactory;
    SingletonFactory <PsiDemandUpdaterAgent, Agent> psiDemandUpdaterAgentFactory;
    SingletonFactory <PsiActionSelectionAgent, Agent> psiActionSelectionAgentFactory;
    SingletonFactory <PsiRelationUpdaterAgent, Agent> psiRelationUpdaterAgentFactory; 
    SingletonFactory <PsiFeelingUpdaterAgent, Agent> psiFeelingUpdaterAgentFactory; 
}; // class
}  // namespace

#endif // OAC_H

