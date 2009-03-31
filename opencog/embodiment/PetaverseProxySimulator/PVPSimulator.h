/*
 * opencog/embodiment/PetaverseProxySimulator/PVPSimulator.h
 *
 * Copyright (C) 2007-2008 Andre Senna
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

#ifndef PVPSIMULATOR_H
#define PVPSIMULATOR_H

#include <xercesc/dom/DOM.hpp>
#include <xercesc/framework/MemBufInputSource.hpp>

#include <xercesc/framework/Wrapper4InputSource.hpp>
#include <xercesc/validators/common/Grammar.hpp>
#include <xercesc/validators/schema/SchemaGrammar.hpp>
#include <xercesc/util/XMLUni.hpp>

#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include <string>
#include <map>
#include <set>

#include <EmbodimentCogServer.h>
#include <Message.h>
#include <ActionParamType.h>
#include <ActionType.h>
#include <exception>

#include "WorldSimulator.h"
#include "PhysiologicalModel.h"

#include "AsynchronousPerceptionAndStatusHandler.h"
#include "util/exceptions.h"
#include "SimProxy.h"

#include "GoldStdGen.h"

#define TEST_TICKET ULONG_MAX - 1

namespace PetaverseProxySimulator {

    class PVPSimulator : public MessagingSystem::EmbodimentCogServer, AsynchronousPerceptionAndStatusHandler {

        private:

            typedef std::map<unsigned long, std::string> ActionTicketToPlanIdMap;
            typedef std::map<unsigned long, std::string> ActionTicketToPetIdMap;
            typedef std::map<unsigned long, PerceptionActionInterface::PetAction> ActionTicketToActionMap;
            typedef std::map<std::string, std::string> ObjTimestampMap;
            typedef std::map<std::string, PerceptionActionInterface::Vector> ObjPositionMap;
            
            typedef std::list<PerceptionActionInterface::PetAction *> PetActionsList;

            typedef std::set<std::string> AvatarsIdSet;
            typedef std::set<std::string> PetsIdSet;
            typedef std::map<std::string, bool> PetsIdMap;
            typedef std::map<std::string, std::string> OwnershipMap;

            typedef std::map<std::string, PetActionsList> PetActionsListMap;
            typedef std::map<std::string, std::string> CurrentPlanIdMap;
            typedef std::map<std::string, PhysiologicalModel *> PetsPhysiologicalModelMap;

            static const int PET_SIGNAL_SENDING_INTERVAL;
            static const char* DEFAULT_OWNER_ID;

            AvatarsIdSet avatarsId;
            PetsIdMap petsId;
            PetsIdSet recoveryPetsId;
            OwnershipMap ownership;


            // list of all actions to be executed
            PetActionsListMap petActionsList;
            
            // the id of the plan currently being executed
            CurrentPlanIdMap currentPlanId;

            WorldSimulator *worldSimulator;
            PetsPhysiologicalModelMap petsPhysiologicalModel;

            bool firstMessageFlag;

            GoldStdGen* goldStdGen; 
            
            /**
             * Process all actions within an action plan. One or more of these
             * actions may need to be executed oin the server and then we need
             * to wait its response to continue execution.
             */
            void processPetActions(const std::string& petId);

            /**
             * Cancel the action plan being executed. Most of the time this is
             * due to the arrival of a new action plan.
             */
            void cancelActionPlan(const std::string& petId);

            void parseActionPlan(XERCES_CPP_NAMESPACE::DOMElement *actionPlan);
            void parseDOMDocument(XERCES_CPP_NAMESPACE::DOMDocument *document);
            bool parseXML(const std::string& xmlText);
            char *getNextToken(char *cursor, char *target);
            void addActionParameters(PerceptionActionInterface::PetAction& action, std::string &xmlText, char * &cursor, PerceptionActionInterface::ActionType &actionType, std::vector<PerceptionActionInterface::ActionParamType> &typeList, bool required, unsigned int paramOffset);
            std::string getCurrentTimestamp();

            std::string PVP_ID;

            SimulationParameters* simParams;

            void initialize();

            int tickCount;

            ActionTicketToPlanIdMap ticketToPlanIdMap;
            ActionTicketToActionMap ticketToActionMap;
            ActionTicketToPetIdMap ticketToPetId;

            pthread_mutex_t currentTimeLock; 
            MessagingSystem::MemoryMessageCentral messagesToSend;

            ObjTimestampMap objTimestamp;
            ObjPositionMap objPosition;
            ObjPositionMap objPreviousPosition;

        public:

            static const char* DEFAULT_PET_ID;

            // ***********************************************/
            // Constructors/destructors

            static opencog::BaseServer* createInstance();
            ~PVPSimulator();
            PVPSimulator();
            void init(SimulationParameters&);
            void init(const Control::SystemParameters &params, SimulationParameters& simParams, const std::string &myId, const std::string &ip, int portNumber);

            // ***********************************************/
            // API

            /**
             * Called when NE retrieve a Message from router to perform some useful proceesing on it. Subclasses
             * are supposed to override this to perform something useful (default implementation just outputs
             * Message contents to stdout).
             */
            virtual bool processNextMessage(MessagingSystem::Message *message);

            /**
             * This method is overriden just for generating gold standards for automated tests
             */
            virtual bool sendMessage(MessagingSystem::Message &msg);

            /**
             * Called to indicate that one time tick of simulation time has passed.
             */
            void timeTick();

            /**
             * Called by MessageSenderTask to send the enqueued messages to PB.
             */
            void sendMessages();

            bool sendAgentAction(char *txt, const char *avatarId, const char* agentType );

            std::string createAvatar(const std::string& avatarId, float x, float y);
            std::string createPet(const std::string& petId, const std::string& ownerId, float x, float y);

            void sendPredavese(const char *txt, std::string petId);
            bool sendOwnerAction(char *txt);
            void sendPetSignals();
            void sendActionStatusPetSignal(const std::string& planId, const std::string& petId, const PerceptionActionInterface::PetAction& action, bool success);

            void sendActionStatusPetSignal(const std::string& planId, const std::string& petId, bool success);

            void setWorldSimulator(WorldSimulator *worldSimulator);
            void setPhysiologicalModel(PhysiologicalModel *physiologicalModel);

            bool connectToSimWorld();
            bool loadPet(const std::string& petId);
            void resetPhysiologicalModel(string petId);

            // ***********************************************/
            // Methods of the AsynchronousPerceptionAndStatusHandler interface:

            /**
             * Receives an object metadata from WorldSimulator
             */
            void mapInfo(std::vector<ObjMapInfo>& objects);

            /**
             * Receives a notification of action status from WorldSimulator
             */
            void actionStatus(unsigned long actionTicket, bool success);

            /**
             * Receives a notification of an error from WorldSimulator
             */
            void errorNotification(const std::string& errorMsg);

            // ***********************************************/
            // Tests/Debug 
            //
            // Althogh the following methods are public, they are not part of the API, thus
            // THEY ARE NOT TO BE TRUSTED

            void publicParseXML(const std::string& xmlText);
            
            /**
             * Routines to persiste and recovery states
             */
            void persistState() throw (opencog::IOException, std::bad_exception);
            void recoveryFromPersistedData(const std::string& fileName);

    }; // class
}  // namespace

#endif
