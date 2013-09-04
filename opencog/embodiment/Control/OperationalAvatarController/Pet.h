/*
 * opencog/embodiment/Control/OperationalAvatarController/Pet.h
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

#ifndef PET_H
#define PET_H

#include <time.h>
#include <string>
#include <exception>

#include <opencog/atomspace/AtomSpace.h>

#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionPlan.h>
#include <opencog/embodiment/Control/AvatarInterface.h>

#include <opencog/spatial/MapExplorerServer.h>

#include "MessageSender.h"

using namespace opencog;
using namespace opencog::pai;
using namespace opencog::control;

namespace opencog { namespace oac {

// PetMode
enum PetMode {
    LEARNING,
    PLAYING,
    SCAVENGER_HUNT
};

/**
 *
 */
class Pet : public AvatarInterface
{

private:    

    // pet metadata
    std::string petId;
    std::string petName;
    std::string agentTraits;
    std::string ownerId;
    std::string demandName; // Currently selected demand name 

    // NOTE: pet class will became a generic Agent class so that it could be
    // used by pets or humanoids. So the type used here.
    std::string agentType;

    double rayOfVicinity;

    PetMode mode;
    std::map<PetMode, AgentModeHandler*> modeHandler;

    // oac components received as constructor parameter
    AtomSpace* atomSpace;
    MessageSender* sender;

    PAI* pai;

    //! The ID of the avatar that is performing the exemplars
    std::string exemplarAvatarId;
    //! Last schema tried
    std::string triedSchema;
    //! Schema being learned
    std::vector<std::string> learningSchema;

    //! Start timestamp to get behaviors from the AtomSpace
    unsigned long exemplarStartTimestamp;
    //! End timestamp to get behaviors from the AtomSpace
    unsigned long exemplarEndTimestamp;

    unsigned long startLearningSessionTimestamp;
    unsigned long endLearningSessionTimestamp;

    bool candidateSchemaExecuted;

    /** the id of the object the pet has on its mouth, empty if there is no
     * object grabbed. 
     */
    std::string grabbedObjId;

    opencog::RandGen* rng;

    /** Populate atom space with behavior nodes
     */
    void executeBehaviorEncoder();

    /**
     * Update the maps that should be kept (persisted) in spaceServer during the
     * last exemplar session.
     */
    void updatePersistentSpaceMaps() throw (opencog::RuntimeException, std::bad_exception);

    /**
     * Update is_exemplar_avatar predicate for the pet based on exemplarAvatarId
     * object.
     *
     * @param active Inform if the predicate is active or not. An active
     * predicate has its TV set to 1.0 and 0.0 otherwise.
     */
    void adjustIsExemplarAvatarPredicate(bool active) throw (opencog::RuntimeException);

    opencog::spatial::MapExplorerServer* visualDebuggerServer;

public:

    static const unsigned long UNDEFINED_TIMESTAMP;

    /**
     * A requested command is a trick sent to the pet to be executed
     */
    class RequestedCommand
    {
    public:
        inline RequestedCommand( void ) : name( "" ), readed( true ) { }
        virtual inline ~RequestedCommand( void ) { }

        std::string name;
        std::vector<std::string> arguments;
        bool readed;
    };

    /**
     * Pet constructor.
     *
     * @param petID The SL id of the pet.
     * @param name The name the pet.
     * @param agentType The agent type (pet or humanoid)
     * @param agentTraits The agent traits type
     * @param ownerID The SL id of the owner of the pet.*
     * @param atomSpace A atomSpace with the Pet short memory.
     */
    Pet(const std::string& petId, const std::string& petName, const std::string&
            agentType, const std::string& agentTraits, const std::string&
            ownerID, AtomSpace* atomSpace, MessageSender* sender);
    ~Pet();

    /** Set the PAI of this Pet
     */
    void setPAI(PAI* pai);

    /** Get the PAI of this Pet
     */
    inline PAI& getPai( void ) {
        return *pai;
    }

    /** Init AtomSpace with pet traits, feelings and other atoms
     */
    void initTraitsAndFeelings();

    /** Get the pet name.
     *
     * @return Pet's name.
     */
    const std::string& getName() const;

    /** Set the pet name.
     *
     * @param petName new pet name.
     */
    void setName(const std::string& petName);

    /** Getter for agent type.
     * @note An agent should never have its type changed
     * @return Agent type
     */
    const std::string& getType() const;

    /** Getter for pet id.
     * @note A pet should never have its id changed
     */
    const std::string& getPetId() const;

    /** Getter for owner id.
     */
    const std::string& getOwnerId() const;

    /** Setter for owner id.
     */
    void setOwnerId(const std::string& ownerId);

    /**
     * Getter and setter for currently selected demand name
     */
    const std::string & getDemandName() const;
    void setDemandName(const std::string & demandName);

    const std::string& getExemplarAvatarId() const;

    /**
     * @warning always use this method, even inside the class because it
     * controls the is_teacher predicate for the pet
     */
    void  setExemplarAvatarId(const std::string& exemplarAvatarId);

    /** Get pet mode
     * @return (PLAYING or LEARNING).
     */
    const PetMode getMode() const;

    /** Set pet mode
     * @param petMode (PLAYING or LEARNING)
     */
    void  setMode(PetMode);

    /** Get the tried schemata names
     */
    const std::string & getTriedSchema();

    void setTriedSchema(const std::string & triedSchema);

    /** Get the learning schemata names
     */
    const std::vector<std::string> & getLearningSchema();

    unsigned long getExemplarStartTimestamp();
    unsigned long getExemplarEndTimestamp();

    /** Restart a learning process when the LS goes down.
     *
     * This method should ONLY be called when the LS goes up again and the Pet
     * is still in LEARNING mode.
     */
    void restartLearning() throw (opencog::RuntimeException,
            std::bad_exception);

    /**
     * Create a new Pet loading pet metadata from file.
     *
     * @param filename The name of the file with the pet metadata.
     * @param petId The pet identification number.
     * @param atomSpace An AtomSpace containing the pet short memory.
     *
     * @return The pet newly created.
     */
    static Pet* importFromFile(const std::string& filename, const std::string&
            petId, AtomSpace* atomSpace, MessageSender* sender);

    /**
     * Save pet metadata into file.
     *
     * @param filename The name of the file where data will be written.
     * @param pet The pet whose metadata will be written.
     */
    static void exportToFile(const std::string& filename, Pet & pet) throw
        (opencog::IOException, std::bad_exception);

    // IMPLEMENTATION OF METHODS OF AvatarInterface (getPetId() is already defined
    // above):

    AtomSpace& getAtomSpace();

    void stopExecuting(const std::vector<std::string> &commandStatement,
            unsigned long timestamp);

    bool isInLearningMode() const;
    void startLearning(const std::vector<std::string> &commandStatement,
            unsigned long timestamp);
    void stopLearning(const std::vector<std::string> &commandStatement,
            unsigned long timestamp);

    bool isExemplarInProgress() const;
    void startExemplar(const std::vector<std::string> &commandStatement,
            unsigned long timestamp);
    void endExemplar(const std::vector<std::string> &commandStatement, unsigned
            long timestamp);

    void trySchema(const std::vector<std::string> &commandStatement, unsigned
            long timestamp);
    void reward(unsigned long timestamp);
    void punish(unsigned long timestamp);

    AgentModeHandler& getCurrentModeHandler( void );

    float computeWalkingSpeed() const;

    const std::string& getTraits( void ) const;

    unsigned long getLatestRewardTimestamp( void );
    unsigned long getLatestPunishmentTimestamp( void );

    void getAllActionsDoneInATrickAtTime(const Temporal& time,
            HandleSeq& actionsDone);
    void getAllObservedActionsDoneAtTime(const Temporal& time,
            HandleSeq& actionsDone);
    bool isNear(const Handle& objectHandle);
    //bool getVicinityAtTime(unsigned long timestamp, HandleSeq& petVicinity);
    void getHighLTIObjects(HandleSeq& highLTIObjects);

    /**
     * @see AvatarInterface::setLatestGotoTarget
     */
    void setLatestGotoTarget(
            const std::pair<std::string, SpaceServer::SpaceMapPoint>& target ) {
        this->targetObject = target;
    };

    /**
     * @see AvatarInterface::getLatestGotoTarget
     */
    const std::pair<std::string, SpaceServer::SpaceMapPoint>& getLatestGotoTarget( void ) {
        return this->targetObject;
    };

    void setRequestedCommand(std::string command,
            std::vector<std::string> parameters);

    inline RequestedCommand& getLatestRequestedCommand( void ) {
        lastRequestedCommand.readed = true;
        return lastRequestedCommand;
    };

    inline bool isRequestedCommandNotReaded( void ) {
        return !lastRequestedCommand.readed;
    }

    /**
     * Callback method called by RuleEngine to indicate the candidate schema was
     * selected to be executed
     */
    void schemaSelectedToExecute(const std::string& schemaName);

    Handle getMyHandle() const;

    // functions used to set, get and verify if Pet has something in its
    // mouth, i.e., if it has grabbed something

    /**
     * Set the id of the grabbed object, i.e., the object that is in pet's
     * mouth
     *
     * @param id The object id
     */
    void setGrabbedObj(const std::string& id);

    /**
     * Return the id of the grabbed object, i.e., the object that is in pet's
     * mouth
     *
     * @return The object id
     */
    const std::string& getGrabbedObj();

    /**
     * Return true if grabbed object name is different from an empty string,
     * false otherwise.
     */
    bool hasGrabbedObj();

    RequestedCommand lastRequestedCommand;

    unsigned long latestRewardTimestamp;
    unsigned long latestPunishmentTimestamp;

    // target object used by goto and gonear combo functions
    std::pair<std::string, SpaceServer::SpaceMapPoint> targetObject;

    /**
     * Visual debugger is a service that let the user to see, through a 3D client,
     * the current state of the agent LocalSpaceMap. This method starts a server
     * that will be used to send the LocalSpaceMap for each connected client.
     *
     * @param host Host used to bind the server
     * @param port Port used to bind the server
     */
    void startVisualDebuggerServer( const std::string& host, const std::string& port );

    /**
     * This function will stop the visual debugger server, if called
     */
    void stopVisualDebuggerServer( void );  

    /**
     * Updates the LocalSpaceMap used by the visual debugger clients to render
     * the environemtn.
     *
     * @param map Latest LocalSpaceMap
     */
    void sendMapToVisualDebuggerClients( const SpaceServer::SpaceMap & map );

}; // class

} } // namespace opencog::oac

#endif
