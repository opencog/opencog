/*
 * opencog/embodiment/Control/AvatarInterface.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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

#ifndef AVATAR_INTERFACE_H_
#define AVATAR_INTERFACE_H_

/**
 * AvatarInterface.h
 *
 * This is an abstract class to define the interface that the Avatar class must
 * provide for usage by other classes (like PAI, Predavese parser and handlers, etc).
 */
#include <fstream>

#include <opencog/util/Logger.h>

#include <opencog/atomspace/AtomSpace.h>

#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spacetime/Temporal.h>
#include <opencog/spacetime/SpaceTime.h>

#include <opencog/embodiment/Control/AgentModeHandler.h>


using namespace opencog;

namespace opencog { namespace control {

class AvatarInterface
{
public:
    virtual ~AvatarInterface() {}

    virtual const std::string& getPetId() const = 0;
    virtual const std::string& getExemplarAvatarId() const = 0;

    virtual AtomSpace & getAtomSpace() = 0;

    virtual void stopExecuting(const std::vector<std::string> &commandStatement, unsigned long timestamp) = 0;

    virtual bool isInLearningMode() const = 0;
    virtual void startLearning(const std::vector<std::string> &commandStatement, unsigned long timestamp) = 0;
    virtual void stopLearning(const std::vector<std::string> &commandStatement, unsigned long timestamp) = 0;

    virtual bool isExemplarInProgress() const = 0;
    virtual void startExemplar(const std::vector<std::string> &commandStatement, unsigned long timestamp) = 0;
    virtual void endExemplar(const std::vector<std::string> &commandStatement, unsigned long timestamp) = 0;

    virtual void trySchema(const std::vector<std::string> &commandStatement, unsigned long timestamp) = 0;
    virtual void reward(unsigned long timestamp) = 0;
    virtual void punish(unsigned long timestamp) = 0;

    /**
     * One handler mode shall be created to every agent mode.
     */
    virtual AgentModeHandler& getCurrentModeHandler( void ) = 0;

    virtual void setOwnerId(const std::string& ownerId) = 0;
    virtual void setExemplarAvatarId(const std::string& avatarId) = 0;
    virtual const std::string& getOwnerId() const = 0;
    virtual void setName(const std::string& petName) = 0;
    virtual const std::string& getName() const = 0;

    // functions used to set, get and verify if Pet has something in its
    // mouth, i.e., if it has grabbed something
    virtual void setGrabbedObj(const std::string& id) = 0;
    virtual const std::string& getGrabbedObj() = 0;
    virtual bool hasGrabbedObj() = 0;

    virtual void getAllActionsDoneInATrickAtTime(const Temporal& recentPeriod, HandleSeq& actionsDone) = 0;
    virtual void getAllObservedActionsDoneAtTime(const Temporal& recentPeriod, HandleSeq& behaviourDescriptions) = 0;
    virtual bool isNear(const Handle& objectHandle) = 0;
   // virtual bool getVicinityAtTime(unsigned long timestamp, HandleSeq& petVicinity) = 0;
    virtual void getHighLTIObjects(HandleSeq& highLTIObjects) = 0;

    /**
     * This method keeps the latest object name, used by goto_obj and gonear_obj
     * when building a goto plan
     * @param target Object name amn it's position on LocalSpaceMap
     */
    virtual void setLatestGotoTarget( const std::pair<std::string, SpaceServer::SpaceMapPoint>& target ) = 0;
    /**
     * Returns the latest object name used by goto_obj or gonear_obj
     * @return Object name and it's position
     */
    virtual const std::pair<std::string, SpaceServer::SpaceMapPoint>& getLatestGotoTarget( void ) = 0;

    /**
     * When an avatar requests the pet to execute a trick, this
     * method will be used to register the command on RuleEngine
     * @param command The requested trick
     * @param parameters The list of arguments of the trick
     */
    virtual void setRequestedCommand(std::string command, std::vector<std::string> parameters) = 0;
    /**
     * Computes a speed for the pet to walk at in combo schema execution
     * (possibly based on its mood & the schema its executing, possibly with
     * random variation to make it less robotic) - this is in m/s and acc to
     * Tristan this "The range of valid values for speed is between -5m/s and
     * 30m/s" (whatever the hell traveling at "-5m/s" means)
     */
    virtual float computeWalkingSpeed() const {
        return 3.5;
    }

    /**
     * Computes an angle to be the minimal rotation for pet in combo schema execution
     * (possibly based on its mood & the schema its executing, possibly with
     * random variation to make it less robotic) - in radians
     */
    virtual float computeRotationAngle() const {
        return 0.1;
    }

    /**
     * Computes a duration for following, in seconds, e.g. based on how obedient
     * / interested / whatever the pet is
     */
    virtual float computeFollowingDuration() const {
        return 5.0;
    }

    /**
     * Return the type of the Agent (pet, humanoid, etc)
     *
     * @return agent type
     */
    virtual const std::string& getType( void ) const = 0;


    /**
     * Return the personality traits of the Agent
     *
     * @return agent traits
     */
    virtual const std::string& getTraits( void ) const = 0;


    /**
     * Save a LocalSpaceMap2D copy on the current application directory
     */
    void saveSpaceMapFile() {
        logger().debug("AvatarInterface - saveSpaceMapFile().");
        if (!spaceServer().isLatestMapValid()) {
            logger().warn("AvatarInterface - There is no space map yet.");
            return;
        }
        const SpaceServer::SpaceMap& sm = spaceServer().getLatestMap();
        static unsigned int mapCounter = 0;
        std::stringstream fileName;
        fileName << "ww_mapPersistence_";
        fileName << getPetId();
        fileName << "_";
        fileName << mapCounter;
        fileName << ".txt";
        SpaceServer::SpaceMap& map = (SpaceServer::SpaceMap&) sm;
        
        std::ofstream saveFile( fileName.str( ).c_str( ) );
        saveFile << SpaceServer::SpaceMap::toString( map );
        saveFile.close( );

        ++mapCounter;
        
    }

};

}} // opencog::control

#endif /*AVATAR_INTERFACE_H_*/
