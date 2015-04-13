/*
 * tests/embodiment/Control/PerceptionActionInterface/AvatarInterfaceMock.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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
#ifndef PET_INTERFACE_MOCK_H_
#define PET_INTERFACE_MOCK_H_

#include "AvatarInterfaceMock.h"
#include <opencog/util/misc.h>
#include <opencog/embodiment/Learning/behavior/BE.h>
#include <opencog/embodiment/Learning/behavior/BDTracker.h>
#include <opencog/embodiment/Learning/behavior/PAIWorldProvider.h>
#include <opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h>
#include <opencog/embodiment/AtomSpaceExtensions/makeVirtualAtom.h>
#include <opencog/embodiment/Control/AvatarInterface.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAIUtils.h>
#include <opencog/embodiment/Control/AgentModeHandler.h>

#include <cassert>

using namespace opencog;
using namespace opencog::pai;
using namespace opencog::spatial;
using namespace behavior;

using namespace std;

class DummyAgentModeHandler : public control::AgentModeHandler
{
public:
    DummyAgentModeHandler( void ) : modeName( "DUMMY" ) {}
    virtual ~DummyAgentModeHandler( void ) { }
    void handleCommand( const std::string& name, const std::vector<std::string>& arguments ) {
    }
    inline const std::string& getModeName( void ) {
        return this->modeName;
    }
    inline void update( void ) { }
private:
    std::string modeName;
};

class AvatarInterfaceMock: public control::AvatarInterface
{

    std::pair<std::string, spatial::Point> latestGotoTarget;
    string pet;
    string petName;
    string ownerId;
    string exemplarAvatarId;
    string grabbedObjId;
    PAI* pai;
    bool isLearning;
    set<string> learningWhat;
    bool exemplarInProgress;
    string whatExemplar;
    unsigned long exemplarStartTimestamp;
    unsigned long exemplarEndTimestamp;
    string traits;
    string type;
    DummyAgentModeHandler modeHandler;

    // Method copied from Pet.cc on September 14th, 2007
    void executeBehaviorEncoder(const char* learningSchema) {
        string pet_id = "pet_id";

        AtomSpace& atomSpace = getAtomSpace();

        // Define the behavior interval
        Temporal exemplarTimeInterval(exemplarStartTimestamp, exemplarEndTimestamp);
        Handle trickConceptNode = atomSpace.addNode(CONCEPT_NODE, learningSchema);
        Handle trickExemplarAtTimeLink = atomSpace.getTimeServer().addTimeInfo(
                trickConceptNode, exemplarTimeInterval);
        BehaviorEncoder encoder(new PAIWorldProvider(pai), pet_id, trickExemplarAtTimeLink, 1);

        // Adds the inheritance link as Ari asked
        Handle exemplarConceptNode = atomSpace.addNode(CONCEPT_NODE, "exemplar");
        HandleSeq inhLinkHS;
        inhLinkHS.push_back(trickConceptNode);
        inhLinkHS.push_back(exemplarConceptNode);
        atomSpace.addLink(INHERITANCE_LINK, inhLinkHS);

        // position tracker
        atom_tree *positionTemplate =
            makeVirtualAtom(EVALUATION_LINK,
                            makeVirtualAtom(atomSpace.addNode(PREDICATE_NODE, AGISIM_POSITION_PREDICATE_NAME), NULL),
                            makeVirtualAtom(NODE, NULL), NULL
                           );

        encoder.addBETracker(*positionTemplate, new ActionBDTracker(&atomSpace));

        // action tracker
        atom_tree *actionTemplate =
            makeVirtualAtom(EVALUATION_LINK,
                            makeVirtualAtom(atomSpace.addNode(PREDICATE_NODE, ACTION_DONE_PREDICATE_NAME), NULL),
                            makeVirtualAtom(NODE, NULL),
                            NULL
                           );

        encoder.addBETracker(*actionTemplate, new MovementBDTracker(&atomSpace));

        // TODO: Use the exemplarEndTimestamp as well -- the current BehaviorEncoder considers the "NOW" as end of the examplar interval.
        Temporal startTime(exemplarStartTimestamp);
        encoder.update(startTime);
    }

public:
    AvatarInterfaceMock() {
        pet = PAIUtils::getInternalId("1");
        petName = "no_name";
        //ownerId = "no_owner_id";
        ownerId = PAIUtils::getInternalId("2");
        exemplarInProgress = false;
        grabbedObjId.assign("");
        type = "pet";
        traits = "Maxie";
        isLearning = false;
        pai = NULL;
    }

    AvatarInterfaceMock(string _pet, string _petName, string _ownerId) {

        pet = _pet;
        petName = _petName;
        ownerId = _ownerId;

        exemplarInProgress = false;
        isLearning = false;
        pai = NULL;
    }

    void setPAI(PAI* _pai) {
        pai = _pai;
    }

    const string& getPetId() const {
        return pet;
    }
    AtomSpace& getAtomSpace() {
        return pai->getAtomSpace();
    }

    void stopExecuting(const vector<string> &commandStatement, unsigned long timestamp) {
        cout << "AvatarInterfaceMock: stop executing '" << commandStatement.front() << "' at " << timestamp << endl;
    }

    bool isInLearningMode() const {
        return isLearning;
    }
    void startLearning(const vector<string> &commandStatement, unsigned long timestamp) {
        cout << "AvatarInterfaceMock: start learning '" << commandStatement.front() << "' at " << timestamp << endl;
        isLearning = true;
        learningWhat.insert(commandStatement.front());
    }
    void stopLearning(const vector<string> &commandStatement, unsigned long timestamp) {
        cout << "AvatarInterfaceMock: stop learning '" << commandStatement.front() << "' at " << timestamp << endl;
        isLearning = false;
        learningWhat.erase(commandStatement.front());
    }

    bool isExemplarInProgress() const {
        return exemplarInProgress;
    }
    void startExemplar(const vector<string> &commandStatement, unsigned long timestamp) {
        if (exemplarInProgress) {
            endExemplar(vector<string>(1, whatExemplar), timestamp);
        }
        exemplarInProgress = true;
        whatExemplar = commandStatement.front();
        cout << "AvatarInterfaceMock: Exemplar of '" << whatExemplar << "' started at " << timestamp << " done by " << exemplarAvatarId << endl;
        exemplarStartTimestamp = timestamp;
    }
    void endExemplar(const vector<string> &commandStatement, unsigned long timestamp) {
        // an exemplar should be in progress
        assert(exemplarInProgress);

        cout << "AvatarInterfaceMock: Exemplar of '" << whatExemplar << "' ended at " << timestamp << " done by " << exemplarAvatarId << endl;
        exemplarEndTimestamp = timestamp;
        exemplarInProgress = false;

        executeBehaviorEncoder(commandStatement.front().c_str());
    }

    void trySchema(const vector<string> &commandStatement, unsigned long timestamp) {
        cout << "AvatarInterfaceMock: TrySchema!" << endl;
    }

    void reward(unsigned long timestamp) {
        cout << "AvatarInterfaceMock: Rewarding pet!" << endl;
    }
    void punish(unsigned long timestamp) {
        cout << "AvatarInterfaceMock: Punishing pet!" << endl;
    }

    control::AgentModeHandler& getCurrentModeHandler( void ) {
        return modeHandler;
    }


    void boostSGILink(Handle sgiLink) {
        cout << "AvatarInterfaceMock: boostSGILink(" << sgiLink << ") called!" << endl;
    }

    void setOwnerId(const string& _ownerId) {
        ownerId = _ownerId;
    }
    void setExemplarAvatarId(const string& _exemplarAvatarId) {
        exemplarAvatarId = _exemplarAvatarId;
    }
    const string& getOwnerId() const {
        return ownerId;
    }
    const string& getExemplarAvatarId() const {
        return exemplarAvatarId;
    }
    void setName(const string& _petName) {
        petName = _petName;
    }
    const string& getName() const {
        return petName;
    }

    void getAllActionsDoneInATrickAtTime(const Temporal& recentPeriod, HandleSeq& actionsDone) {}
    void getAllObservedActionsDoneAtTime(const Temporal& recentPeriod, HandleSeq& behaviourDescriptions) {}
    bool isNear(const Handle& objectHandle) {
        return false;
    }
    bool getVicinityAtTime(unsigned long timestamp, HandleSeq& petVicinity) {
        return true;
    }
    void getHighLTIObjects(HandleSeq& highLTIObjects) {};
    void setLatestGotoTarget( const std::pair<std::string, spatial::Point>& target ) {
        cout << "AvatarInterfaceMock: setLatestGotoTarget!" << endl;
    }
    const std::pair<std::string, spatial::Point>& getLatestGotoTarget( void ) {
        cout << "AvatarInterfaceMock: getLatestGotoTarget!" << endl;
        return this->latestGotoTarget;
    }
    void setRequestedCommand(string command, vector<string> parameters) {}

    // functions used to set, get and verify if Pet has something in its
    // mouth, i.e., if it has grabbed something
    void setGrabbedObj(const string& id) {
        grabbedObjId = id;
    }

    const std::string& getGrabbedObj() {
        return grabbedObjId;
    }

    bool hasGrabbedObj() {
        return !grabbedObjId.empty();
    }

    const std::string& getTraits( void ) const {
        return traits;
    }
    const std::string& getType( void ) const {
        return type;
    }

};

#endif /*PET_INTERFACE_MOCK_H_*/
