/*
 * opencog/embodiment/Control/Predavese/NominalReferenceResolver.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
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
#include "util/Logger.h"
#include "util/StringManipulator.h"

#include "NominalReferenceResolver.h"
#include "WorldWrapperUtil.h"
#include "AtomSpaceUtil.h"
#include "Predavese.h"

using namespace opencog;

NominalReferenceResolver::NominalReferenceResolver(Control::PetInterface& _petInterface) : petInterface(_petInterface) {

}

string NominalReferenceResolver::solve(const string& name, const string& speakerId, unsigned long timestamp) const {
    logger().log(opencog::Logger::DEBUG,  " %s - Name: \"%s\", timestamp: %lu.", __FUNCTION__, name.c_str(), timestamp); 

    if (name == "") 
        return name;

    set<Handle> candidates; 
    createSetOfCandidates(name, timestamp, candidates);

    scoreCandidatesMap scoredCandidates;
    scoreCandidates(name, speakerId, timestamp, candidates, scoredCandidates);

    Handle nameSelected = selectCandidate(scoredCandidates);

    return petInterface.getAtomSpace().getName(nameSelected);
}

bool NominalReferenceResolver::createSetOfCandidates(const string& name, unsigned long timestamp, set<Handle> & candidates) const {

    logger().log(opencog::Logger::DEBUG,  " %s - Name: \"%s\".", __FUNCTION__, name.c_str()); 

    // currently or recently observed by the pet
    // known according to the pet's memory to be in the pet's vicinity
    // ---> currently or recently known to be in the pet's vicinity
    HandleSeq petVicinity;

    if (!petInterface.getVicinityAtTime(timestamp, petVicinity)) 
        return false;

    unsigned long timestampOfRecently = timestamp-predavese::recentPeriodOfTime;
    if (!petInterface.getVicinityAtTime(timestampOfRecently, petVicinity))
        return false;
    
    // remove duplicate objects
    copy(petVicinity.begin(), petVicinity.end(), inserter(candidates, candidates.end()));

    // possessing particularly high long-term importance for the pet. (HLTI)
    HandleSeq highLTIObjects;
    petInterface.getHighLTIObjects(highLTIObjects);
    
    // remove duplicate objects and adding new ones.
    copy(highLTIObjects.begin(), highLTIObjects.end(), inserter(candidates, candidates.end()));

    stringstream strCandidates;
    foreach(Handle it, candidates){
        strCandidates << TLB::getAtom(it)->toString() << ", "; 
    }
    logger().log(opencog::Logger::DEBUG,  " %s - Candidates set: %s", __FUNCTION__, strCandidates.str().c_str()); 
    return true;
}

void NominalReferenceResolver::scoreCandidates(const string& name, const string& speakerId, unsigned long timestamp, const set<Handle> & candidatesSet, scoreCandidatesMap& scoredCandidates) const {

   logger().log(opencog::Logger::DEBUG,  "%s - %d candidates.", __FUNCTION__, candidatesSet.size()); 

    const AtomSpace& as = petInterface.getAtomSpace();
    const SpaceServer::SpaceMap& sm = as.getSpaceServer().getLatestMap();
    
    unsigned long timestampOfRecently;
    if ( timestamp > predavese::recentPeriodOfTime )
        timestampOfRecently = timestamp - predavese::recentPeriodOfTime;
    else timestampOfRecently = 0;

    HandleSeq actionsDoneRecentlyInATrick;
    HandleSeq observedActionsDoneRecently;

    Temporal recentlyTime(timestampOfRecently, timestamp);
    petInterface.getAllActionsDoneInATrickAtTime(recentlyTime, actionsDoneRecentlyInATrick);
    petInterface.getAllObservedActionsDoneAtTime(recentlyTime, observedActionsDoneRecently);


    Handle speakerHandle = WorldWrapper::WorldWrapperUtil::getAvatarHandle(as, speakerId);
    SpaceServer::SpaceMapPoint speakerPos = WorldWrapper::WorldWrapperUtil::getLocation(sm, as, speakerHandle);
    double speakerDir = WorldWrapper::WorldWrapperUtil::getOrientation(sm, as, speakerHandle);

    HandleSeq wordLinkHandles; 
    as.getHandleSet(back_inserter(wordLinkHandles), WR_LINK, false); 

    foreach(Handle candidate, candidatesSet) {
        
        string candidateId = as.getName(candidate);

        // name match? 
        foreach(Handle wl, wordLinkHandles) {
            if(as.getOutgoing(wl,1) == candidate) {
                string candidateName = as.getName(as.getOutgoing(wl,0));
                if(StringManipulator::toLower(candidateName) == StringManipulator::toLower(name) ) {
                    scoredCandidates[candidate] += 5; // name match
                }
                else {
                    scoredCandidates[candidate] += -3; // name mismatch
                }
            }
        }

        // speaker pointedAt?

        if(sm.containsObject(candidateId)) {
            SpaceServer::SpaceMapPoint candidatePos =  WorldWrapper::WorldWrapperUtil::getLocation(sm, as, candidate);
            SpaceServer::SpaceMapPoint candidateDir = make_pair(    candidatePos.first-speakerPos.first, 
                                                                    candidatePos.second-speakerPos.second);
            // dot product between vector and [1 0] X axis
            double angle = acos( candidateDir.first / (sqrt(pow(candidateDir.first,2)+pow(candidateDir.second,2)) ));
            double epslon = 0.001;
            if ( angle < speakerDir+epslon || angle > speakerDir-epslon ) {
                scoredCandidates[candidate] += 4;
            }
        } 

        // holding 
        // speaker holding?
        if (AtomSpaceUtil::getHoldingObjectId(as, speakerId) == candidateId) {
            scoredCandidates[candidate] += 3.5;
        }
        
        // pet holding?
        if( AtomSpaceUtil::getHoldingObjectId(as, petInterface.getPetId()) == candidateId) {
            scoredCandidates[candidate] += 3.5;
        }

        // is near by?
        if ( petInterface.isNear(candidate)) {
            scoredCandidates[candidate] += 3;
        }

        // recently used in my trick?
        bool usedByPet = false;
        foreach(Handle evalLinkOfActionDone, actionsDoneRecentlyInATrick) {
            HandleSeq listLinkHandleOfArgs = as.getOutgoing(evalLinkOfActionDone);
            foreach(Handle ha, listLinkHandleOfArgs) {
                if(StringManipulator::toLower(as.getName(ha)) == StringManipulator::toLower(candidateId)){
                    scoredCandidates[candidate] += 3; 
                    usedByPet = true;
                    break;
                }
            }
            if(usedByPet)
                break;
        }

        // recently used outside trick by speaker?
        // recently used outside trick by me?
        // recently used by someone else?

        bool usedBy[3] = {false, false, false};
        enum userType{pet, speaker, someoneElse};
        userType user;

        foreach(Handle evalLinkOfActionDone, observedActionsDoneRecently) {
            Handle listLinkHandleOfArgs = as.getOutgoing(evalLinkOfActionDone,1);
            HandleSeq ha = as.getOutgoing(listLinkHandleOfArgs); // handle of arguments
            
            // action done by myself?
            if(as.getType(ha[0]) == EXECUTION_LINK) {
                if(as.getName(as.getOutgoing(ha[0],0)) == "say"){
                    if( speakerId == as.getName(as.getOutgoing(as.getOutgoing(ha[0],1), 0)))
                        user = speaker;
                    else user = someoneElse;
                } 
                else  {
                    user = pet;
                }
                ha = as.getOutgoing(as.getOutgoing(ha[0],1));
            }else if(as.getType(ha[0]) == SL_AVATAR_NODE && as.getName(ha[0]) == speakerId ) {
                    user = speaker; 
            }else user = someoneElse;

            foreach(Handle argNode, ha) {
                    if(as.getType(argNode) == LIST_LINK) {
                        foreach(Handle arg, as.getOutgoing(argNode)) {
                            if(StringManipulator::toLower(as.getName(arg)) == StringManipulator::toLower(candidateId)) {
                                usedBy[user] = true;
                            }
                        }
                    }
                    else if (StringManipulator::toLower(as.getName(argNode)) == StringManipulator::toLower(candidateId))
                    {
                            usedBy[user] = true;
                    }
                    if (usedBy[user])
                        break;
            }  
           if (usedBy[pet] && usedBy[speaker] && usedBy[someoneElse])
              break; 
        }

        if(usedBy[pet]){
            scoredCandidates[candidate] += 2;
        }
        if(usedBy[speaker]){
           scoredCandidates[candidate] += 2;
        }
        if(usedBy[someoneElse]){
           scoredCandidates[candidate] += 1;
        }
    }

    typedef pair<Handle, double> scoredCandidate_t;
    stringstream strScored;
    foreach(scoredCandidate_t it, scoredCandidates){
        strScored << TLB::getAtom(it.first)->toString() << " score: " <<  it.second << ", "; 
    }
    logger().log(opencog::Logger::DEBUG,  "%s - Scored Candidates: %s", __FUNCTION__, strScored.str().c_str()); 
} 

Handle NominalReferenceResolver::selectCandidate(const scoreCandidatesMap& scoredCandidates) const {
    logger().log(opencog::Logger::DEBUG,  " %s. %d scored candidates.", __FUNCTION__, scoredCandidates.size()); 

    typedef pair<Handle, double> candidate_t;
    candidate_t selected = *scoredCandidates.begin();
    foreach(candidate_t candidate, scoredCandidates) {
        if (candidate.second > selected.second) 
            selected = candidate;
    }
    logger().log(opencog::Logger::DEBUG,  "%s - Selected \"%s\" with %d score.", __FUNCTION__, petInterface.getAtomSpace().getName(selected.first).c_str(), selected.second); 

    return selected.first;
}


