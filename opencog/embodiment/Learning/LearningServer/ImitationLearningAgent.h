/*
 * opencog/embodiment/Learning/LearningServer/ImitationLearningAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef MESSAGESYSTEM_LEARNINGTASK_H
#define MESSAGESYSTEM_LEARNINGTASK_H

#include <algorithm>
#include <opencog/atomspace/AtomSpace.h>

#include <opencog/server/Agent.h>

#include <opencog/embodiment/Learning/Filter/EntropyFilter.h>
#include <opencog/embodiment/Learning/LearningServer/AtomSpaceWorldProvider.h>
#include <opencog/embodiment/Learning/ImitationLearningAPI/PetaverseImitationLearning.h>
#include <opencog/embodiment/Learning/ImitationLearningAPI/PetaverseVocabularyProvider.h>
#include <opencog/embodiment/Learning/PetaverseHC/petaverse-hillclimber.h>
#include <opencog/embodiment/Learning/PetaverseHC/HCPetaverseVocabularyProvider.h>


namespace opencog { namespace messaging {

using namespace hillclimbing;
using namespace std;
using namespace Filter;

// Warning: it is a Mind Agent not a virtual agent in a virtual world
class ImitationLearningAgent : public opencog::Agent
{

    typedef FitnessEstimator::NoSpaceLifeFitnessEstimator FE;


    typedef enum {
        LTS_IDLE,
        LTS_LEARN
    } learningTaskState;

public:
    ImitationLearningAgent(CogServer&);
    virtual ~ImitationLearningAgent();

    virtual void run();

    virtual const ClassInfo& classinfo() const {
        return info();
    }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::ImitationLearningAgent");
        return _ci;
    }

    // attempt to initialize the learning algo
    // return true if it succeeds
    // false if it fails
    bool initLearning(int number_of_estimation_per_cycle,
                      WorldProvider* wp,
                      const argument_list& al,
                      const std::string& pet_id,
                      const std::string& owner_id,
                      const std::string& avatar_id,
                      const std::string& trick_name);

    void addLearningExample(WorldProvider* wp, const argument_list& al);
    void waitForReward();
    void setFitness(fitness_t);
    void stopLearning();
    const combo_tree& getBestSchema();
    const combo_tree& getBestSchemaEstimated();

private:

    learningTaskState _lts;

    BehaviorCategory* _BDCat;
    std::vector<Temporal> _exemplarTemporals;
    arity_t _arity;
    argument_list_list _all;

    EntropyFilter* _entropyFilter;
    combo_tree_ns_set _atomic_perceptions;
    combo_tree_ns_set _atomic_actions;

    FE* _fitnessEstimator;

    PetaverseImitationLearningBase* _PIL;
    PetaverseVocabularyProviderBase* _PVoc;

    definite_object_set _definite_objects;
    std::set<message> _messages;
    agent_to_actions _atas;

    std::string _pet_id;
    std::string _owner_id;
    std::string _avatar_id;
    std::string _trick_name;
};

typedef std::shared_ptr<ImitationLearningAgent> ImitationLearningAgentPtr;

}} // ~namespace opencog::messaging


#endif
