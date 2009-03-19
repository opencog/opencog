/**
 * LearningTask.h
 *
 * Author: Nil Geisweiller
 * Created : Sun Feb 17 2008
 */
#ifndef MESSAGESYSTEM_LEARNINGTASK_H
#define MESSAGESYSTEM_LEARNINGTASK_H

#include <algorithm>

#include "comboreduct/combo/vertex.h"

#include "IdleTask.h"
#include "EntropyFilter.h"
#include "AtomSpaceWorldProvider.h"
#include "atomspace/AtomSpace.h"
#include "PetaverseImitationLearning.h"
#include "PetaverseVocabularyProvider.h"
#include "petaverse-hillclimber.h"
#include "HCPetaverseVocabularyProvider.h"



namespace MessagingSystem
{

using namespace hillclimbing;
using namespace combo;
using namespace std;
using namespace Filter;

class NetworkElement;

class ImitationLearningTask : public IdleTask
{

    typedef FitnessEstimator::NoSpaceLifeFitnessEstimator FE;


    typedef enum {
        LTS_IDLE,
        LTS_LEARN
    } learningTaskState;

public:
    ImitationLearningTask();
    ~ImitationLearningTask();

    void run(NetworkElement *ne);

    //attempt to initialize the learning algo
    //return true if it succeeds
    //false if it fails
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

    BehaviorCategory _BDCat;
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

    RandGen* _rng;
};

}//~MessageSystem


#endif
