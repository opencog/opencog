/**
 * NoSpaceLife.h
 *
 * Author(s):
 *   Nil Geisweiller
 * Creation: Wed Dec 5 2007
 */
#ifndef _NOSPACELIFE_H
#define _NOSPACELIFE_H

#include "comboreduct/combo/vertex.h"

#include <opencog/atomspace/AtomSpace.h>
#include "SpaceServer.h"
#include "CompositeBehaviorDescription.h"
#include "PetComboVocabulary.h"

#include "util/RandGen.h"

#define ACTION_DONE_TIME 200 //each action takes 2 seconds
#define PAUSE_TIME 200
#define BEHAVED_STR "behaved"

namespace ImaginaryLife
{

using namespace behavior;
using namespace combo;

class NoSpaceLife
{
public:
    typedef combo_tree::iterator pre_it;
    typedef combo_tree::sibling_iterator sib_it;

    /**
     * cbd is the CompositeBehaviorDescription to imitate
     * the reason it is provided is because the generated BD will uses
     * the timestamps of cbd to calculate the action durations
     * and pause durations
     */
    NoSpaceLife(SpaceServer& spaceServer, const std::string& pet_id,
                const std::string& owner_id, const std::string& avatar_id,
                const CompositeBehaviorDescription& cbd,
                const Temporal& exemplarTemporal,
                opencog::RandGen& rng);

    ~NoSpaceLife();

    /**
     * process a list of actions [from, to),
     * is it assumed that all objects are definite
     * returns true iff the action sequence has been successful
     */
    bool processSequential_and(sib_it from, sib_it to);

    /**
     * get the generated BD
     */
    CompositeBehaviorDescription& getGeneratedBD();

    /**
     * get the current SpaceMap
     */
    Handle getCurrentMapHandle();

    unsigned long getCurrentTime();

    AtomSpace& getAtomSpace() const;

private:
    SpaceServer& _spaceServer; //a reference to the SpaceServer and AtomSpace that lives within LS
    //not a const because it can be changed by the algo

    const std::string& _pet_id;
    const std::string& _owner_id;
    const std::string& _avatar_id; //the avatar to imitate
    //const definite_object_set& _definite_objects; //reference to
    //definite_objects
    //that leaves within HCTask

    unsigned long _currentTime;

    unsigned _currentIndex;

    Handle _currentMapHandle;

    bool _hasTimeChanged;

    const CompositeBehaviorDescription& _imitatedBD;

    const Temporal& _exemplarTemporal;

    CompositeBehaviorDescription _generatedBD;

    opencog::RandGen& _rng;

    void generateElementaryBD(ElementaryBehaviorDescription& ebd,
                              pre_it it,
                              unsigned long start_time,
                              unsigned long end_time);

    builtin_action choose_random_step() const;

    definite_object choose_definite_object_that_fits(indefinite_object obj,
                                                     int arg_index);

};

}//~namespace ImaginaryLife

#endif
