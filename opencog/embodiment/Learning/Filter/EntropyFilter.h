/*
 * opencog/embodiment/Learning/Filter/EntropyFilter.h
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

#ifndef ENTROPYFILTER_H_
#define ENTROPYFILTER_H_

#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <opencog/comboreduct/combo/vertex.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/Temporal.h>

#include <opencog/embodiment/Learning/behavior/WorldProvider.h>
#include <opencog/embodiment/Learning/RewritingRules/RewritingRules.h>
#include <opencog/embodiment/Learning/behavior/BehaviorCategory.h>
#include "EntityRelevanceFilter.h"

// Enable optimization of the algo based on _isMoving. It computes
// _isMoving (via setIsMoving) for all entities over the given time
// interval and at each considered instant do not calculate the
// predicates that are invariant over fixed entites (via checking the
// predicate with the method doesInvolveMoving) and use their previous
// value instead (via _perceptToBoolTime)
#define ISMOVING_OPTIMIZE

//enable optimize using lru_cache
//note : it's not very efficient, probably because
//there is too few redundancies,
//so I disable it anyway to make the code looks clearer
//#define LRU_CACHE_OPTIMIZE

//use a set of definite_object rather than a map to bool, can be faster
//if there are only a few moving object
#define ISMOVING_SET

namespace Filter
{

using namespace combo;
using namespace behavior;

typedef combo_tree::iterator pre_it;
typedef combo_tree::sibling_iterator sib_it;

class EntropyFilter
{
    typedef opencog::size_tree_order<vertex> combo_tree_order;

    typedef std::set<combo_tree, combo_tree_order> combo_tree_ns_set;
    typedef combo_tree_ns_set::iterator combo_tree_ns_set_it;
    typedef combo_tree_ns_set::const_iterator combo_tree_ns_set_const_it;

    typedef std::map<const combo_tree, unsigned long, combo_tree_order> combo_tree_time_map;
    typedef combo_tree_time_map::iterator combo_tree_time_map_it;
    typedef combo_tree_time_map::const_iterator combo_tree_time_map_const_it;

    typedef std::pair<bool, unsigned long> bool_time_pair;
    typedef std::map<const combo_tree, bool_time_pair, combo_tree_order>
    combo_tree_bool_time_map;
    typedef combo_tree_bool_time_map::iterator combo_tree_bool_time_map_it;
    typedef combo_tree_bool_time_map::const_iterator combo_tree_bool_time_map_const_it;

    typedef std::set<vertex> vertex_set;
    typedef vertex_set::iterator vertex_set_it;
    typedef vertex_set::const_iterator vertex_set_const_it;

    //typedef std::map<std::string, bool> definite_object_bool_map;
    typedef boost::unordered_map<std::string, bool, boost::hash<std::string> >
    definite_object_bool_map;
    typedef definite_object_bool_map::iterator definite_object_bool_map_it;
    typedef definite_object_bool_map::const_iterator
    definite_object_bool_map_const_it;

    typedef boost::unordered_set<std::string, boost::hash<std::string> >
    definite_object_hash_set;
    typedef definite_object_hash_set::iterator definite_object_hash_set_it;
    typedef definite_object_hash_set::const_iterator
    definite_object_hash_set_const_it;

    typedef boost::function < bool (const combo_tree) > EvalFunct;

public:
    //constructor, destructor
    //input_arg_types corresponds the list of types of each input arguments
    //of the combo program to learn
    //self_id and owner_id correspond to the atom_name when id::self
    //and id::owner respectively are evaluated
    //in case of imitation learning self_id will be the id of the avatar
    //to imitate
    //dos contains the set of definite_objects
    //while ados contains the set of agents' actions definite_objects
    EntropyFilter(const std::string& self_id,
                  const std::string& owner_id,
                  AtomSpace& atomSpace,
                  const perception_set& elementary_perceptions,
                  const indefinite_object_set& idos,
                  const definite_object_set& dos,
                  const message_set& ms,
                  const agent_to_actions& atas,
                  const type_tree_seq& input_arg_types);
    ~EntropyFilter();

    //update the map _perceptToTim and _total_time by evaluating all
    //perceptions in the interval temp
    void updatePerceptToTime(const Temporal& temp,
                             const argument_list& al);

    //like above but using lowerBound and upperBound, provided for convenience
    void updatePerceptToTime(unsigned long lb, unsigned long up,
                             const argument_list& al);

    //when the set of definite object changes the set of possible
    //perception must be rebuild accordingly
    //that method add the new perceptions in _perceptToTime
    void rebuildPerceptToTime();

    //fill pred_set with all predicates with entropy above threshold
    void generateFilteredPerceptions(combo_tree_ns_set& pred_set,
                                     double threshold);

    //update _perceptToTime and _total_time
    //according to the intervals of the BehaviorCategory
    //then fill pred_set with perceptions with entropy > threshold
    void generateFilteredPerceptions(combo_tree_ns_set& pred_set,
                                     double threshold,
                                     const BehaviorCategory& BDCat,
                                     const std::vector<Temporal>& est,
                                     const argument_list_list& all);

    //rebuild the object set and add new perceptions
    //then update _perceptToTime and _total_time
    //according to the interval of cbd
    //then fill pred_set with perception with entropy > threshold
    void generateFilteredPerceptions(combo_tree_ns_set& pred_set,
                                     double threshold,
                                     const CompositeBehaviorDescription& cbd,
                                     const Temporal& et,
                                     const argument_list& al);
private:
    //attributes
    const std::string& _self_id;
    const std::string& _owner_id;
    AtomSpace& _atomSpace;
    const perception_set& _elementary_perceptions;
    const indefinite_object_set& _idos;
    const definite_object_set& _dos;
    const message_set& _ms;
    const agent_to_actions& _atas;
    const type_tree_seq& _input_arg_types; //input arguments of the
    //combo program to learn
    arity_t _arity;//size of _input_arg_types

    unsigned int _hasSaidDelay;

    vertex_set _operands; //set of all objects definite and indefinite
    //messages and arguments, in children of
    //perception

#ifdef ISMOVING_OPTIMIZE
    combo_tree_bool_time_map _perceptToBoolTime; //associate a perception to its
    //last truth value
    //expectation time
#else
    combo_tree_time_map _perceptToTime; //associate a perception to its
    //expectation time (of being true)
#endif

    unsigned int _percept_count; //number of perceptions used by the lru_cache

    unsigned long _total_time; //total time of a perception (equal for all)

    Handle _spaceMapNode;

#ifdef ISMOVING_SET
    definite_object_hash_set _isMoving;
#else
    definite_object_bool_map _isMoving;
#endif

    std::vector<definite_object> _indefToDef; //map an indefinite object
    //to a definite object
    //this in order to avoid
    //reevaluating nearest_X
    //several time during the same
    //spaceMap.

    /**
     * Return true is the vertex corresponds to a predicate that is
     * invariant if the entities involved are immobile.
     */
    inline bool doesInvolveMoving(vertex v);

    /**
     * Set the value of the cache _isMoving with val for entity obj
     */
    inline void setIsMoving(const definite_object& obj,
                            bool val);

    /**
     * Set the value of the cache _isMoving for entity obj with true
     * if the object has moved between SpaceMap pre_sm and sm.
     */
    inline void setIsMoving(const definite_object& obj,
                            const SpaceServer::SpaceMap* pre_sm,
                            const SpaceServer::SpaceMap& sm);

    //look up the cache isMoving if obj is moving
    inline bool getIsMoving(const definite_object& obj);

    //eval perception
    bool evalPerception(Handle smh, const combo_tree tr);

    //build (or rebuild) the set of operands
    //by inserting all new definite_objects, indefinite_objects
    //messages, input arguments ($1, $2, ...)
    void build_operand_set();

    //build and insert all atomic perceptions of p
    //that is all possible arguments, if there is, are inserted as children
    //and taken from the set _operands
    void build_and_insert_atomic_perceptions(perception p);

    //like above but take in input a combo_tree and the number of arguments
    //that remains to insert on the combo_tree
    void build_and_insert_atomic_perceptions(const combo_tree& tr, arity_t arity_rest);

};

}//~namespace Filter


#endif
