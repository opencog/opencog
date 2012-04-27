/*
 * opencog/embodiment/Learning/FitnessEstimator/NoSpaceLifeFitnessEstimator.h
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

#ifndef _NOSPACELIFEFITNESSESTIMATOR_H
#define _NOSPACELIFEFITNESSESTIMATOR_H

#include <opencog/util/lru_cache.h>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/embodiment/Learning/behavior/BehaviorCategory.h>
#include <opencog/embodiment/Learning/behavior/BehaviorDescriptionMatcher.h>
#include <opencog/embodiment/Learning/behavior/WorldProvider.h>
#include "SizePenalty.h"

//that paramter is here to determine to the maximum number of cycles the
//the process is going to run after the total number of cycles
//has reached the size of the exemplar (its number of actions)
//in case it has not finished before
//so total number of cycles is calculated
//(1+MAX_ADDITIONAL_CYCLE_COEF)*bd.size()
//where bd.size() is the sie of the current composite behavior description
#define MAX_ADDITIONAL_CYCLE_COEF 1.0

#define IS_FE_LRU_CACHE
#define FE_LRU_CACHE_SIZE 1000000

namespace FitnessEstimator
{

typedef double fitness_t;

typedef combo::combo_tree::iterator pre_it;
typedef combo::combo_tree::sibling_iterator sib_it;

/**
 *fitness estimation using NoSpaceLife and BehaviorDescriptionMatcher
 */
struct NoSpaceLifeFitnessEstimator: std::unary_function<combo::combo_tree, fitness_t> {

public:

    /**
     * constructor
     * @param wp          The worldProvider, contain the action space and
     *                    timestamp of the world (the timestamp is usefull here
     *                    to retrieve the examplars)
     * @param petName     The name of the pet
     * @param ownerName   The name of the owner
     * @param avatarName  The name of the avatar to imitate
     * @param dos         The definite object set, used to compute sizePenalty
     * @param opCount     The number of operators
     * @param predCount   The number of predicates that have passed the entropy
     *                    filter, it is used to compute sizePenalty function
     * @param actionCount The number of actions that have passed the similarity
     *                    filter, it is used to compute sizePenalty function
     */
    NoSpaceLifeFitnessEstimator(WorldProvider* wp,
                                const std::string& petName,
                                const std::string& ownerName,
                                const std::string& avatarName,
                                const std::string& trickName,
                                const combo::definite_object_set& dos,
                                behavior::BehaviorCategory& BDCat,
                                const std::vector<Temporal>& exemplarTemporals,
                                const combo::argument_list_list& all,
                                int indefinite_object_count,
                                int operator_count,
                                int predicate_Count,
                                int action_count);

    ~NoSpaceLifeFitnessEstimator();

    /**
     * operator
     */
    result_type operator()(const argument_type& tr) const;

    /**
     * public methods
     */

    /**
     * update the fitness estimator by retreiving the last examplar
     * and update predCount and actionCount
     */
    void update(int indefinite_object_count, int operator_count,
                int predicate_count, int action_count);

private:

    WorldProvider* _wp; //not const because Combo2BD may add atoms
    //into wp's atomspace

    const combo::definite_object_set& _dos;

    const std::string& _petName;
    const std::string& _ownerName;
    const std::string& _avatarName;
    const std::string& _trickName;

    const behavior::BehaviorCategory& _BDCat; //list of the examplars retreived
    const std::vector<Temporal>& _exemplarTemporals;
    const combo::argument_list_list& _all; //argument lists, one for each BD
    behavior::BehaviorDescriptionMatcher _BDMatcher;
    SizePenalty _sizePenalty;

#ifdef IS_FE_LRU_CACHE
    typedef std::vector<fitness_t> fitness_vec;
    typedef fitness_vec::iterator fitness_vec_it;
    typedef fitness_vec::const_iterator fitness_vec_const_it;
    typedef opencog::lru_cache_arg_result<combo::combo_tree, fitness_vec> BDCache;
    mutable BDCache _bd_cache;
    mutable unsigned int _cache_success;
    mutable unsigned int _total_fitness_call;
#endif

    //true to be activated
    bool _randomOperatorOptimization;

    //this value is at false if we want to sent indefinite plan
    //that would be for random operator optimization
    bool _sendDefinitePlan;

    /**
     * private methods
     */
    int getTrialCount(const combo::combo_tree& tr) const;

};
}

#endif
