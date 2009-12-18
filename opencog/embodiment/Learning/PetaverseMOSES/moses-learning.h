/*
 * opencog/embodiment/Learning/PetaverseMOSES/moses-learning.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Predrag Janicic
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

#ifndef _MOSES_LEARNING_H
#define _MOSES_LEARNING_H

#include <opencog/util/tree.h>
#include <opencog/util/lru_cache.h>

#include <opencog/learning/moses/moses/optimization.h>

#include <opencog/embodiment/Learning/FitnessEstimator/DistortedComboSize.h>
#include <opencog/embodiment/Learning/ImitationLearningAPI/PetaverseImitationLearning.h>

#include <boost/unordered_set.hpp>

#define ESTIMATOR_CACHE_SIZE 500000

#define MIN_FITNESS -1.0e10
#define INIT_FITNESS 0.0 //supposed to be the average, neither good nor bad


namespace moses
{

typedef FitnessEstimator::NoSpaceLifeFitnessEstimator FE;
typedef FitnessEstimator::DistortedComboSizeOrder Comp;

typedef boost::unordered_set<combo_tree, boost::hash<combo_tree> > combo_tree_hash_set;
typedef combo_tree_hash_set::iterator combo_tree_hash_set_it;

typedef enum {
    HC_INIT,
    HC_REWARD,
    HC_START_ITERATION,
    HC_FINISH_ITERATION,
    HC_BUILD_CANDIDATES,
    HC_ESTIMATE_CANDIDATES,
    HC_FINISH_CANDIDATES,
    HC_IDLE
} HCState;


class moses_learning : public PetaverseImitationLearningBase
{

public:

    //constructor, generate/filter conditions and actions
    moses_learning(int nepc,
                   const FE& fitness_estimator,
                   const definite_object_set& dos,
                   const operator_set& eo,
                   const combo_tree_ns_set& perceptions,
                   const combo_tree_ns_set& actions,
                   opencog::RandGen& rng);

    ~moses_learning();

    void operator()();
    const combo_tree& best_program();
    const combo_tree& best_program_estimated();
    const combo_tree& current_program();
    void set_current_fitness(fitness_t f);
    void reset_estimator();

private:
    struct petaverse_score  *score;
    struct petaverse_bscore *bscore;
    const FE& _fitness_estimator;
    Comp _comp;
    const definite_object_set& _dos;
    const std::set<vertex>& _elementary_operators;
    const combo_tree_ns_set& _perceptions;
    const combo_tree_ns_set& _actions;

    fitness_t _current_fitness; //real fitness of current program
    fitness_t _current_fitness_estimated;//estimated fitness of current program
    combo_tree _current_program;
    fitness_t _best_fitness;
    combo_tree _best_program;
    fitness_t _best_fitness_estimated;
    combo_tree _best_program_estimated;
    combo_tree _center;

    int _nepc;

    opencog::RandGen& _rng;

    HCState _hcState;

    combo_tree_hash_set _used_as_center;
    combo_tree_hash_set _used_for_owner;
    ordered_programs _ordered_best_estimates; //_ordered_neighborhood;
    int _number_of_calls;

    const combo_tree_score max_score;

    metapopulation<petaverse_score, petaverse_bscore, sliced_iterative_hillclimbing> *metapop;

};




struct petaverse_score {
    petaverse_score(const FE& fitnessEstimator)
            : _fitnessEstimator(fitnessEstimator) {
        _estimator_cache = new opencog::lru_cache<FE>(ESTIMATOR_CACHE_SIZE,
                                                      _fitnessEstimator);
    }

    petaverse_score(const petaverse_score& ps)
            : _fitnessEstimator(ps._fitnessEstimator) {
        _estimator_cache = new opencog::lru_cache<FE>(ESTIMATOR_CACHE_SIZE,
                                                      ps._fitnessEstimator);
    }

    int operator()(const combo_tree& tr) const {
        int score = -1000 + (int)(1000 * (*_estimator_cache)(tr));
        std::cout << "scoring " << tr << " score: " << score << endl;
        return score;
    }

    ~petaverse_score() {
        delete _estimator_cache;
    }

private:
    opencog::lru_cache<FE>* _estimator_cache;
    const FE& _fitnessEstimator;
};



struct petaverse_bscore {
    petaverse_bscore(const FE& fitnessEstimator)
            : _fitnessEstimator(fitnessEstimator) {
        _estimator_cache = new opencog::lru_cache<FE>(ESTIMATOR_CACHE_SIZE,
                                                      _fitnessEstimator);
    }

    petaverse_bscore(const petaverse_bscore& ps)
            : _fitnessEstimator(ps._fitnessEstimator) {
        _estimator_cache = new opencog::lru_cache<FE>(ESTIMATOR_CACHE_SIZE,
                                                      ps._fitnessEstimator);
    }

    behavioral_score operator()(const combo_tree& tr) const {
        behavioral_score bs(2);
        bs[0] = 1000 - (int)(1000 * (*_estimator_cache)(tr));
        bs[1] = tr.size();
        return bs;
    }

    ~petaverse_bscore() {
        delete _estimator_cache;
    }

private:
    opencog::lru_cache<FE>* _estimator_cache;
    const FE& _fitnessEstimator;
};



}//~namespace moses


#endif
