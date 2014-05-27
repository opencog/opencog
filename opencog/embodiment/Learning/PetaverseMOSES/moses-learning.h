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

#include <opencog/learning/moses/metapopulation/metapopulation.h>
#include <opencog/learning/moses/moses/moses_params.h>
#include <opencog/learning/moses/scoring/scoring.h>
#include <opencog/learning/moses/optimization/optimization.h>

#include <opencog/embodiment/Learning/FitnessEstimator/NoSpaceLifeFitnessEstimator.h>
#include <opencog/embodiment/Learning/FitnessEstimator/DistortedComboSize.h>
#include <opencog/embodiment/Learning/ImitationLearningAPI/PetaverseImitationLearning.h>

#include <boost/unordered_set.hpp>

#define MIN_FITNESS -1.0e10
#define INIT_FITNESS 0.0 //supposed to be the average, neither good nor bad


namespace opencog { namespace moses {

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
                   const operator_set& ignore_ops,
                   const combo_tree_ns_set& perceptions,
                   const combo_tree_ns_set& actions);

    ~moses_learning();

    void operator()();
    const combo_tree& best_program();
    const combo_tree& best_program_estimated();
    const combo_tree& current_program();
    void set_current_fitness(fitness_t f);
    void reset_estimator();

private:
    struct petaverse_cscore *cscore;
    struct petaverse_bscore *bscore;
    struct hill_climbing *climber;
    const FE& _fitness_estimator;
    Comp _comp;
    const definite_object_set& _dos;
    const operator_set& _ignore_ops;
    const combo_tree_ns_set& _perceptions;
    const combo_tree_ns_set& _actions;
    metapop_parameters *_metaparms;

    fitness_t _current_fitness; //real fitness of current program
    fitness_t _current_fitness_estimated;//estimated fitness of current program
    combo_tree _current_program;
    fitness_t _best_fitness;
    combo_tree _best_program;
    fitness_t _best_fitness_estimated;
    combo_tree _best_program_estimated;
    combo_tree _center;

    int _nepc;

    HCState _hcState;

    combo_tree_hash_set _used_for_owner; 
    int _number_of_calls;

    const score_t max_score;

    metapopulation *metapop;
    moses_statistics stats;
};

struct petaverse_cscore : public cscore_base
{
    petaverse_cscore(const FE& fitnessEstimator)
            : _fitnessEstimator(fitnessEstimator) {}

    petaverse_cscore(const petaverse_cscore& ps)
            : _fitnessEstimator(ps._fitnessEstimator) {}

    composite_score operator()(const combo_tree& tr) const
    {
        score_t score = _fitnessEstimator(tr);
        std::cout << "scoring " << tr << " score: " << score << std::endl;

// This complexity ratio matches the original code, but it is not
// obviously correct to me.
#define CPXY_RATIO 1.0

        complexity_t cpxy = tr.size();
        composite_score cs(score, cpxy, cpxy / CPXY_RATIO);
        return cs;
    }
    ~petaverse_cscore() {}

private:
    const FE& _fitnessEstimator;
};

// @todo: this is not a good behavioral score, it should rather be
// such that each feature correspond to the dissimilarity of an action
// in the action sequence to imitate
struct petaverse_bscore : public bscore_base
{
    petaverse_bscore(const FE& fitnessEstimator)
            : _fitnessEstimator(fitnessEstimator) {}

    petaverse_bscore(const petaverse_bscore& ps)
            : _fitnessEstimator(ps._fitnessEstimator) {}

    behavioral_score operator()(const combo_tree& tr) const
    {
        behavioral_score bs;
        bs.push_back(_fitnessEstimator(tr));
        return bs;
    }

    behavioral_score best_possible_bscore() const
    {
        return {0.0};           // @todo that might be wrong
    }

    ~petaverse_bscore() {}
private:
    const FE& _fitnessEstimator;
};

} // ~namespace moses
} // ~namespace opencog

#endif
