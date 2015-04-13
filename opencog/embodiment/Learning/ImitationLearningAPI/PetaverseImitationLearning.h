/*
 * opencog/embodiment/Learning/ImitationLearningAPI/PetaverseImitationLearning.h
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


/**
 * Abstract class to be inherited to use a learning algo in the context
 * of petaverse imitation learning, the inherited class will be used by
 * ImitationLearningTask
 */

#ifndef _PETAVERSE_IMITATION_LEARNING_BASE_H
#define _PETAVERSE_IMITATION_LEARNING_BASE_H

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/embodiment/Learning/FitnessEstimator/NoSpaceLifeFitnessEstimator.h>

class PetaverseImitationLearningBase
{

public:

    //ns = normal size
    typedef std::set<combo::combo_tree, opencog::size_tree_order<combo::vertex> >
    combo_tree_ns_set;

    typedef double fitness_t;
    typedef std::set<combo::definite_object> definite_object_set;
    typedef std::set<combo::vertex> operator_set;
    typedef FitnessEstimator::NoSpaceLifeFitnessEstimator FE;


    //--------------------------------------------------------------------
    //ctor, dtor
    //--------------------------------------------------------------------

    /**
     * empty construct
     */
    PetaverseImitationLearningBase() {};

    /**
     * Base constructor
     *
     * this will actually not be used but is provided as a typical
     * example of what the learning algorithm's constructor could be
     * the constructor of petaverse-hillclimbing looks like this.
     *
     * @param number_of_estimation_per_cycle  represents the max number of
     *                                        fitness estimations that the
     *                                        learning algorithm is allowed
     *                                        to perform in the course
     *                                        of a learning task cycle
     *                                        This number is determined
     *                                        by the LearningServer
     *
     *                                        Note that if the learning algo
     *                                        is spending an important amount
     *                                        of its time in representation
     *                                        building (like BOA or Pleasure)
     *                                        or other heavy consuming tasks
     *                                        and the number of fitness
     *                                        estimations isn't representative
     *                                        of the amount of time resources
     *                                        allocated to a cycle
     *                                        the semantic of this number
     *                                        should be reconsidered.
     *
     * @param fe  corresponds to the fitness estimator, it is provided by
     *            ImitationLearningTask.
     *            Note that the particular NoSpaceLifeFitnessEstimator
     *            fitness function is imposed in the definition of this API
     *            Since this is the only one in used in petaverse for the moment
     *            that's not a big deal. When other fitness estimator will
     *            defined later, then we'll simply update the API so that it
     *            takes some abstract fitness estimator object instead.
     *
     * @param dos  corresponds to the set of definite objects in the scene
     *             (may or may not be useful depending on the algorithm...)
     *
     * @param os  corresponds to the set of operators used in learning
     *            (excluding actions, perceptions, indefinite objects)
     *            (may or may not be useful depending on the algorithm...)
     *
     * @param perceptions  corresponds to the set of atomic perceptions
     *                     (like near(obj1 obj), is_avatar(random_object), etc)
     *                     possibly involed in the trick to be learned.
     *                     That set is determined by the EntropyFilter
     *
     * @param actions      corresponds to the set of atomic actions
     *                     (like goto_obj(stick), etc)
     *                     possibly involved in the trick to be learned.
     *                     That set is determined by the ActionFilter
     */
    PetaverseImitationLearningBase(int number_of_estimation_per_cycle,
                                   const FE& fe,
                                   const definite_object_set& dos,
                                   const operator_set& os,
                                   const combo_tree_ns_set& perceptions,
                                   const combo_tree_ns_set& actions) {}

    /**
     * Base destructor
     */
    virtual ~PetaverseImitationLearningBase() {}

    //-----------------------------------------------------------------------
    //operator
    //-----------------------------------------------------------------------

    /**
     * At each cycle ImitationLearningTask call this method,
     * it should compute at max a certain number of fitness estimation
     * (see the constructor) or perform some non expensive internal operations
     * and update the various variables possibly accessed
     * (by ImitationLearningTask) in the next cycles (see access methods below)
     */
    virtual void operator()() = 0;

    //-----------------------------------------------------------------------
    //access methods
    //-----------------------------------------------------------------------

    /**
     * That method is called at the end of a learning session
     * is should return the program that has received the maximum reward
     * from the owner.
     */
    virtual const combo::combo_tree& best_program() = 0;

    /**
     * This method is called in case there is no best_program,
     * that is the owner did not send any reward,
     * In such case we will return the best program estimated
     * that is the one that has gotten the highest fitness score
     * of the last fitness estimator (since the last reset_estimator, if so).
     */
    virtual const combo::combo_tree& best_program_estimated() = 0;

    /**
     * This method is called everytime the owner wants the pet to try
     * a new solution. Note that every time that method is called
     * a new solution should be returned. Typically this method should return
     * to the owner the best program never tried so far.
     */
    virtual const combo::combo_tree& current_program() = 0;


    //-----------------------------------------------------------------------
    //acting methods
    //-----------------------------------------------------------------------

    /**
     * this method inform the learning algo that the user has send the reward
     * equal to f. It allows to decide what is the best program to return
     * in the end of the learning session.
     */
    virtual void set_current_fitness(fitness_t f) = 0;

    /**
     * this method is call by ImitationLEarningTask to indicate
     * that the fitness function has changed (the fitness object is the same
     * but it is computing a different fitness function), this typically happens
     * when a new exemplar is sent to LS.
     *
     * That method can be useful or not depending on the algo but if it contains
     * some cache for instance then that method should reset the cache because
     * now the fitness landscape has changed and the cache isn't valid anymore
     */
    virtual void reset_estimator() = 0;

};

#endif

