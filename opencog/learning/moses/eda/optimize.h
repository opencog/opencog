/*
 * opencog/learning/moses/eda/optimize.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _EDA_OPTIMIZE_H
#define _EDA_OPTIMIZE_H

#include <opencog/util/Logger.h>
#include <opencog/util/oc_omp.h>

#include "../representation/instance_set.h"

namespace opencog { 
namespace moses {
    
    // Generic evolutionary competition algorithm, intended for use
    // with algorithms that can discern dependencies between the effects
    // of "genes" within the population (i.e. intended for use with 
    // estimation-of-distribution algorithms, such as the Bayesian 
    // optimization algorithm, but can also be used with "dumb"
    // algorithms).
    //
    // Rough outline: a population of individuals is created and
    // scored.  Then iteration on a population is performed until
    // the TerminationPolicy is met. Uses ScoringPolicy to rank
    // individuals in the population. Uses SelectionPolicy to select
    // individuals for tournament rounds.  Uses ReplacementPolicy to
    // replace loosing individuals in the population.  The dependencies
    // between genes in individuals are modelled using the
    // StructureLearningPolicy, while the ProbsLearningPolicy is used to
    // generate specific genetic variations (based on the learned
    // dependencies.)
    //
    // Other arguments:
    // n_select: number of individuals to select out of the population,
    //     and enroll into the tournament. The SelectionPolicy will be
    //     used to select this many, each generation.
    //
    // Returns the number of scoring evaluations actually performed.
    template <typename ScoreT,
              typename ScoringPolicy,
              typename TerminationPolicy,
              typename SelectionPolicy,
              typename StructureLearningPolicy,
              typename ProbsLearningPolicy,
              typename ReplacementPolicy,
              typename LoggingPolicy>
    int optimize(instance_set<ScoreT>& current, // population
                 int n_select,  // number of individuals to enroll.
                 int n_generate,
                 int max_gens,  // maximum number of generations
                 const ScoringPolicy& score,
                 const TerminationPolicy& termination_criterion,
                 const SelectionPolicy& select,
                 const StructureLearningPolicy& learn_structure,
                 const ProbsLearningPolicy& learn_probs,
                 const ReplacementPolicy& replace,
                 LoggingPolicy& write_log, 
                 RandGen& rng) 
    {


        // Logger
        logger().debug("Probabilistic Learning Optimization");
        // ~Logger

        typedef typename StructureLearningPolicy::model_type model_type;

        // Compute scores of the initial instance set.
        logger().debug("Evaluate the initial population (%u individuals)",
                       current.size());

        // OMP_ALGO forces use of parallel algorithms on what would
        // otherwise be ordinary std:: namespace classes. In this
        // case, std::transform.
        //
        // Note use of bind+cref below. This is because score() will
        // typically be implemented not as an actual function, but
        // as a unary_function<instance,ScoreT>.  Now std::transform
        // passes by value, which would cause the score() object to
        // be copied. We really don't want that, so the bind+cref step
        // is a trick done to pass score() by reference. This way, the
        // score object can remember outcomes of previous scorings.
        // Note score.operator() needs to be thread-safe!
        OMP_ALGO::transform(current.begin(), current.end(),
                            current.begin_scores(),
                            bind(boost::cref(score), _1));

        // Main loop.
        int generation = 0;
        for (; generation < max_gens 
                 && !termination_criterion(current.begin(), current.end());
             ++generation)
        {
            // Log the population (instance_set) and scores.
            write_log(current.begin(), current.end(),
                      current.fields(), generation);
            
            // Select n_select promising instances to enroll in tournament.
            logger().debug("Select %d promising instances for model building",
                           n_select);
            std::vector<scored_instance<ScoreT> > promising(n_select);
            select(current.begin(), current.end(), promising.begin(), n_select);
            
            // Initialize the model.
            logger().debug("Build probabilistic model");
            model_type model(current.fields(), promising.begin(),
                             promising.end(), rng);
            
            // Update the model.
            learn_structure(current.fields(), promising.begin(),
                            promising.end(), model);
            learn_probs(current.fields(), promising.begin(),
                        promising.end(), model);
            
            //create new instances and integrate them into the current
            //instance set, replacing existing instances
            logger().debug("Sample and evaluate %d new candidates"
                           " according to the model", n_generate);
            instance_set<ScoreT> new_instances(n_generate, current.fields());
            foreach(auto& inst, new_instances)
                inst = model();
            OMP_ALGO::transform(new_instances.begin(), new_instances.end(),
                                new_instances.begin_scores(),
                                bind(boost::cref(score), _1));

            logger().debug("Replace the new candidates");
            replace(new_instances.begin(), new_instances.end(),
                    current.begin(), current.end());
        }

        //log the final result
        write_log(current.begin(), current.end(),
                  current.fields(), generation);
        
        // Return # of evaluations actually performed
        return current.size() + generation * n_generate;
    }
    
} // ~namespace moses
} // ~namespace opencog

#endif
