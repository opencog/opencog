/*
 * opencog/learning/moses/moses/moses.h
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
#ifndef _MOSES_MOSES_H
#define _MOSES_MOSES_H

#include "metapopulation.h"

typedef std::set<combo::vertex> operator_set;
typedef std::set<combo::combo_tree, opencog::size_tree_order<combo::vertex> >
combo_tree_ns_set;

/**
 * the main function of MOSES
 *
 * @param mp          the metapopulation 
 * @param max_evals   the max evaluations
 * @param max_gens    the max number of demes to create and optimize, if
 *                    negative, then no limit
 * @param max_score   the max score tree
 * @param ignore_ops  the set of operators to ignore
 * @param perceptions the set of perceptions of an optional interactive agent
 * @param actions     the set of actions of an optional interactive agent
 */
template<typename Scoring, typename BScoring, typename Optimization>
void moses(metapopulation<Scoring, BScoring, Optimization>& mp,
           int max_evals, int max_gens, const composite_score& max_score,
           const operator_set& ignore_ops = operator_set(),
           const combo_tree_ns_set* perceptions = NULL,
           const combo_tree_ns_set* actions = NULL)
{
    // Logger
    logger().info("MOSES starts");
    // ~Logger
    int gen_idx = 0;

    while ((mp.n_evals() < max_evals) && (max_gens != gen_idx++)) {
        // Logger
        logger().info("Deme expansion: %i", gen_idx);
        // ~Logger

        //run a generation
        if (mp.expand(max_evals - mp.n_evals(), max_score, ignore_ops,
                      perceptions, actions)) {
        } else // In iterative hillclimbing it is possible (but not
               // likely) that the metapop gets empty and expand
               // return false
            break;
        if (mp.best_score() >= max_score || mp.empty())
            break;
    }    
    // Logger
    logger().info("MOSES ends");
    // ~Logger
}


template<typename Scoring, typename Domination, typename Optimization>
void moses(metapopulation<Scoring, Domination, Optimization>& mp,
           int max_evals, int max_gens, score_t max_score, 
           const operator_set& ignore_ops = operator_set(),
           const combo_tree_ns_set* perceptions = NULL,
           const combo_tree_ns_set* actions = NULL)
{
    moses(mp, max_evals, max_gens, 
          composite_score(max_score, worst_possible_score.second),
          ignore_ops, perceptions, actions);
}

// ignore the max_gens, for backward compatibility
template<typename Scoring, typename Domination, typename Optimization>
void moses(metapopulation<Scoring, Domination, Optimization>& mp,
           int max_evals, score_t max_score, 
           const operator_set& ignore_ops = operator_set(),
           const combo_tree_ns_set* perceptions = NULL,
           const combo_tree_ns_set* actions = NULL)
{
    moses(mp, max_evals, -1, 
          composite_score(max_score, worst_possible_score.second),
          ignore_ops, perceptions, actions);
}

/**
 * @brief The sliced version of moses
 *
 * It is only used for testing
 * 
 * @todo should be removed once slice and non-slice MOSES are totally
 * factorized
 *
 * Lists of relevant operators, perceptions, and actions may or may not be
 * provided. The initial design assumed fixed lists, this version
 * has a constructor including these parameters, specific for actions
 * 
 * @param mp the metapopulation
 * @param max_evals the max evlautions
 * @parma max_score the max score, the type is score tree
 * @param ignore_ops the operator set to ignore
 * @param perceptions the set of perceptions of the interactive agent
 * @param actions the set of actions of the interactive agent
 */
template<typename Scoring, typename Domination, typename Optimization>
void moses_sliced(metapopulation<Scoring, Domination, Optimization>& mp,
                  int max_evals,
                  const composite_score& max_score,
                  const operator_set& ignore_ops,
                  const combo_tree_ns_set* perceptions,
                  const combo_tree_ns_set* actions)
{
    int o;
    int max_for_slice = 20;

    while (mp.n_evals() < max_evals) {
        o = 0;
        if (mp.create_deme(ignore_ops, perceptions, actions)) {
            while (o >= 0)
                o = mp.optimize_deme(max_evals, max_for_slice, max_score);

            mp.close_deme();

        } else
            break;
    }

    // print the best solution
    std::cout << "sampled " << mp.n_evals()
              << " best " << mp.best_score().first << endl
              << mp.best_tree() << std::endl;
}


/**
 * @brief The sliced version of moses
 *
 * Lists of relevant operators, perceptions, and actions may or may not be
 * provided. The initial design assumed fixed lists, this version
 * has a constructor including these parameters, specific for actions
 * 
 * @param mp the metapopulation
 * @param max_evals the max evlautions
 * @parma max_score the max score, the type is score_t
 * @param ignore_ops the operator set to ignore
 * @param perceptions the set of perceptions of the interactive agent
 * @param actions the set of actions of the interactive agent
 */
template<typename Scoring, typename Domination, typename Optimization>
void moses_sliced(metapopulation<Scoring, Domination, Optimization>& mp,
                  int max_evals, score_t max_score,
                  const operator_set& ignore_ops,
                  const combo_tree_ns_set* perceptions,
                  const combo_tree_ns_set* actions)
{
    moses_sliced(mp, max_evals,
                 composite_score(max_score, worst_possible_score.second),
                 ignore_ops, perceptions, actions);
}

} //~namespace moses

#endif
