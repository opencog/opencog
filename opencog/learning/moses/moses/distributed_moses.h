/** distributed_moses.h --- 
 *
 * Copyright (C) 2010 Novamente LLC
 *
 * Author: Nil Geisweiller
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


#ifndef _OPENCOG_DISTRIBUTED_MOSES_H
#define _OPENCOG_DISTRIBUTED_MOSES_H

#include <stdio.h>

#include <boost/program_options.hpp>

#include "metapopulation.h"
#include "moses.h"

namespace moses
{

using namespace boost::program_options;

string build_command_line(const variables_map& vm, const combo_tree& tr, unsigned int eval)
{
    return "./moses-exec -H pa -m 100000 -k 3 -E 3";
}

/**
 * the main function of Distributed MOSES
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
void distributed_moses(metapopulation<Scoring, BScoring, Optimization>& mp,
                       const variables_map& vm, unsigned int jobs,
                       const moses_parameters& pa = moses_parameters())
{
    // Logger
    logger().info("Distributed MOSES starts");
    // ~Logger

    int gen_idx = 0;

    //    while ((mp.n_evals() < max_evals) && (max_gens != gen_idx++)) {
    const combo_tree exemplar(get_tree(*mp.select_exemplar()));

    string command_line = build_command_line(vm, exemplar,
                                             pa.max_evals - mp.n_evals());

    FILE* res_fp = popen(command_line.c_str(), "r");
    
    char line[4096];

    while ( fgets( line, sizeof line, res_fp)) {
        printf("%s", line);
    }
    pclose(res_fp);
        
        //run a generation
        // if (mp.expand(max_evals - mp.n_evals(), max_score, ignore_ops,
        //               perceptions, actions)) {
        // } else // In iterative hillclimbing it is possible (but not
        //        // likely) that the metapop gets empty and expand
        //        // return false
        //     break;
        // if (mp.best_score() >= max_score || mp.empty())
        //     break;
    //    }    
    // Logger
    logger().info("Distributed MOSES ends");
    // ~Logger
}

} // ~namespace moses

#endif // _OPENCOG_DISTRIBUTED_MOSES_H
