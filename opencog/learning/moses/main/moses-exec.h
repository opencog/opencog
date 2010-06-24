/** moses-exec.h --- 
 *
 * Copyright (C) 2010 Nil Geisweiller
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
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

#ifndef _OPENCOG_MOSES_EXEC_H
#define _OPENCOG_MOSES_EXEC_H

#include <iostream>
#include <fstream>
#include <stdio.h>

#include <boost/program_options.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>

#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/combo/table.h>

// for operator>> to combo 
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h> 

#include "../moses/moses.h"
#include "../moses/optimization.h"
#include "../moses/scoring_functions.h"
#include "../moses/scoring.h"

using namespace boost::program_options;
using namespace std;
using namespace moses;
using namespace reduct;
using opencog::logger;
using namespace ant_combo;


static const string un="un"; // univariate
static const string sa="sa"; // simulation annealing
static const string hc="hc"; // hillclimbing

/**
 * 1) create a metapopulation
 * 2) run moses
 * 3) print the results
 */
template<typename Scoring, typename BScoring, typename Optimization>
void metapop_moses_results(opencog::RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const combo::type_tree& tt,
                           const reduct::rule& si,
                           const Scoring& sc,
                           const BScoring& bsc,
                           const Optimization& opt,
                           int max_evals,
                           int max_gens,
                           const vertex_set& ignore_ops,
                           long result_count) {
    metapopulation<Scoring, BScoring, Optimization> 
        metapop(rng, bases, tt, si, sc, bsc, opt);
    moses::moses(metapop, max_evals, max_gens, 0, ignore_ops);
    metapop.print_best(result_count);
}

/**
 * like above but takes the algo type instead of the algo template
 */
template<typename Scoring, typename BScoring>
void metapop_moses_results(opencog::RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const combo::type_tree& tt,
                           const reduct::rule& si,
                           const Scoring& sc,
                           const BScoring& bsc,
                           const string& opt_algo,
                           int max_evals,
                           int max_gens,
                           const vertex_set& ignore_ops,
                           long result_count) {
    if(opt_algo == un) { // univariate
        metapop_moses_results(rng, bases, tt, si, sc, bsc,
                              univariate_optimization(rng),
                              max_evals, max_gens, ignore_ops, result_count);
    } else if(opt_algo == sa) { // simulation annealing
        metapop_moses_results(rng, bases, tt, si, sc, bsc,
                              simulated_annealing(rng),
                              max_evals, max_gens, ignore_ops, result_count);
    } else if(opt_algo == hc) { // hillclimbing
        metapop_moses_results(rng, bases, tt, si, sc, bsc,
                              iterative_hillclimbing(rng),
                              max_evals, max_gens, ignore_ops, result_count);
    } else {
        std::cerr << "Unknown optimization algo " << opt_algo << ". Supported algorithms are un (for univariate), sa (for simulation annealing) and hc (for hillclimbing)" << std::endl;
        exit(1);
    }
}

#endif // _OPENCOG_MOSES_EXEC_H
