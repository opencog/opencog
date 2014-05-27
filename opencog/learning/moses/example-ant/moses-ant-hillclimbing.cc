/*
 * opencog/learning/moses/main/moses-ant-hillclimbing.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Predrag Janicic
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
#include <algorithm>
#include <iostream>
#include <set>
#include <string>
#include <boost/lexical_cast.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/numeric.h>
#include <opencog/util/oc_omp.h>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>
#include <opencog/comboreduct/reduct/reduct.h>

#include "../moses/moses_main.h"
#include "../optimization/optimization.h"
#include "../scoring/scoring_base.h"
#include "ant_scoring.h"


using namespace opencog;
using namespace moses;
using namespace reduct;
using namespace boost;
using namespace ant_combo;
using namespace std;

int main(int argc,char** argv) 
{
    int max_evals,rand_seed;
    if (argc == 3) {
        rand_seed=lexical_cast<int>(argv[1]);
        randGen().seed(rand_seed);
        max_evals=atoi(argv[2]);
    } else {
        cerr << "usage: " << argv[0] << " seed maxevals" << endl;
        exit(1);
    }

    // Simplificity pressure
    float simplicity_pressure = 0.5;

    // Multi-thread
    static const string localhost = "localhost";
    unsigned n_jobs = 4;
    jobs_t jobs{{localhost, n_jobs}};
    setting_omp(jobs[localhost]);

    // Logger setting
    static const string log_file = "moses-ant-hillclimbing.log";
    remove(log_file.c_str());
    logger().setLevel(Logger::DEBUG);
    logger().setFilename(log_file);

    type_tree tt(id::lambda_type);
    tt.append_children(tt.begin(),id::action_result_type,1);

    ant_bscore bscorer(simplicity_pressure);
    behave_cscore cscorer(bscorer);

    combo_tree_ns_set perceptions;
    combo_tree_ns_set actions;

    actions.insert(combo_tree(get_instance(id::turn_left)));
    actions.insert(combo_tree(get_instance(id::turn_right)));
    actions.insert(combo_tree(get_instance(id::move_forward)));

    perceptions.insert(combo_tree(get_instance(id::is_food_ahead)));

    metapop_parameters metaparms;
    metaparms.perceptions = &perceptions;
    metaparms.actions = &actions;
    metaparms.complexity_temperature = 100;

    // Define optimization algo
    optim_parameters opt_params;
    opt_params.max_dist = 3;
    hc_parameters hc_params;
    hc_params.widen_search = true;
    hc_params.crossover = true;
    hill_climbing hc(opt_params, hc_params);

    metapopulation metapop(combo_tree(id::sequential_and), tt, action_reduction(),
                           cscorer, bscorer, hc, metaparms);
  
    boost::program_options::variables_map vm;

    moses_parameters moses_param(vm, jobs, true, max_evals, -1, 0, 100);
    moses_statistics st;
    run_moses(metapop, moses_param, st);
}

