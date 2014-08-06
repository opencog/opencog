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
#include <stdlib.h>

#include <iostream>
#include <string>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/numeric.h>
#include <opencog/util/oc_omp.h>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>
#include <opencog/comboreduct/reduct/reduct.h>

#include "../deme/deme_expander.h"
#include "../metapopulation/metapopulation.h"
#include "../moses/moses_main.h"
#include "../optimization/optimization.h"
#include "../scoring/scoring_base.h"
#include "../scoring/behave_cscore.h"
#include "ant_scoring.h"


using namespace opencog;
using namespace moses;
using namespace reduct;
using namespace boost;
using namespace ant_combo;
using namespace std;

// This demo is nearly identical to the AntUTest unit test.
//
// The performance of this demo is very highly dependent on the
// initial random number seed. At the time of this writing, some
// seeds result in a solution being found in a few seconds or
// less, while others take at least minutes, or longer. The
// bottom of the file opencog/learning/moses/diary-performance.txt
// provides more information.

int main(int argc,char** argv)
{
    int max_evals,rand_seed;
    if (argc == 3) {
        rand_seed = atoi(argv[1]);
        randGen().seed(rand_seed);
        max_evals = atoi(argv[2]);
    } else {
        cerr << "usage: " << argv[0] << " seed maxevals" << endl;
        exit(1);
    }

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
    tt.append_children(tt.begin(), id::action_result_type, 1);

    ant_bscore bscorer;
    // See the diary for the complexity ratio.
    double complexity_ratio = 0.16;
    bscorer.set_complexity_coef(complexity_ratio);
    behave_cscore cscorer(bscorer);

    combo_tree_ns_set perceptions;
    combo_tree_ns_set actions;

    actions.insert(combo_tree(get_instance(id::turn_left)));
    actions.insert(combo_tree(get_instance(id::turn_right)));
    actions.insert(combo_tree(get_instance(id::move_forward)));

    perceptions.insert(combo_tree(get_instance(id::is_food_ahead)));

    deme_parameters demeparms;
    demeparms.perceptions = &perceptions;
    demeparms.actions = &actions;

    metapop_parameters metaparms;
    metaparms.complexity_temperature = 2000;  // See diary entry

    // Define optimization algo
    optim_parameters opt_params;
    hc_parameters hc_params;
    hc_params.widen_search = false;  // Same as default
    hc_params.crossover = true;      // Same as default
    hill_climbing hc(opt_params, hc_params);

    deme_expander dex(tt, action_reduction(),
                          action_reduction(), cscorer, hc, demeparms);
    metapopulation metapop(combo_tree(id::sequential_and),
                           cscorer, metaparms);

    boost::program_options::variables_map vm;

    moses_parameters moses_param(vm, jobs, true, max_evals, -1, 0, 100);
    moses_statistics st;
    run_moses(metapop, dex, moses_param, st);
}

