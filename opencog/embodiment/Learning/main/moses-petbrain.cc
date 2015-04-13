/*
 * opencog/embodiment/Learning/main/moses-petbrain.cc
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

#include <iostream>

#include <boost/lexical_cast.hpp>

#include <opencog/util/mt19937ar.h>

#include <opencog/comboreduct/reduct/reduct.h>

#include <opencog/learning/moses/moses/moses_main.h>
#include <opencog/learning/moses/scoring/behave_cscore.h>
#include <opencog/learning/moses/optimization/optimization.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>


using namespace opencog;
using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;

struct interactive_bscore : public bscore_base
{
    result_type operator()(const combo_tree& tr) const
    {
        cout << "Fitness Function of : " << tr << " enter the score :" << endl;
        score_t score = 0.0;
        cin >> score;
        result_type pbs;
        pbs.push_back(score);
        return pbs;
    }
    behavioral_score best_possible_bscore() const
    {
        behavioral_score bs;
        return bs;
    }
};


int main(int argc, char** argv)
{
    int max_evals;
    try {
        if (argc != 3)
            throw "foo";
        int rand_seed = lexical_cast<int>(argv[1]);
        max_evals = lexical_cast<int>(argv[2]);
        randGen().seed(rand_seed);
    } catch (...) {
        cerr << "usage: " << argv[0] << " seed maxevals" << endl;
        exit(1);
    }

    type_tree tt(id::lambda_type);
    tt.append_children(tt.begin(), id::action_result_type, 1);

    interactive_bscore bscorer;
    behave_cscore cscorer(bscorer);
    hill_climbing climber;

    combo_tree_ns_set perceptions;
    combo_tree_ns_set actions;

    actions.insert(combo_tree(ant_combo::get_instance(id::turn_left)));
    actions.insert(combo_tree(ant_combo::get_instance(id::turn_right)));
    actions.insert(combo_tree(ant_combo::get_instance(id::move_forward)));

    perceptions.insert(combo_tree(ant_combo::get_instance(id::is_food_ahead)));

    deme_parameters demeparms;
    demeparms.perceptions = &perceptions;
    demeparms.actions = &actions;

    deme_expander dex(tt, action_reduction(), action_reduction(),
                      cscorer, climber, demeparms);
    metapopulation metapop(combo_tree(id::sequential_and), cscorer);

    cout << "build metapop" << endl;


    boost::program_options::variables_map vm;
    jobs_t jobs;
    moses_parameters moses_param(vm, jobs, true, max_evals, -1, 0);
    moses_statistics stats;

    run_moses(metapop, dex, moses_param, stats);
}
