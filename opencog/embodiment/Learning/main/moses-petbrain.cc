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

#include <opencog/learning/moses/moses/moses.h>
#include <opencog/learning/moses/moses/scoring_functions.h>
#include <opencog/learning/moses/optimization/optimization.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>


using namespace opencog;
using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;

int main(int argc, char** argv)
{
    int max_evals, rand_seed;
    try {
        if (argc != 3)
            throw "foo";
        rand_seed = lexical_cast<int>(argv[1]);
        max_evals = lexical_cast<int>(argv[2]);
    } catch (...) {
        cerr << "usage: " << argv[0] << " seed maxevals" << endl;
        exit(1);
    }

    MT19937RandGen rng(rand_seed);

    type_tree tt(id::lambda_type);
    tt.append_children(tt.begin(), id::action_result_type, 1);

    interactive_score scorer;
    interactive_bscore bscorer;
    hill_climbing climber(rng);

    metapopulation<interactive_score, interactive_bscore, hill_climbing>
        metapop(rng, combo_tree(id::sequential_and), tt, action_reduction(),
                scorer, bscorer, climber);

    cout << "build metapop" << endl;

    operator_set ignore_ops;
    combo_tree_ns_set perceptions;
    combo_tree_ns_set actions;

    actions.insert(combo_tree(ant_combo::get_instance(id::turn_left)));
    actions.insert(combo_tree(ant_combo::get_instance(id::turn_right)));
    actions.insert(combo_tree(ant_combo::get_instance(id::move_forward)));

    perceptions.insert(combo_tree(ant_combo::get_instance(id::is_food_ahead)));


    moses_parameters moses_param(max_evals, -1, 0,
                                 ignore_ops, &perceptions, &actions);

    //had to put namespace moses otherwise gcc-4.1 complains that it is ambiguous
    moses::moses(metapop, moses_param);
}
