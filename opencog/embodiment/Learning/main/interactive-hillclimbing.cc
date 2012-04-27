/*
 * opencog/embodiment/Learning/main/interactive-hillclimbing.cc
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

#include <opencog/embodiment/Learning/PetaverseHC/interactive-hillclimber.h>
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/util/lru_cache.h>
#include <set>
#include <string>
#include <algorithm>
#include <iostream>

using namespace opencog;
using namespace hillclimbing;
using namespace std;

struct InteractiveFitnessFunction : unary_function<combo_tree, fitness_t> {
    result_type operator()(argument_type tr) const {
        cout << "Fitness Function of : " << tr << " enter the score :" << endl;
        fitness_t score = 0.0;
        cin >> score;
        return score;
    }
};

struct InteractiveFitnessEstimator : unary_function<combo_tree, fitness_t> {
    result_type operator()(argument_type tr) const {
        cout << "Fitness Estimation of : " << tr << " enter the score :" << endl;
        fitness_t score = 0.0;
        cin >> score;
        return score;
    }
};

int main(int argc, char** argv)
{

    int nepc = 1000;

    if (argc > 1)
        nepc = atoi(argv[1]);

    InteractiveFitnessFunction iff;
    InteractiveFitnessEstimator ife;

    interactive_hillclimber<InteractiveFitnessEstimator> hc(ife, nepc);

    int i = 0;

    while (true) {
        cout << "HillClimbing Iteration " << i << " :" << endl;
        for (int j = 0; j < nepc; j++) {
            std::cout << "CYCLE : " << j << std::endl;
            hc();
        }
        hc.set_current_fitness(iff(hc.current_program()));
        hc();
        cout << "************************************************" << endl;
        cout << "* Best program (gotten from the previous iteration) : "
             << hc.best_program() << " with score : " << hc.best_fitness() << endl;
        cout << "* Best program estimated : " << hc.best_program_estimated() <<
             " with score : " << hc.best_fitness_estimated() << endl;
        cout << "* Current program : " << hc.current_program() <<
             " with score : " << hc.current_fitness() << endl;
        cout << "************************************************" << endl;
        i++;
    }

}
