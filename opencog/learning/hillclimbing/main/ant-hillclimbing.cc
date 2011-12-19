/*
 * opencog/learning/hillclimbing/main/ant-hillclimbing.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#include <opencog/learning/hillclimbing/ant-hillclimber.h>
#include <opencog/comboreduct/combo/vertex.h>
#include <set>
#include <string>
#include <algorithm>
#include <iostream>
#include "../../moses/example-ant/ant_scoring.h"

using namespace opencog;
using namespace hillclimbing;
using namespace combo;
using namespace std;

int main(int argc, char** argv)
{

    logger().setPrintErrorLevelStdout();

    int number_of_steps = 600;
    int number_of_iterations = 1000;
    int noise = 0;
    int number_of_estimation = 1000; //number of estimations per cycle
    int number_of_cycle_per_iteration = 1000;

    if (argc != 6 || (argc == 2 && strcmp(argv[1], "-h") == 0)) {
        std::cout << "Usage :" << std::endl;
        std::cout << "ant-hillclimbing number_of_steps number_of_iterations noise number_of_estimation_per_cycle number_of_cycles" <<
                  std::endl;
        exit(1);
    }
    if (argc > 1)
        number_of_steps = atoi(argv[1]);
    if (argc > 2)
        number_of_iterations = atoi(argv[2]);
    if (argc > 3)
        noise = atoi(argv[3]);
    if (argc > 4)
        number_of_estimation = atoi(argv[4]);
    if (argc > 5)
        number_of_cycle_per_iteration = atoi(argv[5]);

    opencog::MT19937RandGen rng(0);

    AntFitnessFunction aff(number_of_steps);
    AntFitnessEstimator afe(rng, number_of_steps, noise);

    ant_hillclimber<AntFitnessEstimator> antHC(afe, number_of_estimation);

    int i = 0;

    while (i < number_of_iterations) {
        cout << "Ant HillClimbing Iteration " << i << " :" << endl;
        //std::cout << "CYCLE : " << 0 << std::endl;
        antHC();//the first one is for the control
        //std::cout << "CURRENT PROG : " << antHC.current_program()
        //      << " CURRENT FIT EST : " << antHC.current_fitness() << std::endl;
        for (int j = 0; j < number_of_cycle_per_iteration; ++j) {
            //std::cout << "CYCLE : " << j+1 << std::endl;
            antHC();
            //std::cout << "CURRENT PROG : " << antHC.current_program()
            // << " CURRENT FIT EST : " << antHC.current_fitness()
            // << std::endl;
        }
        antHC.set_current_fitness(aff(antHC.current_program(rng)));
        //std::cout << "CYCLE : " << number_of_cycle_per_iteration+1 << std::endl;
        antHC();
        //std::cout << "CURRENT PROG : " << antHC.current_program()
        //      << " CURRENT FIT EST : " << antHC.current_fitness() << std::endl;
        cout << "************************************************" << endl;
        cout << "* Best program (gotten from the previous sub-step) : "
             << antHC.best_program() <<
             " with score : " << antHC.best_fitness() << endl;
        cout << "* Best program estimated : " << antHC.best_program_estimated() <<
             " with score : " << antHC.best_fitness_estimated() << endl;
        cout << "* Current program : " << antHC.current_program(rng) <<
             " with score : " << antHC.current_fitness() << endl;
        cout << "************************************************" << endl;
        i++;
    }

}
