/*
 * opencog/learning/moses/example-progs/onemax.cc
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

#include "headers.h"

// Demonstration program for the "onemax" optimization problem.  This
// is a standard learning/optimization demonstraton problem: a scoring
// function is given that counts the number of one-bits in a bit-string.
// This is the "one_max" scoring function.  The optimizer is supposed
// to be able to find the best solution to this function: namely, a
// bit-string of all ones.
//
// As such problems go, this is among the simplest to solve.  The code
// below illustrates how to do this using teh MOSES infrastructure.

int main(int argc, char** argv)
{
    // Tell the logger to print detailed debugging messages to stdout.
    // This will let us watch what the optimizer is doing.
    logger().setLevel(Logger::FINE);
    logger().setPrintToStdoutFlag(true);

    // Parse program arguments
    optargs args(argc, argv);

    // Initialize random number generator (from the first argument
    // given to the program).
    MT19937RandGen rng(args.rand_seed);

    // Create a set of "fields". Each field is a discrete variable,
    // with two possible settings. That is, each field is a boolean.
    // The number of such boolean variables to create was passed as
    // the second argument to the program.
    field_set fs(field_set::disc_spec(2), args.length);

    // Create a population of bit-strings corresponding to the field
    // specification above. The length of the bit string will be the
    // same as the field specification. The population size was passed
    // as the third argument to the program.
    instance_set<int> population(args.popsize, fs);

    // Initialize each member of the population to a random value.
    foreach(instance& inst, population)
        generate(fs.begin_bits(inst), fs.end_bits(inst),
                 bind(&RandGen::randbool, boost::ref(rng)));

    // Declare a logger, where debug and logging messages will be written.
    cout_log_best_and_gen logger;

    // Run the optimizer.  
// why num to seldet for demes is popsize ... 
// num to generate is the num to evaluate ... 
    optimize(population,   // population fo bit strings, from above.
             args.popsize,                     // num to select
             args.popsize / 2,                 // num to generate
             args.max_gens,                    // max number of generations to run
             one_max(),                        // scoring function
             terminate_if_gte<int>(args.length), // termination criterion
             tournament_selection(2, rng),
             univariate(),  // why ?? 
             local_structure_probs_learning(),  // Useless ...!? no structure!
             replace_the_worst(),
             logger,
             rng);

    // XXX show how to demo the results.
    // cout << "Found this" << population << endl;
}
