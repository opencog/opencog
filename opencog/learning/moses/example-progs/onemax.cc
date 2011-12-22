/*
 * opencog/learning/moses/example-progs/onemax.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
 * Documented by Linas Vepstas
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
// below illustrates how to do this using the MOSES infrastructure.
//
// This program requires four arguments:
// -- an initial seed value for the random number generator
// -- the bit-string length
// -- the population size
// -- the maximum number of generations to run. 
//
// Suggest values of 0 15 20 100 for these.
//
// The population size should always be larger than the bit-string
// length, as otherwise, the least-fit individuals will be bred out of
// the population before the fittest individual is found, causing the
// algo loop forever (well, only up to max generations). There's even
// some risk of this when the population is only a little larger than
// the bit-string length.
//
// As output, this will print the fittest individual found at each
// generation. At the conclusion, the entire population will be printed.

int main(int argc, char** argv)
{
    // Tell the system logger to print detailed debugging messages to
    // stdout. This will let us watch what the optimizer is doing.
    // Set to Logger::WARN to only show arnings and errors.
    logger().setLevel(Logger::FINE);
    logger().setPrintToStdoutFlag(true);

    // We also need to declare a specific logger for the aglo.
    // This one uses the same system logger() above, and writes all
    // messages ad the "debug" level. This allows the main loop of the
    // algo to be traced.
    cout_log_best_and_gen mlogger;

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

    // Run the optimizer.
    // For this problem, there is no dependency at all between different
    // bits in the bit string.  Thus, for the "structure learning" step,
    // use the univariate() model, which is basically a no-op; it doesn't
    // try to learn any structure.
// xxx explain why num to select is popsize ... 
// num to generate is the num to evaluate ... 
    int num_score_evals = 
    optimize(population,   // population fo bit strings, from above.
             args.popsize,                       // num to select
             args.popsize / 2,                   // num to generate
             args.max_gens,                      // max number of generations to run
             one_max(),                          // ScoringPolicy
             terminate_if_gte<int>(args.length), // TerminationPolicy
             tournament_selection(2, rng),       // SelectionPolicy
             univariate(),                       // StructureLearningPolicy
             local_structure_probs_learning(),
             replace_the_worst(),
             mlogger,
             rng);

    // The logger is asynchronous, so flush it's output before
    // writing to cout, else output will be garbled.
    logger().flush();

    cout << "A total of " << num_score_evals
         << " scoring funtion evaluations were done." << endl;

    // Show the final population
    // cout << "Final population:\n" << population << endl;
    cout << "The final population was:" << endl;
    instance_set<int>::const_iterator it = population.begin();
    for(; it != population.end(); it++) {
       cout << "Score: " << it->second
            << "\tindividual: " << population.fields().stream(it->first)
            << endl;
    }
}
