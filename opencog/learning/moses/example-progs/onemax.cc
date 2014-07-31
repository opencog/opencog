/*
 * opencog/learning/moses/example-progs/onemax.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
 * Documented by Linas Vepstas, 2011
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
// bit-string of all ones.  The onemax problem is a special case of the
// "nmax" problem, with 'n'=2.
//
// NOTE: This is NOT a demonstration of program learning, which is what
// MOSES is designed for; rather, this a demonstration of the use of a
// certain component within MOSES, the so-called "optimizer". MOSES itself
// relies heavily on this optimizer to implement its meta-optimization
// algorithm.
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
    // Set to Logger::WARN to show only warnings and errors.
    logger() = Logger("demo.log");
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
    randGen().seed(args.rand_seed);

    // Create a set of "fields". Each field is a discrete variable,
    // with two possible settings. That is, each field is a boolean.
    // The number of such boolean variables to create was passed as
    // the second argument to the program.
    field_set fs(field_set::disc_spec(2), args.length);

    // Create a population of bit-strings corresponding to the field
    // specification above. The length of the bit string will be the
    // same as the length of the field specification: one bit per field.
    // The population size was passed as the third argument to the
    // program.
    instance_set<int> population(args.popsize, fs);

    // Initialize each member of the population to a random value.
    for (instance& inst : population)
        generate(fs.begin_bit(inst), fs.end_bit(inst),
                 bind(&RandGen::randbool, boost::ref(randGen())));

    // Run the optimizer.
    // For this problem, there is no dependency at all between different
    // bits in the bit string.  Thus, for the "structure learning" step,
    // use the univariate() model, which is basically a no-op; it doesn't
    // try to learn any structure.
    //
    // The "num to select" argument is number of individuals to select
    // for learning the population distribution. For this problem, it
    // makes sense to select them all.  For smaller selections, the
    // SelectionPolicy is used to make the selection; here, the
    // tournament_selection() policy is used to select the fittest
    // individuals from the population. Since we select all, holding
    // a tournament is pointless.
    //
    // The "num to generate" is the number of individuals to create for
    // the next generation.  These are created with reference to the
    // learned model.  If the model is working well, then the created
    // individuals should be fairly fit.  In this example, it makes
    // sense to replace half the population each generation.
    // The generated individuals are then folded into the population
    // using the replace_the_worst() replacement policy. This
    // replacement policy is unconditional: the worst part of the
    // current population is replaced by the new individuals (even if
    // the new individuals are less fit than the current population!
    // But this is good enough for this example...)
    //
    // The one_max() scoring function simply counts the number of
    // one-bits in the bit-string. It is defined in scoring_functions.h
    // The termination policy will halt iteration if an individual is
    // discovered to have a score of "args.length" -- but of course,
    // this is the bit string length, and such a score means all bits
    // are one.
    //
    int num_score_evals =
    optimize(population,   // population of bit strings, from above.
             args.popsize,                       // num to select
             args.popsize / 2,                   // num to generate
             args.max_gens,                      // max number of generations to run
             one_max(),                          // ScoringPolicy
             terminate_if_gte<int>(args.length), // TerminationPolicy
             tournament_selection(2),            // SelectionPolicy
             univariate(),                       // StructureLearningPolicy
             local_structure_probs_learning(),   // ProbsLearningPolicy
             replace_the_worst(),                // ReplacementPolicy
             mlogger);

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
            << "\tindividual: " << population.fields().to_string(it->first)
            << endl;
    }
}
