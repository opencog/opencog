/*
 * opencog/learning/moses/example-progs/continmax.cc
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

using std::string;
using std::vector;
using boost::lexical_cast;

// Demonstration program for the "max" optimization problem, for
// continuous variables.  This is a variation of the standard
// onemax/nmax demonstraton problems. Here, a scoring function is
// used that sums the squares of the values of a set of discrete
// variables.  This is the "sphere" scoring function.  The optimizer is
// supposed to be able to find the best solution to this function:
// namely, all of the variables should be zero.  Although finding a
// solution is trivial if one applies a bit of calculus, it becomes
// a bit tricker for evoilutionary algorithms, primarily because the
// need to handle continuously-valued variables makes the size of the
// search space uncountable (cardinality of the continuum), and because
// evolutionary algorithms typically do not track derivatives (which
// is how the problem becomes easy when applying calculus).
//
// XXX Currently, this doesn't really work well, or maybe at all, in
// part because the contin implementation in the field set is incomplete
// or broken or maybe both; there is a confusion between depth and arity
// in that code. (i.e. confusion between depth and breadth, between arity
// and log_2(arity), etc.) See
// https://bugs.launchpad.net/opencog/+bug/908247
//
// NOTE: This is NOT a demonstration of program learning, which is what
// MOSES is designed for; rather, this a demonstration of the use of a
// certain component within MOSES, the so-called "optimizer". MOSES itself
// relies heavily on this optimizer to implement its meta-optimization
// algorithm.
//
// This program requires five arguments:
// -- an initial seed value for the random number generator
// -- the number of contin variables
// -- the population size
// -- the maximum number of generations to run.
// -- the number that is -log_2(epsilon) where epsilon is the smallest
//    distinction between continuous variables what will be drawn.
//
// XXX todo -- finish documentation to make it look more like the
// onemax/nmax example programs.

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
    vector<string> add_args{"<depth>"};
    optargs args(argc, argv, add_args);
    int depth = lexical_cast<int>(argv[5]);

    // Initialize random number generator (from the first argument
    // given to the program).
    randGen().seed(args.rand_seed);

    // Create a set of "fields". Each field is a contin variable.

    /*field_set fs(field_set::spec(field_set::contin_spec(2.0,2.5,0.5,depth),
      args.length));*/
    field_set fs(field_set::contin_spec(0.0, 0.5, 0.5, depth), args.length);
    instance_set<contin_t> population(args.popsize, fs);
    for (instance& inst : population) {
        occam_randomize_contin(fs, inst);
        cout << fs.to_string(inst) << endl;
        cout << fs.to_string_raw(inst) << endl;
    }

    contin_t epsilon = fs.contin().front().epsilon();

    // Run the optimizer.
    int num_score_evals =
    optimize(population,   // population of instances, from above.
             args.popsize,                       // num to select
             args.popsize / 2,                   // num to generate
             args.max_gens,                      // max number of generations to run
             sphere(fs),                         // ScoringPolicy
             terminate_if_gte<contin_t>(-args.length*epsilon), // TerminationPolicy
             //terminate_if_gte(args.length*(7-2*epsilon)*(7-2*epsilon)),

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

#if 0
    // Show the final population
    // cout << "Final population:\n" << population << endl;
    cout << "The final population was:" << endl;
    instance_set<int>::const_iterator it = population.begin();
    for(; it != population.end(); it++) {
       cout << "Score: " << it->second
            << "\tindividual: " << population.fields().to_string(it->first)
            << endl;
    }
#endif
}
