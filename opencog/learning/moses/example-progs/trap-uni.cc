/*
 * opencog/learning/moses/example-progs/trap-uni.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Linas Vepstas, 2011
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

using boost::lexical_cast;

// Demonstration program for the "univariate trap" optimization problem.
// This is a standard learning/optimization demonstraton problem: a 
// scoring function is given that surrounds the correct solution with
// nearby solutions which are given a very low score.  This is related
// to the "nmax" problem, except for having a vee-shaped scoring function.
// The vee-shape is meant to be "deceptive", in that it fools ordinary
// GA or hill-climbing solvers into loking for a solution in the wrong
// place (i.e. on the wrong side of the vee -- the vee isolates the 
// singular, unique global maximum by a valley on all sides, which is
// densely surrounded by exponentially many local but inferior maxima.)
//
// As such problems go, this turns out to be "deceptively" easy for
// the moses univariate optimizer to solve. This is because it has been
// constructed to treat each possible value of a variable as independent,
// and analyzing probability distributions for these.  Thus, the
// optimizer doesn't perceive the "vee" shape in the scoring function.
// It is able to solve this problem at exactly the same efficiency as the
// nmax problem.  The code here just illustrates this.
//
// NOTE: This is NOT a demonstration of program learning, which is what
// MOSES is designed for; rather, this a demonstration of the use of a
// certain component within MOSES, the so-called "optimizer". MOSES itself
// relies heavily on this optimizer to implement its meta-optimization
// algorithm.
//
//
// This program requires five arguments:
// -- an initial seed value for the random number generator
// -- the number of discrete variables
// -- the population size
// -- the maximum number of generations to run.
// -- the number of values that variables may take.
//
// Suggest values of 0 8 70 100 5 for these.
//
// The population size should always be at least (n-1)*num_variables
// as otherwise, the least-fit individuals will be bred out of
// the population before the fittest individual is found, causing the
// algo loop forever (well, only up to max generations). There's even
// some risk of this when the population is only a little larger than
// this lower bound; maybe one-n-a-half or twice this lower bound would
// be a good choice.
//
// As output, this will print the fittest individual found at each
// generation. At the conclusion, the entire population will be printed.

// Return, as the score, the trap function. The trap has a V-shape,
// giving highest score for bit strings with all bits set, 
// -----------------------------------------------------------

// Trap scoring function.
// The per-variable scoring function is implemented in "vee()".
// The operator() function computes the score for a given instance.

struct trap : public unary_function<instance, int>
{
    trap(const field_set& fs, int n) : fields(fs), max(n-1) {}

    int vee(int x) const
    {
        if (max <= x) return x;
        return max-1-x;
    }

    int operator()(const instance& inst) const
    {
        return accumulate
               (make_transform_iterator(fields.begin_disc(inst),
                    bind(&trap::vee, this, _1)),
                make_transform_iterator(fields.end_disc(inst), 
                    bind(&trap::vee, this, _1)),
                0);
    }

private:
    const field_set& fields;
    int max;
};


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
    vector<string> add_args{"<trap size>"};
    optargs args(argc, argv, add_args);
    int n = lexical_cast<int>(argv[5]);


    // Initialize random number generator (from the first argument
    // given to the program).
    randGen().seed(args.rand_seed);

    // Create a set of "fields". Each field is a discrete variable,
    // with 'n' different possible settings. That is, each field has
    // a multiplicity or "arity" of 'n'.  The number of such discrete
    // variables to create was passed as the second argument to the
    // program.
    field_set fs(field_set::disc_spec(n), args.length);

    // Create a population of instances (bit-strings) corresponding
    // to the field specification above. The length of the bit string
    // will be ciel(log_2(n)) times the length of the field specification.
    // This is because 'n' values require at least log_2(n) bits to be
    // represented in binary.  The population size was passed as the
    // third argument to the program.
    instance_set<int> population(args.popsize, fs);

    // Initialize each member of the population to a random value.
    for (instance& inst : population)
        generate(fs.begin_disc(inst), fs.end_disc(inst),
                 bind(&RandGen::randint, boost::ref(randGen()), n));

    // Run the optimizer.
    // For this problem, there is no dependency at all between different
    // fields ("genes") in the field set.  Thus, for the "structure
    // learning" step, use the univariate() model, which is basically a
    // no-op; it doesn't try to learn any structure.
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
    // The trap() scoring function is described above.
    //
    // The termination policy will halt iteration if an individual is
    // discovered to have a score of "(n-1)*args.length" -- but of
    // course, since each field takes a value from 0 to (n-1) so the
    //
    int num_score_evals =
    optimize(population,   // population of instances, from above.
             args.popsize,                       // num to select
             args.popsize / 2,                   // num to generate
             args.max_gens,                      // max number of generations to run
             trap(fs, n),                        // ScoringPolicy
             terminate_if_gte<int>((n-1)*args.length), // TerminationPolicy
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
