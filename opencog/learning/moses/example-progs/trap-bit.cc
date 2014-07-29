/*
 * opencog/learning/moses/example-progs/trap-bit.cc
 *
 * Copyright (C) 2011 Linas Vepstas
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

// XXX under construction XXX

// XXX this example is broken, and will remain so until "multivariate"
// is ported over/re-implemented. Basically, there is no structure
// learning at this time.  
// XXX some of the documentation below may be misleading.
//
// Demonstration program for the "bit-trap" optimization problem.
// This is a standard learning/optimization demonstraton problem: a 
// scoring function is given that surrounds the correct solution with
// nearby solutions which are given a very low score.  This is a variant
// of the "univariate-trap" problem, the difference here being that
// the population being optimized over is a set of bit-strings, and the
// scoring functions make different bits depend on one-another.  Thus,
// the optimal solution cannot be found without structure learning; the
// MOSES univariate() learner is a no-op, and so cannot solve this problem.
// XXX which is why we need to put structure leanring back in the code XXX
//
// The correlation between variables is accomplished by using a
// vee-shaped scoring function. The vee-shape is meant to be "deceptive",
// in that it fools ordinary GA or hill-climbing solvers into loking
// for a solution in the wrong place (i.e. on the wrong side of the vee
// -- the vee isolates the singular, unique global maximum by a valley
// on all sides, which is densely surrounded by combinatorially many
// local but inferior maxima.)
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
// The population size should always be at least 2^n*num_variables
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
// The per-trap scoring function is implemented in "vee()".
// The operator() function computes the score for a given instance.

struct trap : public unary_function<instance, int>
{
    trap(int n, int len) : trapsz(n), rounds(len) {}

    int vee(int x) const
    {
        if (trapsz <= x) return x;
        return trapsz-1-x;
    }

    // This scoring function loops over the bits in the bit string,
    // grouping them into "traps" of "trapsz" bits each.  The number of
    // bits in each trap is counted, and then passed through the
    // trap function. The trap score is then accumulated.  The length
    // of the bit string must be a multiple of the trap size, else
    // the trailing fractional bits will be ignored.
    int operator()(const instance& inst) const
    {
        instance::const_iterator it = inst.begin();
        int odo = 0;   // odometer
        int total = 0;
        int subtotal = 0;
        int nr = 0;
        for (; it != inst.end(); it ++)
        {
            packed_t bits = *it;

            // loop over bits in a packed_t
            for (unsigned int b=0; b<sizeof(packed_t)*8; b++)
            {
                subtotal += bits & 0x1;
                odo++;
                bits = bits >> 1;
                if (0 == odo%trapsz)
                {
                     total += vee(subtotal);
                     subtotal = 0;
                     nr ++;
                     if (rounds < nr) break;
                }
            }
        }
        return total;
    }

private:
    int trapsz;
    int rounds;
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
    // with two possible settings. That is, each field is a boolean.
    // The number of such boolean variables to create will be the
    // length (second argument) times the trap size (last argument).
    field_set fs(field_set::disc_spec(2), n*args.length);

    // Create a population of instances (bit-strings) corresponding
    // to the field specification above. The length of the bit string
    // will be ciel(log_2(n)) times the length of the field specification.
    // This is because 'n' values require at least log_2(n) bits to be
    // represented in binary.  The population size was passed as the
    // third argument to the program.
    instance_set<int> population(args.popsize, fs);

    // Initialize each member of the population to a random value.
    for (instance& inst : population)
        generate(fs.begin_bit(inst), fs.end_bit(inst),
                 bind(&RandGen::randbool, boost::ref(randGen())));

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
             trap(n, args.length),               // ScoringPolicy
             terminate_if_gte<int>(n*args.length), // TerminationPolicy
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
