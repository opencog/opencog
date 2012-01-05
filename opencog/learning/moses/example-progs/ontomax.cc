/*
 * opencog/learning/moses/example-progs/ontomax.cc
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

using std::string;
using std::vector;
using boost::lexical_cast;

// This is some sort of term-algebra variant of the onemax/nmax problem.
// I don't think it works, the term support in MOSES is incomplete and/or
// broken. Algorithmically, the term-algebra support is similar to the
// contin support, but for general terms arranged in an n-ary tree, 
// instead of a 2-ary tree for contin.
//
// XXX Someday, fix all of this!

void recbuild(term_tree& tr, term_tree::iterator it,
	      int b, int maxd, int d, int s)
{
    *it = lexical_cast<string>(d)+lexical_cast<string>(s);
    if (d<maxd) {
        tr.append_children(it,b);
        int child_s=0;
        for (term_tree::sibling_iterator sib = it.begin(); sib != it.end(); ++sib)
            recbuild(tr, sib, b, maxd, d+1, s*b+child_s++);
    }
}

int main(int argc,char** argv)
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
    vector<string> addition_args{"<depth>", "<branching>"};
    optargs args(argc, argv, addition_args);
    int depth = lexical_cast<int>(argv[5]);
    int branching = lexical_cast<int>(argv[6]);

    // Initialize random number generator (from the first argument
    // given to the program).
    MT19937RandGen rng(args.rand_seed);

    term_tree tr("");
    recbuild(tr, tr.begin(), branching, depth, 0, 0);
    field_set fs(field_set::term_spec(tr), args.length);
    instance_set<contin_t> population(args.popsize,fs);
    foreach(instance& inst,population) {
        occam_randomize_term(fs,inst,rng);
    }


    int num_score_evals =
    optimize(population,   // population of bit strings, from above.
             args.popsize,                       // num to select
             args.popsize / 2,                   // num to generate
             args.max_gens,                      // max number of generations to run
             termmax(fs),                        // ScoringPolicy
             terminate_if_gte<contin_t>((depth+pow(float(branching),
                                                   depth)-1)*args.length),
                                                 // TerminationPolicy
             tournament_selection(2, rng),       // SelectionPolicy
             univariate(),                       // StructureLearningPolicy
             local_structure_probs_learning(),   // ProbsLearningPolicy
             replace_the_worst(),                // ReplacementPolicy
             mlogger,
             rng);

    // The logger is asynchronous, so flush it's output before
    // writing to cout, else output will be garbled.
    logger().flush();

    cout << "A total of " << num_score_evals
         << " scoring funtion evaluations were done." << endl;

}
