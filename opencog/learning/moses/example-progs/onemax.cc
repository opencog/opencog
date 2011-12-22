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

int main(int argc,char** argv)
{
    // Set flag to print only cassert and other ERROR level logs on stdout.
    logger().setPrintErrorLevelStdout();

    optargs args(argc,argv);

    cout_log_best_and_gen logger;

    // Create a
    field_set fs(field_set::disc_spec(2), args.length);
    MT19937RandGen rng(args.rand_seed);

    instance_set<int> population(args.popsize,fs);
    foreach(instance& inst,population)
        generate(fs.begin_bits(inst), fs.end_bits(inst),
                 bind(&RandGen::randbool, boost::ref(rng)));

    optimize(population,args.n_select, args.n_generate,args.max_gens,
             one_max(), terminate_if_gte<int>(args.length),
             tournament_selection(2, rng),
             univariate(),local_structure_probs_learning(),
             replace_the_worst(),logger, rng);
}
