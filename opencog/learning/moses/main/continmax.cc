/*
 * opencog/learning/moses/main/continmax.cc
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
#include "edaopt.h"
#include <opencog/learning/moses/eda//initialization.h>
#include <opencog/util/mt19937ar.h>

int main(int argc,char** argv) { 
  cassert(TRACE_INFO, argc==6);
  optargs args(argc,argv);
  int depth=lexical_cast<int>(argv[5]);
  cout_log_best_and_gen logger;

  opencog::MT19937RandGen rng(args.rand_seed);

  /*field_set fs(field_set::spec(field_set::contin_spec(2.0,2.5,0.5,depth),
    args.length));*/
  field_set fs(field_set::contin_spec(0.0,0.5,0.5,depth),args.length);
  instance_set<contin_t> population(args.popsize,fs);
  foreach(instance& inst,population) {
    occam_randomize_contin(fs,inst,rng);
    cout << fs.stream(inst) << endl;
    cout << fs.stream_raw(inst) << endl;
  }

  contin_t epsilon=fs.contin().front().epsilon();
  optimize(population,args.n_select,args.n_generate,args.max_gens,
	   sphere(fs),
	   terminate_if_gte<contin_t>(-args.length*epsilon),
	   //terminate_if_gte(args.length*(7-2*epsilon)*(7-2*epsilon)),
	   tournament_selection(2, rng),
	   univariate(),local_structure_probs_learning(),
	   replace_the_worst(),logger, rng);
}
