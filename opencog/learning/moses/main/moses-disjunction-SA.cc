/*
 * opencog/learning/moses/main/moses-disjunction.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Predrag Janicic
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
#include <opencog/learning/moses/moses/moses.h>
#include <opencog/learning/moses/moses/optimization.h>
#include <opencog/learning/moses/moses/scoring_functions.h>
#include <boost/lexical_cast.hpp>
#include <opencog/comboreduct/reduct/reduct.h>
#include <iostream>
#include <opencog/util/mt19937ar.h>

using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;


int main(int argc,char** argv) { 
  int arity,max_evals,rand_seed;
  double min_temp, init_temp, temp_step_size;
  double accept_prob_temp_intensity, dist_temp_intensity, fraction_of_remaining;
      
  try {
    if (argc!= 10)
      throw "foo";
    rand_seed = lexical_cast<int>(argv[1]);
    arity = lexical_cast<int>(argv[2]);
    max_evals = lexical_cast<int>(argv[3]);
    init_temp = lexical_cast<double>(argv[4]);
    min_temp = lexical_cast<double>(argv[5]);
    temp_step_size = lexical_cast<double>(argv[6]);
    accept_prob_temp_intensity = lexical_cast<double>(argv[7]);
    dist_temp_intensity = lexical_cast<double>(argv[8]);
    fraction_of_remaining = lexical_cast<double>(argv[9]);

  } catch (...) {
    cerr << "usage: " << argv[0] 
         << " seed  arity  maxevals  init_temp  min_temp  temp_step_size" 
         << " accept_prob_temp_intensity(0~1) dist_temp_intensity(0~1) fraction_of_remaing" << endl;
    exit(1);
  }

  opencog::MT19937RandGen rng(rand_seed);

  disjunction scorer;

  type_tree tt(id::lambda_type);
  tt.append_children(tt.begin(),id::boolean_type,arity+1);

  metapopulation<logical_score, logical_bscore, simulated_annealing>
      metapop(rng, combo_tree(id::logical_and), tt, logical_reduction(), true,
              logical_score(scorer, arity, rng),
              logical_bscore(scorer, arity, rng), true,
              simulated_annealing(rng, init_temp, min_temp, temp_step_size,
                                  accept_prob_temp_intensity,
                                  dist_temp_intensity,
                                  fraction_of_remaining)
                                  );
  
  cout << "build metapop" << endl;

  moses::moses(metapop,max_evals,0);
}
