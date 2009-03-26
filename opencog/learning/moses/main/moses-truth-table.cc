/*
 * opencog/learning/moses/main/moses-truth-table.cc
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
#include "moses/moses.h"
#include "moses/optimization.h"
#include "moses/scoring_functions.h"

#include "util/mt19937ar.h"

using namespace moses;
using namespace reduct;
using namespace boost;

int main(int argc,char** argv) { 
  int arity,max_evals,rand_seed;
  try {
    if (argc!=4)
      throw "foo";
    rand_seed=lexical_cast<int>(argv[1]);
    max_evals=lexical_cast<int>(argv[2]);
  } catch (...) {
    cerr << "usage: " << argv[0] << " seed maxevals file_with_cases" << endl;
    exit(1);
  }

  opencog::MT19937RandGen rng(rand_seed);

  ifstream in(argv[3]);
  CaseBasedBoolean bc(in);
  arity=bc.arity();

  type_tree tt(id::application_type);
  tt.append_children(tt.begin(),id::boolean_type,arity+1);

  //even_parity scorer;
  //disjunction scorer;

  passenger_data_score  scorer(bc);
  passenger_data_bscore bscorer(bc);

  metapopulation<passenger_data_score,passenger_data_bscore,iterative_hillclimbing> 
    metapop(rng, combo_tree(id::logical_and),tt,logical_reduction(),
	    scorer,
	    bscorer,
	    iterative_hillclimbing(rng));

  moses::moses(metapop,max_evals,0);
}




