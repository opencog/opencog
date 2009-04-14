/*
 * opencog/learning/moses/main/moses-parity.cc
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
  try {
    if (argc!=4)
      throw "foo";
    rand_seed=lexical_cast<int>(argv[1]);
    arity=lexical_cast<int>(argv[2]);
    max_evals=lexical_cast<int>(argv[3]);
  } catch (...) {
    cerr << "usage: " << argv[0] << " seed arity maxevals" << endl;
    exit(1);
  }

  opencog::MT19937RandGen rng(rand_seed);
 
  type_tree tt(id::lambda_type);
  tt.append_children(tt.begin(),id::boolean_type,arity+1);

  // scores for different propositional formulae
  even_parity scorer;
  // disjunction scorer;

/*
  metapopulation<logical_score,logical_bscore,univariate_optimization> 
    metapop(rng, combo_tree(id::logical_and),tt,logical_reduction(),
	    logical_score(scorer,arity,rng),
	    logical_bscore(scorer,arity,rng),
	    univariate_optimization(rng));
  //had to put namespace moses otherwise gcc-4.1 complains that it is ambiguous
  moses::moses(metapop,max_evals,0);
*/ 

  metapopulation<logical_score,logical_bscore,iterative_hillclimbing> 
    metapop(rng, combo_tree(id::logical_and),tt,logical_reduction(),
	    logical_score(scorer,arity,rng),
	    logical_bscore(scorer,arity,rng),
	    iterative_hillclimbing(rng));
  //had to put namespace moses otherwise gcc-4.1 complains that it is ambiguous
  moses::moses(metapop,max_evals,0);

}
