#include "moses/moses.h"
#include "moses/optimization.h"
#include "moses/scoring_functions.h"
#include <boost/lexical_cast.hpp>
#include <comboreduct/reduct/reduct.h>
#include <iostream>
#include "util/mt19937ar.h"

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
