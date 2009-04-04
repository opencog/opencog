#include <iostream>

#include <boost/lexical_cast.hpp>

#include "comboreduct/reduct/reduct.h"

#include "util/mt19937ar.h"
#include "util/Logger.h"

#include "moses/moses.h"
#include "moses/optimization.h"
#include "moses/scoring_functions.h"


using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;

int main(int argc,char** argv) { 
  //set flag to print only cassert and other ERROR level logs on stdout
  opencog::logger().setPrintErrorLevelStdout();

  int order,max_evals,rand_seed;
  int arity=1,nsamples=20;
  try {
    if (argc!=4)
      throw "foo";
    rand_seed=lexical_cast<int>(argv[1]);
    order=lexical_cast<int>(argv[2]);
    max_evals=lexical_cast<int>(argv[3]);
  } catch (...) {
    cerr << "usage: " << argv[0] << " seed order maxevals" << endl;
    exit(1);
  }

  opencog::MT19937RandGen rng(rand_seed);

  type_tree tt(id::lambda_type);
  tt.append_children(tt.begin(),id::contin_type,2);

  RndNumTable rands(nsamples,arity,rng);

  cout << rands << endl;

  metapopulation<contin_score,contin_bscore,univariate_optimization> 
    metapop(rng, combo_tree(id::plus),
	    tt,contin_reduction(rng),
	    contin_score(simple_symbolic_regression(order),rands, rng),
	    contin_bscore(simple_symbolic_regression(order),rands, rng),
	    univariate_optimization(rng));
  
  //had to put namespace moses otherwise gcc-4.1 complains that it is ambiguous
  moses::moses(metapop,max_evals,0);
}
