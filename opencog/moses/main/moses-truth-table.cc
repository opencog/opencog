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




