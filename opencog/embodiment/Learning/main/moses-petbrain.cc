#include "moses/moses/moses.h"
#include "moses/moses/optimization.h"
#include "moses/moses/scoring_functions.h"
#include <boost/lexical_cast.hpp>
#include <comboreduct/reduct/reduct.h>
#include <iostream>
#include "util/mt19937ar.h"
#include "comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;
using namespace ant_combo;

int main(int argc,char** argv) { 
  int max_evals,rand_seed;
  try {
    if (argc!=3)
      throw "foo";
    rand_seed = lexical_cast<int>(argv[1]);
    max_evals=lexical_cast<int>(argv[2]);
  } catch (...) {
    cerr << "usage: " << argv[0] << " seed maxevals" << endl;
    exit(1);
  }

  opencog::MT19937RandGen rng(rand_seed);

  type_tree tt(id::lambda_type);
  tt.append_children(tt.begin(),id::action_result_type,1);

  interactive_score scorer;
  interactive_bscore bscorer;

  metapopulation<interactive_score,interactive_bscore,iterative_hillclimbing> 
    metapop(rng,combo_tree(id::sequential_and),tt,action_reduction(),
	    scorer,
	    bscorer,
            iterative_hillclimbing(rng));
  
  cout << "build metapop" << endl;



  operator_set os;
  combo_tree_ns_set perceptions;
  combo_tree_ns_set actions;

  actions.insert(combo_tree(instance(id::turn_left)));
  actions.insert(combo_tree(instance(id::turn_right)));
  actions.insert(combo_tree(instance(id::move_forward)));

  perceptions.insert(combo_tree(instance(id::is_food_ahead)));

  ordered_programs op;

  //had to put namespace moses otherwise gcc-4.1 complains that it is ambiguous
  moses::moses(metapop,max_evals,0,&os,&perceptions,&actions,op);
}
