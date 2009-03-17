#include "interactive-hillclimber.h"
#include "comboreduct/combo/vertex.h"
#include "util/lru_cache.h"
#include <set>
#include <string>
#include <algorithm>
#include <iostream>
#include "util/mt19937ar.h"
#include "comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace hillclimbing;
using namespace ant_combo;
using namespace std;

struct InteractiveFitnessFunction : unary_function<combo_tree, fitness_t> {
  result_type operator()(argument_type tr) const
  { 
    cout << "Fitness Function of : " << tr << " enter the score :" << endl;
    fitness_t score = 0.0;
    cin >> score;
    return score;
  }
};

struct InteractiveFitnessEstimator : unary_function<combo_tree, fitness_t> {
  result_type operator()(argument_type tr) const
  { 
    cout << "Fitness Estimation of : " << tr << " enter the score :" << endl;
    fitness_t score = 0.0;
    cin >> score;
    return score;
  }
};

int main(int argc, char** argv) {

  int nepc = 1000;
  
  if(argc > 1)
    nepc = atoi(argv[1]);
 
  MT19937RandGen rng(0);

  InteractiveFitnessFunction iff;
  InteractiveFitnessEstimator ife;
  
  interactive_hillclimber<InteractiveFitnessEstimator> hc(ife, nepc);

  int i = 0;

  while(true) {
    cout << "HillClimbing Iteration " << i << " :" << endl;
    for(int j = 0; j < nepc; j++) {
      std::cout << "CYCLE : " << j << std::endl;
      hc();
    }
    hc.set_current_fitness(iff(hc.current_program(rng)));
    hc();
    cout << "************************************************" << endl;
    cout << "* Best program (gotten from the previous iteration) : "
	 << hc.best_program() << " with score : " << hc.best_fitness() << endl;
    cout << "* Best program estimated : " << hc.best_program_estimated() <<
      " with score : " << hc.best_fitness_estimated() << endl;
    cout << "* Current program : " << hc.current_program(rng) <<
      " with score : " << hc.current_fitness() << endl;
    cout << "************************************************" << endl;
    i++;
  }

}
