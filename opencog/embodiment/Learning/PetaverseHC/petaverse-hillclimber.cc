#include "petaverse-hillclimber.h"
#include "RewritingRules.h"

namespace hillclimbing {

  //constructor, generate/filter conditions and actions
  petaverse_hillclimber::petaverse_hillclimber(int nepc,
					       const FE& fitness_estimator,
					       const definite_object_set& dos,
					       const operator_set& eo,
					       const combo_tree_ns_set& conditions,
					       const combo_tree_ns_set& actions,
                                               bool abibb,
                                               bool neic,
					       opencog::RandGen& rng) 
    : _comp(dos),
      _elementary_operators(eo), _conditions(conditions),
      _actions(actions), _rng(rng), 
      _hillclimber(fitness_estimator, nepc, _elementary_operators,
		   _conditions, _actions, _comp,
                   hillclimbing_action_reduction(),
                   hillclimbing_full_reduction(),
                   abibb, neic) {
    
    //right after run the operator once to have already a learned candidate
    _hillclimber();
  }

  petaverse_hillclimber::~petaverse_hillclimber() { }

  void petaverse_hillclimber::operator()() {
    _hillclimber();
  }
  
  const combo_tree& petaverse_hillclimber::best_program() {
    _best_program = _hillclimber.best_program();
    reduct::post_learning_rewrite(_best_program);
    return _best_program;
  }

  const combo_tree& petaverse_hillclimber::best_program_estimated() {
    _best_program_estimated = _hillclimber.best_program_estimated();
    reduct::post_learning_rewrite(_best_program_estimated);
    return _best_program_estimated;
  }

  const combo_tree& petaverse_hillclimber::current_program() {
    _current_program = _hillclimber.current_program(_rng);
    reduct::post_learning_rewrite(_current_program);
    return _current_program;      
  }

  void petaverse_hillclimber::set_current_fitness(fitness_t f) {
    _hillclimber.set_current_fitness(f);
  }
    
  void petaverse_hillclimber::reset_estimator() {            
    //reset hillclimber
    _hillclimber.reset_estimator();
  }
    
}//~namespace hillclimbing
