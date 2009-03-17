#ifndef _HILLCLIMBING_PETAVERSE_HILLCLIMBER_H
#define _HILLCLIMBING_PETAVERSE_HILLCLIMBER_H

#include "hillclimber.h"
#include "comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h"
#include "comboreduct/reduct/reduct.h"

namespace hillclimbing {

  using namespace ant_combo;

  template<typename FitnessEstimator>
  struct ant_hillclimber {
    ant_hillclimber(const FitnessEstimator& fe, int fepc) {
      //enumerate operators used in hillclimbing
      _elementary_operators.insert(id::sequential_and);
      _elementary_operators.insert(id::action_boolean_if);
      //enumerate actions used in hillclimbing
      combo_tree a1(instance(id::turn_left));
      combo_tree a2(instance(id::turn_right));
      combo_tree a3(instance(id::move_forward));
      _actions.insert(a1);
      _actions.insert(a2);
      _actions.insert(a3);
      //enumarate intuitions used in hillclimbing
      combo_tree p(instance(id::is_food_ahead));
      _perceptions.insert(p);
      
      _hillclimber = new hillclimber<FitnessEstimator>
          (fe, fepc, _elementary_operators, _perceptions, _actions, _comp,
           action_reduction(), action_reduction(),
           false, false);
    }

    ~ant_hillclimber() {
      delete(_hillclimber);
    }

    void operator()() {
      (*_hillclimber)();
    }

    fitness_t best_fitness() const { return _hillclimber->best_fitness(); }
    const combo_tree& best_program() const { return _hillclimber->best_program(); }

    fitness_t best_fitness_estimated() const {
      return _hillclimber->best_fitness_estimated(); }
    const combo_tree& best_program_estimated() const { 
      return _hillclimber->best_program_estimated(); }

    fitness_t current_fitness() const {
      return _hillclimber->current_fitness(); }
    void set_current_fitness(fitness_t f) {
      _hillclimber->set_current_fitness(f); }
    const combo_tree& current_program(RandGen& rng) {
      return _hillclimber->current_program(rng); }

    void reset_cache() {
      _hillclimber->reset_cache();
    }

  private:
    size_tree_order<vertex> _comp;
    hillclimber<FitnessEstimator>* _hillclimber;
    std::set<vertex> _elementary_operators;
    combo_tree_ns_set _actions;
    combo_tree_ns_set _perceptions;
  };

}//~namespace hillclimbing


#endif
