#ifndef _HILLCLIMBING_PETAVERSE_HILLCLIMBER_H
#define _HILLCLIMBING_PETAVERSE_HILLCLIMBER_H

#include "hillclimber.h"
#include "DistortedComboSize.h"
#include "PetaverseImitationLearning.h"

namespace hillclimbing {

  class petaverse_hillclimber : public PetaverseImitationLearningBase {
    
    typedef FitnessEstimator::NoSpaceLifeFitnessEstimator FE;
    typedef FitnessEstimator::DistortedComboSizeOrder Comp;

  public:

    //- abibb stands for action_boolean_if_both_branches and is true
    //  the neighborhood generates both branches of a conditional
    //- neic stands for new_exemplar_initializes_center
    //  and if is true then everytime a new exemplar comes
    //  the center is initialized with the empty program instead of the best one    
    petaverse_hillclimber(int nepc,
			  const FE& fitness_estimator,
			  const definite_object_set& dos,
			  const operator_set& eo,
			  const combo_tree_ns_set& conditions,
			  const combo_tree_ns_set& actions,
                          bool abibb,
                          bool neic,
			  opencog::RandGen& rng);

    ~petaverse_hillclimber();

    void operator()();
    
    const combo_tree& best_program();

    const combo_tree& best_program_estimated();

    const combo_tree& current_program();

    void set_current_fitness(fitness_t f);
    
    void reset_estimator();
    
  private:
    Comp _comp;

    const std::set<vertex>& _elementary_operators;
    const combo_tree_ns_set& _conditions;
    const combo_tree_ns_set& _actions;

    opencog::RandGen& _rng;

    hillclimber<FE, Comp> _hillclimber;

    combo_tree _best_program;
    combo_tree _best_program_estimated;
    combo_tree _current_program;

  };

}//~namespace hillclimbing


#endif
