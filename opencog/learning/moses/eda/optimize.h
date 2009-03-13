#ifndef _EDA_OPTIMIZE_H
#define _EDA_OPTIMIZE_H

#include "eda/instance_set.h"
#include "util/foreach.h"

namespace eda {

  using namespace std;
  
  template
  <typename ScoreT,
   typename ScoringPolicy,
   typename TerminationPolicy,
   typename SelectionPolicy,
   typename StructureLearningPolicy,
   typename ProbsLearningPolicy,
   typename ReplacementPolicy,
   typename LoggingPolicy>
  int optimize(instance_set<ScoreT>& current,
	       int n_select,int n_generate,int max_gens,
	       const ScoringPolicy& score,
	       const TerminationPolicy& termination_criterion,
	       const SelectionPolicy& select,
	       const StructureLearningPolicy& learn_structure,
	       const ProbsLearningPolicy& learn_probs,
	       const ReplacementPolicy& replace,
	       LoggingPolicy& write_log, 
	       opencog::RandGen& rng) 
  {
    typedef typename StructureLearningPolicy::model_type model_type;

    cout << "top " << current.size() << endl;
    cout << "top " << distance(current.begin_scores(),
			       current.end_scores()) << endl;

    //compute scores of the initial instance set
    transform(current.begin(),current.end(),current.begin_scores(),score);
    cout << "topd" << endl;

    //main loop
    int gen=0;
    for (;gen<max_gens && !termination_criterion(current.begin(),
						 current.end());++gen) {

      cout << "PPtop" << endl;

      //do logging of instance_set and scores
      write_log(current.begin(),current.end(),current.fields(),gen);
      
      //select promising instances
      std::vector<scored_instance<ScoreT> > promising(n_select);
      select(current.begin(),current.end(),promising.begin(),n_select);

      cout << "PPmid" << endl;

      //initialize the model
      model_type model(current.fields(),promising.begin(),promising.end(), rng);

      //update the model
      learn_structure(current.fields(),promising.begin(),promising.end(),model);
      learn_probs(current.fields(),promising.begin(),promising.end(),model);

      //create new instances and integrate them into the current instance set,
      //replacing existing instances

      replace(begin_generator(bind(score_instance<ScoringPolicy>,
				   bind(model),score)),
	      end_generator(bind(score_instance<ScoringPolicy>,
				 bind(model),score),n_generate),
	      current.begin(),current.end());      

      cout << "PPbot" << endl;
    }

    //log the final result
    write_log(current.begin(),current.end(),current.fields(),gen);

    //return # of evaluations actually performed
    return current.size()+gen*n_generate;
  }

} //~namespace eda

#endif
