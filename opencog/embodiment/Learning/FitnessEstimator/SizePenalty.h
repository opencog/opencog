/**
 * SizePenalty.h
 *
 * Author(s): 
 *   Nil Geisweiller
 * Creation: Mon Nov 12 2007
 */
#ifndef _SIZEPENALTY_H
#define _SIZEPENALTY_H

#include "comboreduct/combo/vertex.h"

namespace FitnessEstimator {
  
  class SizePenalty {
    
  private:
    double a;
    double b;
    double c;

    const std::set<combo::definite_object>& _dos;

    void setc(int indefinite_object_count, int operator_count,
	      int condition_count, int action_count);
    
  public:
    
    SizePenalty(const std::set<combo::definite_object>& dos,
		int indefinite_object_count = 0, int operator_count = 0,
		int predicate_count = 0, int action_count = 0);
    ~SizePenalty();

    //Occam's razor factor
    //tends to 0 when the size of the combo tends to infinity
    //tends to 1 when the size of the combo tends to 1
    double computeSizePenalty(const combo::combo_tree& tr) const;

    void update(int definite_object_count, int operator_count,
		int condition_count, int action_count);

  };

}//~namespace FitnessEstimator

#endif
