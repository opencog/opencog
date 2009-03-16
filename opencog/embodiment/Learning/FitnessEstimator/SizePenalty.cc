/**
 * SizePenalty.cc
 *
 * Author(s):
 *   Nil Geisweiller
 * Creation: Mon Nov 12 2007
 */

#include "SizePenalty.h"
#include "util/exceptions.h"
#include "DistortedComboSize.h"
#include "util/Logger.h"
#include "SystemParameters.h"
#include "NetworkElement.h"

namespace FitnessEstimator {

  SizePenalty::SizePenalty(const std::set<combo::definite_object>& dos,
			   int indefinite_object_count, int operator_count,
			   int predicate_count, int action_count)
    : _dos(dos) {
    a = atof(MessagingSystem::NetworkElement::parameters.get("SIZE_PENALTY_COEF_A").c_str());
    b = atof(MessagingSystem::NetworkElement::parameters.get("SIZE_PENALTY_COEF_B").c_str());
    setc(indefinite_object_count, operator_count,
	 predicate_count, action_count);
  }

  SizePenalty::~SizePenalty() { }

  //Occam's razor factor
  //tends to 0 when the size of the combo tends to infinity
  //tends to 1 when the size of the combo tends to 1
  double SizePenalty::computeSizePenalty(const combo::combo_tree& tr) const {
    opencog::cassert(TRACE_INFO, !tr.empty(),
	    "SizePenalty - combo_tree should not be empty.");
    int s = DistortedComboSize::size(tr, _dos);

    //debug log for SPCTools
    MAIN_LOGGER.log(opencog::Logger::DEBUG, "SizePenalty - SPCTools - Combo size : %d", s);
    //~debug log for SPCTools

    return std::exp(-a*std::log(b*c+std::exp(1))*(double)s);
  }

  void SizePenalty::update(int indefinite_object_count, int operator_count,
			   int condition_count, int action_count) {
    setc(indefinite_object_count, operator_count,
	 condition_count, action_count);
  }

  void SizePenalty::setc(int indefinite_object_count, int operator_count,
			 int condition_count, int action_count) {
    int sum = (_dos.size() + indefinite_object_count
	       + operator_count + condition_count + action_count);
    opencog::cassert(TRACE_INFO, sum > 0,
	    "SizePenalty - sum should be greater than 0.");
    c = (double)(sum);
  }

}//~namespace FitnessEstimator
