#ifndef _REDUCT_GENERAL_RULES_H
#define _REDUCT_GENERAL_RULES_H

#include <LADSUtil/RandGen.h>

#include "ComboReduct/reduct/reduct.h"
#include "ComboReduct/combo/eval.h"

namespace reduct {

  //flattens all associative functions: f(a,f(b,c)) -> f(a,b,c)
  //note that level is recursive that is f(a,f(b,f(c,d))) -> f(a,b,c,d)
  struct level : public crule<level> { 
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //evaluates sub-expressions when possible
  //if an operator is commutative, op(const,var,const) will become
  //op(op(const,const),var), e.g., +(2,x,1)->+(3,x)
  struct eval_constants : public crule<eval_constants> { 
    LADSUtil::RandGen& rng;
    Evaluator* evaluator;
    eval_constants(LADSUtil::RandGen& _rng, Evaluator* e = NULL)
      : rng(_rng), evaluator(e) {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //Reorder children of commutative operators (should be applied upwards)
  struct reorder_commutative : public crule<reorder_commutative> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //Get rid of subtrees marked with a null_vertex in their roots
  struct remove_null_vertices : public crule<remove_null_vertices> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //Remaove all assumptions
  struct remove_all_assumptions : public crule<remove_null_vertices> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

} //~namespace reduct

#endif
