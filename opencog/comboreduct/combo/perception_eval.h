#ifndef _COMBO_PERCEPTION_EVAL_H
#define _COMBO_PERCEPTION_EVAL_H

#include <exception>

#include "util/exception.h"
#include "util/tree.h"

#include "comboreduct/combo/vertex.h"
#include "comboreduct/combo/using.h"
#include "comboreduct/combo/type_tree.h"


namespace combo {

  template<typename It>
  vertex action_eval_throws(It it) throw(EvalException, std::bad_exception) {
    typedef typename It::sibling_iterator sib_it;
    //argument
    /*if(is_argument(*it)) {
      int idx=get_argument(*it).idx;
      //assumption : when idx is negative the argument is necessary boolean
      assert(idx>0);
      return binding(idx);
      }*/
    //perception
    if(*it==id::dummy_ultrametric) {
      return 0.0
    }
    else if(*it==id::dummy_transitive) {
      return id::logical_true;
    }
  }

}

#endif
