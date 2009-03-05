#ifndef _REDUCT_RULE_H
#define _REDUCT_RULE_H

#include <LADSUtil/RandGen.h>

#include "ComboReduct/reduct/using.h"
#include "ComboReduct/combo/vertex.h"

namespace reduct {

  using namespace combo;

  struct rule {
    virtual ~rule() { }
    virtual void operator()(combo_tree&,combo_tree::iterator) const=0;
    virtual rule* clone() const=0;

    void operator()(combo_tree& tr) const { 
      if (!tr.empty())
	(*this)(tr,tr.begin());
    }
  };
  reduct::rule* new_clone(const reduct::rule& r);

  template<typename T>
  struct crule : public rule {
    rule* clone() const { return new T(*((T*)this)); }
  };

  const rule& logical_reduction();
  const rule& contin_reduction(LADSUtil::RandGen& rng);
  const rule& mixed_reduction(LADSUtil::RandGen& rng);
  const rule& full_reduction(LADSUtil::RandGen& rng);
  const rule& action_reduction();
  const rule& perception_reduction();

  const rule& clean_reduction();
  //const rule& clean_and_full_reduction();

  inline void logical_reduce(combo_tree& tr,combo_tree::iterator it) {
    logical_reduction()(tr,it);
  }
  inline void logical_reduce(combo_tree& tr) { logical_reduction()(tr); }

  inline void contin_reduce(combo_tree& tr,combo_tree::iterator it, LADSUtil::RandGen& rng) {
    contin_reduction(rng)(tr,it);
  }
  inline void contin_reduce(combo_tree& tr, LADSUtil::RandGen& rng) { contin_reduction(rng)(tr); }

  inline void mixed_reduce(combo_tree& tr,combo_tree::iterator it, LADSUtil::RandGen& rng) {
    mixed_reduction(rng)(tr,it);
  }
  inline void mixed_reduce(combo_tree& tr, LADSUtil::RandGen& rng) { mixed_reduction(rng)(tr); }

  inline void full_reduce(combo_tree& tr,combo_tree::iterator it, LADSUtil::RandGen& rng) {
    full_reduction(rng)(tr,it);
  }
  inline void full_reduce(combo_tree& tr, LADSUtil::RandGen& rng) { full_reduction(rng)(tr); }

  inline void clean_reduce(combo_tree& tr,combo_tree::iterator it) {
    clean_reduction()(tr,it);
  }
  inline void clean_reduce(combo_tree& tr) { clean_reduction()(tr); }

  inline void clean_and_full_reduce(combo_tree& tr,combo_tree::iterator it, LADSUtil::RandGen& rng) {
    //clean_and_full_reduction()(tr,it);
    clean_reduce(tr,it);
    full_reduce(tr,it,rng);
  }
  inline void clean_and_full_reduce(combo_tree& tr, LADSUtil::RandGen& rng) { 
    //clean_and_full_reduction()(tr,tr.begin()); 
    clean_reduce(tr);
    full_reduce(tr,rng);
  }

  //action
  inline void action_reduce(combo_tree& tr, combo_tree::iterator it) {
    action_reduction()(tr,it);
  }

  inline void action_reduce(combo_tree& tr) { action_reduction()(tr); }

  //perception
  inline void perception_reduce(combo_tree& tr, combo_tree::iterator it) {
    perception_reduction()(tr,it);
  }

  inline void perception_reduce(combo_tree& tr) { perception_reduction()(tr); }
  
} //~namespace reduct

#endif
