#ifndef _EDA_SCORING_H
#define _EDA_SCORING_H

#include "MosesEda/eda/eda.h"
#include <LADSUtil/functional.h>

namespace eda {
  
  template<typename ScoreT>
  struct scored_instance : public LADSUtil::tagged_item<instance,ScoreT> {
    typedef LADSUtil::tagged_item<instance,ScoreT> super;

    scored_instance(const instance& i,const ScoreT& s) : super(i,s) { }
    scored_instance(const instance& i) : super(i) { }
    scored_instance() { }
    template<class T1, class T2>
    scored_instance(const std::pair<T1,T2>& p) : super(p) { }
  };

  template<typename Scoring>
  scored_instance<typename result_of<Scoring(instance)>::type> 
  score_instance(const instance& inst,const Scoring& score) {
    return 
      scored_instance<typename result_of<Scoring(instance)>::type>(inst,
								   score(inst));
  }

} //~namespace eda

#endif
