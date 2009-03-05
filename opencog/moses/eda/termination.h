#ifndef _EDA_TERMINATION_H
#define _EDA_TERMINATION_H

#include <algorithm>
#include <limits>

namespace eda {

  template<typename ScoreT>
  struct terminate_if_gte {
    terminate_if_gte(const ScoreT& b) : bound(b) { }

    template<typename It>
    bool operator()(It from,It to) const {
      return (*std::max_element(from,to)>=bound);
    }
  protected:
    ScoreT bound;
  };

  template<typename ScoreT>
  struct terminate_if_gte_or_no_improv {
    terminate_if_gte_or_no_improv(const ScoreT& b,int n) : 
      bound(b),n_gen(n),at(-1) { }

    template<typename It>
    bool operator()(It from,It to) const {
      ScoreT res=std::max_element(from,to)->second;
      if (res>best_seen || at==-1) {
	best_seen=res;
	at=0;
	return (res>=bound);
      }
      return (at++>n_gen || best_seen>=bound);
    }
  protected:
    ScoreT bound;
    mutable ScoreT best_seen;
    int n_gen;
    mutable int at;
  };

} //~namespace eda

#endif
