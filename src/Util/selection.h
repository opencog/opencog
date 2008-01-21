#ifndef _UTIL_SELECTION_H
#define _UTIL_SELECTION_H

#include "functional.h"
#include "numeric.h"
#include "hash_map.h"
#include <iterator>
#include "dorepeat.h"
#include "RandGen.h"

namespace Util {
  
  struct tournament_selection {
    tournament_selection(int t_size_, Util::RandGen& _rng) : t_size(t_size_), rng(_rng) { }
    int t_size;
    Util::RandGen& rng;

    template<typename In,typename Out>
    void operator()(In from,In to,Out dst,int n_select) const {
      typename std::iterator_traits<In>::difference_type d=distance(from,to);
      dorepeat(n_select) {
	In res=from+rng.randint(d);
	dorepeat(t_size-1) {
	  In tmp=from+rng.randint(d);
	  if (*res<*tmp)
	    res=tmp;
	}
	*dst++=*res;
      }
    }
  };

  template<typename It,typename ScoreT>
  It roulette_select(It from,It to,ScoreT sum, Util::RandGen& rng) {
    sum=ScoreT(double(sum) * rng.randdouble());
    do {
      sum-=*from++;
    } while (sum>=0);
    return --from;
  }
  template<typename It>
  It roulette_select(It from,It to, Util::RandGen& rng) {
    typedef typename std::iterator_traits<It>::value_type score_type;
    return roulette_select(from,to,std::accumulate(from,to,score_type(0)),rng);
  }

  template<typename NodeT>
  class NodeSelector {
  public:
    typedef NodeT value_type;
    typedef std::vector<std::pair<NodeT,int> > PSeq;
  
    NodeSelector(Util::RandGen& _rng) : rng(_rng) {
    }

    NodeT select(int arity) const {
      return roulette_select
	(boost::make_transform_iterator
	 (_byArity[arity].begin(),Util::select2nd<typename PSeq::value_type>()),
	 boost::make_transform_iterator
	 (_byArity[arity].end(),Util::select2nd<typename PSeq::value_type>()),
	 _aritySums[arity], rng).base()->first;
    }
    int select_arity(int from) const {
      //could make this slightly faster by caching the partial sums,
      //but who cares?
      return distance(_aritySums.begin(),
        	      roulette_select(_aritySums.begin()+from,_aritySums.end(),rng));
    }

    void add(const NodeT& n,int arity,int prob) {
      if ((int)_byArity.size()<=arity) {
	_byArity.resize(arity+1);
	_aritySums.resize(arity+1,0);
      }
      _byArity[arity].push_back(make_pair(n,prob));
      _aritySums[arity]+=prob;
    }
  private: 
    Util::RandGen& rng;
    std::vector<PSeq> _byArity;
    std::vector<int> _aritySums;
  };
  
} //~namespace Util

#endif
