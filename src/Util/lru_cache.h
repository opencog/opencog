#ifndef _UTIL_LRU_CACHE_H
#define _UTIL_LRU_CACHE_H

#include <list>
#include <functional>
#include "hashing.h"
#include "hash_map.h"
#include "exceptions.h"

//define this in case you want to count the number of evaluations
//that is when the function call isn't yet in the cache
#define COUNT_EVALUATION

namespace Util {

  template<typename F,
	   typename Hash=boost::hash<typename F::argument_type>,
	   typename Equals=std::equal_to<typename F::argument_type> >
  struct lru_cache {
    typedef typename F::argument_type argument_type;
    typedef typename F::result_type result_type;
    typedef typename std::list<argument_type> list;
    typedef typename list::iterator list_iter;
    typedef Util::hash_map<list_iter,result_type,
			   Util::deref_hash<list_iter,Hash>,
			   Util::deref_equals<list_iter,Equals> > map;
    typedef typename map::iterator map_iter;
    typedef typename map::size_type size_type;
  
    lru_cache(size_type n,const F& f=F()) : _n(n),_map(n+1),_f(f)
	 ,_number_of_evaluations(0) { }

    bool full() const { return _map.size()==_n; }
    bool empty() const { return _map.empty(); }
    
    result_type operator()(const argument_type& x) {
      if (empty()) {
	if (full()) //so a size-0 cache never needs hashing
	return _f(x);
	_lru.push_front(x);
	map_iter it=_map.insert(make_pair(_lru.begin(),_f(x))).first;
	return it->second;
      }
      
      //search for it
      _lru.push_front(x);
      map_iter it=_map.find(_lru.begin());
      
      //if we've found it, update lru and return
      if (it!=_map.end()) {
	_lru.pop_front();
	_lru.splice(it->first,_lru,_lru.begin());
	return it->second;
      }
      
      //otherwise, call _f and do an insertion
#ifdef COUNT_EVALUATION
      _number_of_evaluations++;
#endif
      it=_map.insert(make_pair(_lru.begin(),_f(x))).first;
      
      //if full, remove least-recently-used
      if (_map.size()>_n) {
	_map.erase(--_lru.end());
        _lru.pop_back();
      }
      
      cassert(TRACE_INFO, _map.size()<=_n, "lru_cache - _map size greater than _n (%d).", _n);
      cassert(TRACE_INFO, _lru.size()==_map.size(), "lru_cache - _lru size different from _map size.");
      
      //return the result
      return it->second;
    }
    
    unsigned get_number_of_evaluations() { return _number_of_evaluations; }

    void clear() {
      _map.clear();
      _lru.clear();
    }
    
  protected:
    size_type _n;
    mutable map _map;
    F _f;
    mutable list _lru;
    
    mutable unsigned _number_of_evaluations;
  };
  
} //~namespace Util

#endif
