/*
 * opencog/util/lru_cache.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_LRU_CACHE_H
#define _OPENCOG_LRU_CACHE_H

#include <list>
#include <boost/unordered_map.hpp>
#include "hashing.h"
#include "exceptions.h"
#include "oc_assert.h"

//define this in case you want to count the number of evaluations
//that is when the function call isn't yet in the cache
#define COUNT_EVALUATION

namespace opencog {

  template<typename ARG, typename RESULT>
  struct lru_cache_arg_result {
    typedef ARG argument_type;
    typedef RESULT result_type;
    typedef typename boost::hash<argument_type> Hash;
    typedef typename std::equal_to<argument_type> Equals;
    typedef typename std::list<argument_type> list;
    typedef typename list::iterator list_iter;
    typedef boost::unordered_map<list_iter,result_type,
                              opencog::deref_hash<list_iter,Hash>,
                              opencog::deref_equals<list_iter,Equals> > map;
    typedef typename map::iterator map_iter;
    typedef typename map::size_type size_type;
  
    lru_cache_arg_result(size_type n) : _n(n), _map(n+1) { }

    inline bool full() const { return _map.size()==_n; }
    inline bool empty() const { return _map.empty(); }
    
    //that method not only returns the map_iter corresponding to a
    //input but also places x in front of the lru list if found
    map_iter find(const argument_type& x) {
      //search for it
      _lru.push_front(x);
      map_iter it=_map.find(_lru.begin());
      _lru.pop_front();

      //if we've found it, update lru
      if (it!=_map.end())
	_lru.splice(it->first,_lru,_lru.begin());

      return it;
    }

    inline bool is_cache_failure(map_iter mi) {
      return mi == _map.end();
    }

    //it is assumed that x is not in the cache
    void insert_new(const argument_type& x, const result_type& y) {
      _lru.push_front(x);
      _map.insert(make_pair(_lru.begin(),y)).first;
      if(full()) {
	_map.erase(--_lru.end());
        _lru.pop_back();	
      }
    }
    
    void clear() {
      _map.clear();
      _lru.clear();
    }
    
  protected:
    size_type _n;
    mutable map _map;
    mutable list _lru;
  };

  template<typename F,
	   typename Hash=boost::hash<typename F::argument_type>,
	   typename Equals=std::equal_to<typename F::argument_type> >
  struct lru_cache {
    typedef typename F::argument_type argument_type;
    typedef typename F::result_type result_type;
    typedef typename std::list<argument_type> list;
    typedef typename list::iterator list_iter;
    typedef boost::unordered_map<list_iter,result_type,
			       opencog::deref_hash<list_iter,Hash>,
			       opencog::deref_equals<list_iter,Equals> > map;
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
      
      OC_ASSERT(_map.size()<=_n, "lru_cache - _map size greater than _n (%d).", _n);
      OC_ASSERT(_lru.size()==_map.size(), "lru_cache - _lru size different from _map size.");
      
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
  
} //~namespace opencog

#endif
