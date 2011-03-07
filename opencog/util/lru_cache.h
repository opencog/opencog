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
#include <boost/bind.hpp>

#include "hashing.h"
#include "exceptions.h"
#include "oc_assert.h"

namespace opencog {

/**
 * A set of generic caches. Please be careful if you add more caches
 * that they do not break when the function to cache raises an
 * exception.
 */

// Least Recently Used Cache
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
  
    lru_cache(size_type n,const F& f=F()) : _n(n), _map(n+1), _f(f) {}

    inline bool full() const { return _map.size()==_n; }
    inline bool empty() const { return _map.empty(); }

    //! Remove (aka make dirty) x from cache because entry invalid
    void remove(const argument_type& x) {
        _lru.push_front(x); // temporary so we can get an iterator for searching map
        map_iter it=_map.find(_lru.begin());
        if (it != _map.end()) {
            // remove existing entry
            _lru.erase(it->first);
            _map.erase(it);
        }
        // remove temporary
        _lru.pop_front();
    }

    result_type operator()(const argument_type& x) const
    {
        if (empty()) {
            if (full()) //so a size-0 cache never needs hashing
                return _f(x); // note that cache failures are not counted
            _lru.push_front(x);
            map_iter it=_map.insert(make_pair(_lru.begin(),call_f(x))).first;
            return it->second;
        }
      
        //search for it
        _lru.push_front(x);
        map_iter it=_map.find(_lru.begin());
      
        //if we've found it, update lru and return
        if (it!=_map.end()) {
            _lru.pop_front();
            _lru.splice(_lru.begin(),_lru,it->first);
            _hits++;
            return it->second;
        }
      
        //otherwise, call _f and do an insertion
        it=_map.insert(make_pair(_lru.begin(), call_f(x))).first;
      
        //if full, remove least-recently-used
        if (_map.size()>_n) {
            _map.erase(--_lru.end());
            _lru.pop_back();
        }
      
        OC_ASSERT(_map.size() <= _n,
                  "lru_cache - _map size greater than _n (%d).", _n);
        OC_ASSERT(_lru.size() == _map.size(),
                  "lru_cache - _lru size different from _map size.");
      
        //return the result
        return it->second;
    }

    void clear() {
        _map.clear();
        _lru.clear();
    }
    
protected:
    size_type _n;
    mutable map _map;
    F _f;
    mutable list _lru; // this list is only here so that we know what
                       // is the last used element to remove it from
                       // the cache when it gets full
    
    mutable unsigned _failures; // number of cache failures
    mutable unsigned _hits; // number of cache hits

    inline result_type call_f(const argument_type& x) const {
        _failures++;
        try {
            return _f(x);
        } catch(...) {
            _lru.pop_front(); // remove x, previously inserted in _lru
            throw;
        }
    }
};

// Pseudo Random Replacement Cache, very fast but very dumb, it just
// removes the first element of the hash table when the cache is full.
template<typename F,
         typename Hash=boost::hash<typename F::argument_type>,
         typename Equals=std::equal_to<typename F::argument_type> >
struct prr_cache {
    typedef typename F::argument_type argument_type;
    typedef typename F::result_type result_type;
    typedef boost::unordered_map<argument_type, result_type, Hash, Equals> map;
    typedef typename map::iterator map_iter;
    typedef typename map::size_type size_type;
  
    prr_cache(size_type n, const F& f=F()) 
        : _n(n), _map(n+1), _f(f), _failures(0) {}

    bool full() const { return _map.size()==_n; }
    bool empty() const { return _map.empty(); }

    result_type operator()(const argument_type& x) const
    {
        // search for x
        map_iter it=_map.find(x);

        if(it != _map.end()) {// if we've found return 
            _hits++;
            return it->second;
        }
        else { // otherwise evaluate, insert in _map then return
            result_type res = call_f(x);
            if(full()) { // if the cache is full randomly remove an element
                _map.erase(_map.begin());
            }
            _map.insert(make_pair(x, res)).first;
            return res;
        }
    }
    
    void clear() {
        _map.clear();
    }
    
    unsigned get_failures() const { return _failures; }
    unsigned get_hits() const { return _hits; }

protected:
    size_type _n;
    mutable map _map;
    F _f;
    mutable unsigned _failures;
    mutable unsigned _hits; // number of cache hits

    inline result_type call_f(const argument_type& x) const {
        _failures++;
        return _f(x);
    }

};
  
/// @todo this stuff sucks an should be removed. It is kept before
/// some code in embodiment uses it
// like above but hacked to handle a function that changes, it is
// embodiment, but all this is ugly and should be replaced by an
// elegent use of lru_cache
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
            _lru.splice(_lru.begin(),_lru,it->first);

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

} //~namespace opencog

#endif
