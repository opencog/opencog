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
#include <limits>

#include <boost/unordered_map.hpp>
#include <boost/thread.hpp>
#include <boost/numeric/conversion/cast.hpp>

#include "hashing.h"
#include "exceptions.h"
#include "oc_assert.h"
#include "platform.h"

namespace opencog {

/**
 * A set of generic caches. Please be careful if you add more caches
 * that they do not break when the function to cache raises an
 * exception.
 */

// Least Recently Used Cache. Non thread safe, use
// lru_cache_threaded for that.
template<typename F,
         typename Hash=boost::hash<typename F::argument_type>,
         typename Equals=std::equal_to<typename F::argument_type> >
struct lru_cache {
    typedef typename F::argument_type argument_type;
    typedef typename F::result_type result_type;
    typedef typename std::list<argument_type> list;
    typedef typename list::iterator list_iter;
    typedef boost::unordered_map<list_iter,result_type,
                                 deref_hash<list_iter,Hash>,
                                 deref_equals<list_iter,Equals> > map;
    typedef typename map::iterator map_iter;
    typedef typename map::size_type size_type;

    lru_cache(size_type n, const F& f=F())
        : _n(n), _map(n+1), _f(f), _failures(0), _hits(0) {}

    inline bool full() const { return _map.size()==_n; }
    inline bool empty() const { return _map.empty(); }
    inline unsigned max_size() const { return _n; }

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

    result_type operator()(const argument_type& x) const {
        if (empty()) {
            if (full()) //so a size-0 cache never needs hashing
                return if_f(x);
            _lru.push_front(x);
            map_iter it = _map.insert(make_pair(_lru.begin(), ifx_f(x))).first;
            return it->second;
        }
      
        //search for it
        _lru.push_front(x);
        map_iter it=_map.find(_lru.begin());
      
        //if we've found it, update lru and return
        if (it!=_map.end()) {
            _lru.pop_front();
            _lru.splice(_lru.begin(), _lru,it->first);
            _hits++;
            return it->second;
        }
      
        //otherwise, call _f and do an insertion
        it = _map.insert(make_pair(_lru.begin(), ifx_f(x))).first;
      
        //if full, remove least-recently-used
        if (_map.size() > _n) {
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

    unsigned get_failures() const { return _failures; }
    unsigned get_hits() const { return _hits; }

    void resize(unsigned n) {
        _n = n;
        while(_map.size() > _n) {
            _lru.begin();
            map_iter it = _map.find(_lru.begin());
            OC_ASSERT(it != _map.end(),
                      "Element in _lru has no corresponding iterator in _map");
            _lru.erase(it->first);
            _map.erase(it);
        }
        OC_ASSERT(_lru.size() == _map.size(),
                  "lru_cache - _lru size different from _map size.");
    }
    
protected:
    size_type _n;
    mutable map _map;
    const F& _f;
    mutable list _lru; // this list is only here so that we know what
                       // is the last used element to remove it from
                       // the cache when it gets full
    
    mutable unsigned _failures; // number of cache failures
    mutable unsigned _hits; // number of cache hits

    // increment failure and call
    inline result_type if_f(const argument_type& x) const {
        ++_failures;
        return _f(x);
    }

    // increment failure and exception safe call
    inline result_type ifx_f(const argument_type& x) const {
        ++_failures;
        return x_f(x);
    }

    // exception safe call
    inline result_type x_f(const argument_type& x) const {
        try {
            return _f(x);
        } catch(...) {
            _lru.pop_front(); // remove x, previously inserted in _lru
            throw;
        }
    }
};

// Least Recently Used Cache with thread safety
template<typename F,
         typename Hash=boost::hash<typename F::argument_type>,
         typename Equals=std::equal_to<typename F::argument_type> >
struct lru_cache_threaded : public lru_cache<F, Hash, Equals> {
private:
    typedef lru_cache<F, Hash, Equals> super;
    typedef boost::shared_mutex cache_mutex;
    typedef boost::shared_lock<cache_mutex> shared_lock;
    typedef boost::unique_lock<cache_mutex> unique_lock;
public:
    typedef typename F::argument_type argument_type;
    typedef typename F::result_type result_type;
    typedef typename super::list list;
    typedef typename list::iterator list_iter;
    typedef typename super::map map;
    typedef typename map::iterator map_iter;
    typedef typename map::size_type size_type;
  
    lru_cache_threaded(size_type n, const F& f=F()) : super(n, f) {}

    inline bool full() const {
        shared_lock lock(mutex);
        return super::full();
    }
    inline bool empty() const {
        shared_lock lock(mutex);
        return super::empty();
    }
    inline unsigned max_size() const {
        shared_lock lock(mutex);
        return super::max_size();
    }

    //! Remove (aka make dirty) x from cache because entry invalid
    void remove(const argument_type& x) {
        unique_lock lock(mutex);
        super::remove(x);
    }

    /// @todo buggy thread safe operator
    // result_type operator()(const argument_type& x) const {
    //     if (super::empty()) {
    //         if (super::full()) {
    //             // a size-0 cache never needs hashing
    //             return if_f(x);
    //         }
    //         lru_push_front(x);
    //         result_type r = ifx_f(x);
            
    //         unique_lock lock(mutex);
    //         map_iter it =
    //             super::_map.insert(make_pair(super::_lru.begin(),r)).first;
    //         return it->second;
    //     }
      
    //     //search for it
    //     super::_lru.push_front(x);
    //     map_iter it = super::_map.find(super::_lru.begin());
    //     super::_lru.pop_front();
      
    //     //if we've found it, update lru and return
    //     if (it != super::_map.end()) {
    //         super::_lru.splice(super::_lru.begin(), super::_lru, it->first);
    //         super::_hits++;
    //         return it->second;
    //     }
      
    //     //otherwise, call _f and do an insertion
    //     // briefly free lock for external call
    //     // lock.unlock();
    //     result_type r = ifail_f(x);
    //     // lock.lock();
    //     //--
    //     super::_lru.push_front(x);
    //     it = super::_map.insert(make_pair(super::_lru.begin(), r)).first;
      
    //     //if full, remove least-recently-used
    //     if (super::_map.size()>super::_n) {
    //         super::_map.erase(--super::_lru.end());
    //         super::_lru.pop_back();
    //     }
      
    //     OC_ASSERT(super::_map.size() <= super::_n,
    //               "lru_cache - _map size greater than _n (%d).", super::_n);
    //     OC_ASSERT(super::_lru.size() == super::_map.size(),
    //               "lru_cache - _lru size different from _map size.");
      
    //     //return the result
    //     return it->second;
    // }
    /// @todo REPLACE THAT FAULTY CODE
    result_type operator()(const argument_type& x) const {
        unique_lock lock(mutex);
        return super::operator()(x);
    }

    void clear() {
        unique_lock lock(mutex);
        super::clear();
    }
    
protected:
    mutable cache_mutex mutex;

    // thread safe push_front
    inline void lru_push_front(const argument_type& x) const {
        unique_lock lock(mutex);
        super::_lru.push_front(x);
    }

    // thread safe increment failure
    inline void inc_failure() const {
        unique_lock lock(mutex);
        super::_failure++;
    }

    // increment failure and call
    inline result_type if_f(const argument_type& x) const {
        inc_failure();
        return super::_f(x);
    }

    // increment failure and exception safe call
    inline result_type ifx_f(const argument_type& x) const {
        inc_failure();
        return xs_f(x);
    }

    // expection safe call
    inline result_type x_f(const argument_type& x) const {
        try {
            return _f(x);
        } catch(...) {
            unique_lock lock(mutex);
            super::_lru.pop_front(); // remove x, previously inserted in _lru
            throw;
        }
    }

};
 
// Pseudo Random Replacement Cache, very fast, but very dumb, it just
// removes the first element of the hash table when the cache is
// full. No thread safety, use prr_cache_threaded for that.
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
        : _n(n), _map(n+1), _f(f), _failures(0), _hits(0) {}

    bool full() const { return _map.size()==_n; }
    bool empty() const { return _map.empty(); }

    unsigned max_size() const { return _n; }

    result_type operator()(const argument_type& x) const
    {        
        // search for x
        map_iter it = _map.find(x);

        if(it != _map.end()) { // if we've found return 
            _hits++;
            return it->second;
        }
        else { // otherwise evaluate, insert in _map then return
            result_type res = if_f(x);
            if(full()) { // if the cache is full randomly remove an element
                _map.erase(_map.begin());
            }
            _map[x] = res;
            return res;
        }
    }

    void resize(unsigned n) {
        _n = n;
        while(_map.size() > _n)
            _map.erase(_map.begin());
    }
    
    void clear() {
        _map.clear();
    }
    
    unsigned get_failures() const { return _failures; }
    unsigned get_hits() const { return _hits; }

protected:
    size_type _n;
    mutable map _map;
    const F &_f;
    mutable unsigned _failures;
    mutable unsigned _hits; // number of cache hits

    // increment failures and call
    inline result_type if_f(const argument_type& x) const {
        _failures++;
        return _f(x);
    }

};

// Pseudo Random Replacement Cache with thread safety
template<typename F,
         typename Hash=boost::hash<typename F::argument_type>,
         typename Equals=std::equal_to<typename F::argument_type> >
struct prr_cache_threaded : public prr_cache<F, Hash, Equals> {
private:
    typedef prr_cache<F, Hash, Equals> super;
    typedef boost::shared_mutex cache_mutex;
    typedef boost::shared_lock<cache_mutex> shared_lock;
    typedef boost::unique_lock<cache_mutex> unique_lock;

public:
    typedef typename F::argument_type argument_type;
    typedef typename F::result_type result_type;
    typedef typename super::map map;
    typedef typename map::iterator map_iter;
    typedef typename map::size_type size_type;
  
    prr_cache_threaded(size_type n, const F& f=F()) : super(n, f) {}

    bool full() const {
        shared_lock lock(mutex);
        return super::full();
    }
    bool empty() const {
        shared_lock lock(mutex);
        return super::empty();
    }

    unsigned max_size() const {
        shared_lock lock(mutex);
        return super::max_size();
    }

    result_type operator()(const argument_type& x) const
    {
        {
            // for some unknown reason using shared_lock produces
            // occasional run-time error
            // shared_lock lock(mutex);
            unique_lock lock(mutex);
            // search for x
            map_iter it = super::_map.find(x);
            if(it != super::_map.end()) { // if we've found return 
                super::_hits++;
                return it->second;
            }
        }
        // otherwise evaluate, insert in _map then return
        result_type res = incfail_f(x);
        if(full()) { // if the cache is full randomly remove an element
            unique_lock lock(mutex);
            super::_map.erase(super::_map.begin());
        }
        unique_lock lock(mutex);
        super::_map[x] = res;
        return res;
    }
    
    void resize(unsigned n) {
        unique_lock lock(mutex);
        super::resize(n);
    }

    void clear() {
        unique_lock lock(mutex);
        super::clear();
    }
    
    unsigned get_failures() const {
        shared_lock lock(mutex);
        return super::get_failures();
    }

    unsigned get_hits() const {
        shared_lock lock(mutex);
        return super::get_hits();
    }

protected:
    mutable cache_mutex mutex;

    inline void inc_failures() const {
        unique_lock lock(mutex);
        ++super::_failures;
    }

    // increment failures and call
    inline result_type incfail_f(const argument_type& x) const {
        inc_failures();
        return super::_f(x);
    }

};

/// Cache adjusting automatically its size to avoid running out of RAM
/// or not using enough of the available RAM. The adjustment is done
/// every n calls (as provided in the constructor).
template<typename Cache>
struct adaptive_cache {
    typedef typename Cache::result_type result_type;
    typedef typename Cache::argument_type argument_type;
    
    /// If 1 - (free memory / total memory) > ulimit then the cache size is
    /// divided by ufrac. If used 1 - (free memory / total memory) < llimit then
    /// the cache size is multiplied by lfact.
    /// Try not to set ulimit above 90%, as otherwise, the Linux kernel
    /// obligingly tries to swap everything out to disk (see vm.swappiness
    /// setting & LKML discussions w/ AKPM)
    adaptive_cache(Cache& cache, unsigned ncycles = 1000,
                   float llimit = 0.75, float lfact = 2,
                   float ulimit = 0.90, float ufrac = 2)
        : _cache(cache), _counter(0), _ncycles(ncycles),
          _llimit(llimit), _lfact(lfact),
          _ulimit(ulimit), _ufrac(ufrac) {}

    result_type operator()(const argument_type& x) const {
        using boost::numeric_cast;
        using boost::numeric::positive_overflow;
        using std::numeric_limits;

        if(_counter++ % _ncycles == 0) {
            float tram = getTotalRAM();
            float fram = getFreeRAM();
            float free_mem_ratio = 1 - fram/tram;
            if(free_mem_ratio < _llimit && _cache.full()) {
                try {
                    _cache.resize(numeric_cast<unsigned>(_cache.max_size()*_lfact));
                } catch(positive_overflow&) {
                    _cache.resize(numeric_limits<unsigned>::max());
                }
            }
            else if(free_mem_ratio > _ulimit) {
                _cache.resize(std::max(1U, (unsigned)(_cache.max_size()/_ufrac)));
            }
        }
        return _cache(x);
    }

    unsigned get_failures() const { return _cache.get_failures(); }
    unsigned get_hits() const { return _cache.get_hits(); }

private:
    Cache& _cache;

    mutable unsigned _counter; // call counter, if it eventually wraps around
                               // it's no big deal

    unsigned _ncycles;
    float _llimit;
    float _lfact;
    float _ulimit;
    float _ufrac;
};


/// @todo this stuff sucks an should be removed. It is kept because
/// some code in embodiment still uses it

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
                                   deref_hash<list_iter,Hash>,
                                   deref_equals<list_iter,Equals> > map;
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
