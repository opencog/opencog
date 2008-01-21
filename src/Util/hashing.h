#ifndef _UTIL_HASHING_H
#define _UTIL_HASHING_H

#include <boost/functional/hash.hpp>
#include "tree.h"

namespace Util {

  template<typename T,
	   typename Hash=boost::hash<T> >
  struct deref_hash {
    deref_hash(const Hash& h=Hash()) : hash(h) {} 
    size_t operator()(const T& t) const { return hash(*t); }
    Hash hash;
  };

  template<typename T,
	   typename Equals=std::equal_to<T> >
  struct deref_equals {
    deref_equals(const Equals& e=Equals()) : equals(e) {} 
    bool operator()(const T& x,const T& y) const { return equals(*x,*y); }
    Equals equals;
  };

  template<typename T>
  std::size_t hash_value(const tree<T>& tr) { 
    return boost::hash_range(tr.begin(),tr.end());
  }

} //~namespace Util

#endif
