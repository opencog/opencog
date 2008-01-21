#ifndef _UTIL_LAZY_RANDOM_SELECTOR_H
#define _UTIL_LAZY_RANDOM_SELECTOR_H

#include "hash_map.h"
#include "RandGen.h"

namespace Util {

  //a lazy random selector without replacement -
  //lets you select m random integers in [0,n) without replacement each in O(1)
  //and only uses O(m) memory - useful where n is much larger than m
  struct lazy_random_selector {
    lazy_random_selector(int n, Util::RandGen& _rng) : _n(n),_v(-1),rng(_rng) { }
    bool empty() const { return (_n==0); }
    int operator()();
  private:
    int _n;
    hash_map<int,int> _map;
    int _v;
    Util::RandGen& rng;
  };

} //~namespace Util

#endif
