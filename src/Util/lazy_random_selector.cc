#include "lazy_random_selector.h"
#include "exceptions.h"
#include "numeric.h"
#include "RandGen.h"

namespace Util {

  int lazy_random_selector::operator()() {

    cassert(TRACE_INFO, !empty(), "lazy_random_selector - selector is empty.");
    int idx=rng.randint(_n--);

    hash_map<int,int>::iterator it=_map.find(idx);
    if (idx==_n) {
      if (it!=_map.end()) {
	idx=it->second;
	_map.erase(it);
      }
      return idx;
    }
    int res=(it==_map.end()) ? idx : it->second;
    hash_map<int,int>::iterator last=_map.find(_n);
    if (last==_map.end()) {
      _map.insert(std::make_pair(idx,_n));
    } else {
      _map.insert(std::make_pair(idx,last->second));
      _map.erase(last);
    }
    return res;
  }

} //~namespace Util
