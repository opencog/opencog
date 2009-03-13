#ifndef _MOSES_USING_H
#define _MOSES_USING_H

#include <boost/logic/tribool.hpp>
#include <boost/bind.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/permutation_iterator.hpp>
#include <boost/iterator/indirect_iterator.hpp>

#include <set>
#include <functional>
#include <vector>

#include "comboreduct/combo/vertex.h"

// uncomment this line for debug information to be given during execution
//#define DEBUG_INFO 

namespace moses {

  using namespace combo;

  using std::min;
  using std::max;
  using std::accumulate;
  using std::set;
  using std::unary_function;
  using std::make_pair;
  using std::vector;
  using std::advance;

  using boost::logic::tribool;
  using boost::logic::indeterminate;
  using boost::bind;
  using boost::make_counting_iterator;
  using boost::make_transform_iterator;
  using boost::make_permutation_iterator;
  using boost::make_indirect_iterator;

} //~namespace moses

#endif
