#ifndef _EDA_USING_H
#define _EDA_USING_H

#include <boost/bind.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/permutation_iterator.hpp>
#include <boost/iterator/indirect_iterator.hpp>
#include <boost/utility/result_of.hpp>
#include <boost/next_prior.hpp>
#include <boost/type_traits.hpp>

#include <vector>
#include <algorithm>
#include <utility>

#include "util/functional.h"
#include "util/numeric.h"

/// anything that gets imported into the eda namespace with a using
/// directive should go here
namespace eda {
  using boost::bind;
  using boost::ref;
  using boost::make_counting_iterator;
  using boost::make_transform_iterator;
  using boost::make_permutation_iterator;
  using boost::make_indirect_iterator;
  using boost::result_of;
  using boost::next;
  using boost::prior;

  using opencog::begin_generator;
  using opencog::end_generator;
  using opencog::make_transform_output_iterator;

  using opencog::integer_log2;

  using opencog::nullary_function;  
  using std::unary_function;
  using std::binary_function;

  using std::vector;

  using std::distance;
  using std::copy;
  using std::transform;
  using std::nth_element;
  using std::accumulate;
  using std::adjacent_find;

  using std::pair;
  using std::make_pair;

  using opencog::select1st;
  using opencog::select2nd;
}

#endif
