#ifndef _COMBO_USING_H
#define _COMBO_USING_H

#include <boost/variant.hpp>
//#include <boost/bind.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/indirect_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/multi_array.hpp>

#include <functional>
#include <algorithm>

/// anything that gets imported into the combo namespace with a using
/// directive should go here
namespace combo {
  using boost::variant;
  using boost::static_visitor;
  //  using boost::bind;
  using boost::make_counting_iterator;
  using boost::make_indirect_iterator;
  using boost::make_transform_iterator;
  using boost::apply_visitor;
  using boost::assign::list_of;
  using boost::multi_array;
  using std::find_if;
  using std::accumulate;
}

#endif
