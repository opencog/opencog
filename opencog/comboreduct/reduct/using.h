#ifndef _REDUCT_USING_H
#define _REDUCT_USING_H

#include <boost/variant.hpp>
//#include <boost/bind.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/indirect_iterator.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include "ComboReduct/combo/vertex.h"

#include <functional>
#include <algorithm>

/// anything that gets imported into the reduct namespace with a using
/// directive should go here
namespace reduct {
  using namespace combo;
  using boost::variant;
  using boost::static_visitor;
  //  using boost::bind;
  using boost::make_counting_iterator;
  using boost::make_indirect_iterator;
  using boost::apply_visitor;
  using boost::shared_ptr;
  using boost::ptr_vector;
  using std::find_if;
  using std::distance;
  using std::make_pair;  
}

#endif
