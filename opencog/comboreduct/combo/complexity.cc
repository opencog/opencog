#include "util/exceptions.h"

#include "comboreduct/combo/complexity.h"
#include "comboreduct/combo/using.h"

namespace combo {

  // for a Boolean formula, the complexity is the # of literals

  complexity_t complexity(combo_tree::iterator it) {
    if (*it==id::logical_true || *it==id::logical_false || *it==id::null_vertex)
      return 0;

    if (is_argument(*it))
      return -1;

    if (is_builtin_action(*it)) // PJ
      return -1;

    if (*it==id::logical_not)
      return complexity(it.begin());

//    cassert(TRACE_INFO, *it==id::logical_and || *it==id::logical_or, 
//            "combo_tree node should be of logical types 'id::logical_and' or 'id::logical_or'.");
// PJ now complexity for action trees can also be computed

    int c=0;
    for (combo_tree::sibling_iterator sib=it.begin();sib!=it.end();++sib)
      c+=complexity(sib);
    return c;
  }

} //~namespace combo
