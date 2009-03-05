#include "ComboReduct/combo/type_tree_def.h"

namespace combo {

  bool is_argument_type(type_node n) {
    return (int)n>=id::argument_type;
  }
  unsigned int arg_to_idx(type_node n) {
    LADSUtil::cassert(TRACE_INFO, is_argument_type(n),
		      "Cannot find the idx of a non-argument type");
    return (unsigned int)((int)n-(int)id::argument_type+1);
  }
}
