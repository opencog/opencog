#include "moses/types.h"

namespace moses {

  const tree_score worst_possible_score=
  std::make_pair(-(std::numeric_limits<score_t>::max()-score_t(1)),
		 -(std::numeric_limits<complexity_t>::max()-complexity_t(1)));

} //~namespace moses
