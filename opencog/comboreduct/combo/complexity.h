#ifndef _COMBO_COMPLEXITY_H
#define _COMBO_COMPLEXITY_H

//various routines dealing with computin the (algorithmic) complexity of
//particular combo programs

#include "ComboReduct/combo/vertex.h"

namespace combo {
  typedef int complexity_t ;  //note: complexity is negative with zero being
			      //the best - this allows for simple ordering

  complexity_t complexity(combo_tree::iterator);

} //~namespace combo

#endif
