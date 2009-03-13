#ifndef _EDA_LOGGING_H
#define _EDA_LOGGING_H

#include <algorithm>
#include <iostream>
#include "eda/field_set.h"

namespace eda {

  struct cout_log_best_and_gen {
    template<typename It>
    void operator()(It from,It to,const field_set& fs,int gen) const {
      if (from==to)
	return;

      It best=std::max_element(from,to);
      std::cout << gen << " : "
		<< best->second << " " 
		<< fs.stream(best->first) << std::endl;
    }
  };

} //~namespace eda

#endif
