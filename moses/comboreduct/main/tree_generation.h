/*
 * opencog/comboreduct/combo/tree_generation.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef TREE_GENERATION_H
#define TREE_GENERATION_H

#include <opencog/util/tree.h>

#include "../combo/vertex.h"
#include "../type_checker/type_tree.h"

namespace opencog { namespace trees {

  using namespace opencog::combo;

  template<int MIN_ARITY>
  class Generator {
  public:

    template<typename Selector>
    tree<typename Selector::value_type> operator()(const Selector& sel,
							 int dp) const {
      typename Selector::value_type tmp;
      tree<typename Selector::value_type> tr(tmp);
      build(sel,dp,tr,tr.begin());
      return tr;
    }

    template<typename Selector,typename T,typename iterator>
    void build(const Selector& sel,int dp,tree<T>& tr,iterator it) const {
        if (dp==1) {
            tr.replace(it,sel.select(0));
        } else {
            int arity(sel.select_arity(MIN_ARITY));
            it=tr.replace(it,sel.select(arity));
            --dp;
            for (int i=0;i<arity;++i)
                build(sel,dp,tr,tr.append_child(it));
        }
    }
  };

  typedef Generator<1> FullGenerator;
  typedef Generator<0> GrowGenerator;

  template<typename It,typename Selector>
  void ramped_half_and_half(It from,It to,const Selector& sel,
			    int minDp,int maxDp,bool type_check_enabled) {
    GrowGenerator gg;
    FullGenerator fg;
    bool gg_turn = true;
    bool well_typed;
    float ratio=((float)(maxDp-minDp+1))/(float)distance(from,to);
    for (It tr=from;tr!=to;) {
      (*tr)=(gg_turn?gg(sel,minDp+(int)(distance(from,tr)*ratio))
	     :fg(sel,minDp+(int)(distance(from,tr)*ratio)));
      well_typed = true;
      if(type_check_enabled) {
	std::stringstream strs;
    combo::combo_tree vtr;
	strs << (*tr);
	strs >> vtr;
	try {
	  combo::infer_type_tree(vtr);
	}
	catch(TypeCheckException) {
	  well_typed = false;
	}
      }
      if(well_typed) {
	++tr;
	gg_turn = !gg_turn;
      }
    }
  }

}} // ~namespaces trees opencog

#endif
