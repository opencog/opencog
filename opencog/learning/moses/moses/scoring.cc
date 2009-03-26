/*
 * opencog/learning/moses/moses/scoring.cc
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
#include "moses/scoring.h"
#include "util/numeric.h"
#include <cmath>

namespace moses {

  double information_theoretic_bits(const eda::field_set& fs) {
    double res=0;
    foreach(const eda::field_set::disc_spec& d,fs.disc_and_bits())
      res+=log2(double(d.arity));
    foreach(const eda::field_set::contin_spec& c,fs.contin())
      res+=log2(3.0)*double(c.depth);
    foreach(const eda::field_set::onto_spec& o,fs.onto())
      res+=log2(double(o.branching))*double(o.depth);
    //std::cout << "itb = " << res << std::endl;
    return res;
  }

  int logical_score::operator()(const combo_tree& tr) const {
    // std::cout << "scoring " << tr << " " << arity << " " 
    //   << target << " " << combo::truth_table(tr,arity) << " "
    //   << (target.hamming_distance(combo::truth_table(tr,arity))) << std::endl; // PJ
    
    return -int(target.hamming_distance(combo::truth_table(tr,arity,rng)));
  }
  behavioral_score logical_bscore::operator()(const combo_tree& tr) const { 
    combo::truth_table tt(tr,arity,rng);
    behavioral_score bs(target.size());

    std::transform(tt.begin(),tt.end(),target.begin(),bs.begin(),
		   std::not_equal_to<bool>()); //not_equal because lower is better

    return bs;
  }

  contin_t contin_score::operator()(const combo_tree& tr) const {
    try { 
      std::cout << "scoring " << tr << std::endl;
      contin_t sc=-target.abs_distance(combo::contin_table(tr,rands,rng));
      std::cout << sc << " X " << tr << std::endl;
      return sc;
    } catch (...) {
      std::cout << "threw" << std::endl;
      return get_score(worst_possible_score);
    }
  }
  behavioral_score contin_bscore::operator()(const combo_tree& tr) const { 
    combo::contin_table ct(tr,rands,rng);
    behavioral_score bs(target.size());

    behavioral_score::iterator dst=bs.begin();
    for (combo::contin_table::const_iterator it1=ct.begin(),it2=target.begin();
	 it1!=ct.end();)
      *dst++=fabs((*it1++)-(*it2++));

    return bs;
  }

  tribool dominates(const behavioral_score& x,const behavioral_score& y) {
    //everything dominates an empty vector
    if (x.empty()) { 
      if (y.empty())
	return indeterminate;
      return false;
    } else if (y.empty()) {
      return true;
    }    
    
    tribool res=indeterminate;
    for (behavioral_score::const_iterator xit=x.begin(),yit=y.begin();
	 xit!=x.end();++xit,++yit) {
      if (*xit<*yit) { //individual elements are assumed to represent error
	if (!res)
	  return indeterminate;
	else
	  res=true;
      } else if (*yit<*xit) {
	if (res)
	  return indeterminate;
	else
	  res=false;
      }
    }
    return res;
  }

} //~namespace moses
