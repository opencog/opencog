/*
 * opencog/comboreduct/reduct/contin_rules.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _REDUCT_CONTIN_RULES_H
#define _REDUCT_CONTIN_RULES_H

#include "reduct.h"

namespace reduct {

  //x+0 -> x
  struct reduce_plus_zero : public crule<reduce_plus_zero> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //x*1 -> x
  //x*0 -> 0
  struct reduce_times_one_zero : public crule<reduce_times_one_zero> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //x/z+y/z -> (x+y)/z
  //or more generally, sum 1/z*x_i + sum y_j -> 1/z*(sum x_i) + sum y_j
  //when sevelar choices are possible the chosen one :
  //1)is the one that shorten the most the expression
  //2)if not unique, the lowest one according to the index order
  struct reduce_factorize_fraction : public crule<reduce_factorize_fraction> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //x*y+x*z -> x(y+z)
  //or more generally, sum x*y_i + sum z_j -> x*(sum y_i) + sum z_j
  //when sevelar choices are possible the chosen one :
  //1)is the one that shorten the most the expression
  //2)if not unique, the lowest one according to the index order
  //Note : if x is numerator under div, it works too
  struct reduce_factorize : public crule<reduce_factorize> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //x/c -> 1/c * x
  //x/(c*y) -> 1/c *x/y
  //x/0 -> 1 DELETED BECAUSE NO PROTECTION ANYMORE
  //0/x -> 0
  struct reduce_invert_constant : public crule<reduce_invert_constant> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };
  
  //(x*y)/(x*z) -> y/z
  //or more generally,
  //(prod x_i*prod y_j)/(prod x_i*prod z_k)-> prod y_j/prod z_k
  struct reduce_fraction : public crule<reduce_fraction> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };
  
  //(x/y)/z -> x/(y*z)
  //x/(y/z) -> (x*z)/y
  //x*(y/z) -> (x*y)/z,
  //more generally prod x_i * prod y_j/z_j -> (prod x_i * prod y_j)/(prod z_j)
  struct reduce_times_div : public crule<reduce_times_div> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //+(x) -> x
  //*(x) -> x
  struct reduce_plus_times_one_child : public crule<reduce_plus_times_one_child> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //log(x)+log(y) -> log(x*y), log(x)-log(y) -> log(x/y)
  //or more generally
  //sum log(x_i) - sum log(y_j) -> log((prod x_i)/(prod y_j))
  //works only if at least one log(x_i) exists otherwise
  //there would be a conflict with the rule log(c/x) -> -log((1/c)*x)
  struct reduce_sum_log : public crule<reduce_sum_log> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };
  
  //log(c/x) -> -log(c^1*x)
  //and also
  //log(exp(x)*y) -> x+log(y)
  //or more generally log(prod exp(x_i)*prod y_j) -> sum x_i +log(prod y_j)
  struct reduce_log_div_times : public crule<reduce_log_div_times> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

  //prod exp(x_i) -> exp(sum x_i)
  struct reduce_exp_times : public crule<reduce_exp_times> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };
   
  //x/exp(y) -> x*exp(-y)
  struct reduce_exp_div : public crule<reduce_exp_div> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };
  
  //exp(log(x)+y) -> x*exp(y)
  //or more generally, exp(sum log(x_i) + sum y_j) -> prod x_i * exp(sum y_j)
  struct reduce_exp_log : public crule<reduce_exp_log> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };
  
  //sin(x + c) -> sin(x + (c>pi? c-pi : (c<= pi? c+pi))
  //or more generally
  //sin(sum x_i + sum c_j) -> sin(sum x_i + ((sum c_j)+pi)%2pi -pi
  struct reduce_sin : public crule<reduce_sin> {
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
  };

} //~namespace reduct

#endif
