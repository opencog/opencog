/*
 * opencog/comboreduct/reduct/mixed_rules.h
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
#ifndef _REDUCT_MIXED_RULES_H
#define _REDUCT_MIXED_RULES_H

#include "reduct.h"

namespace opencog { namespace reduct {

//0<c*x -> 0<x if c>0
//0<c*x -> 0<-1*x if c<0
//WARNING : this rule is deprecated, use reduce_gt_zero_prod instead
struct reduce_gt_zero_times_const : public crule<reduce_gt_zero_times_const> {
    reduce_gt_zero_times_const() : crule<reduce_gt_zero_times_const>::crule("reduce_gt_zero_times_const") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//0<c*x*x -> false if c<0
//or more generally 0<c*x^p_x*y^p_y*exp(... where p_x, p_y... are divisible by 2
struct reduce_gt_zero_pair_power : public crule<reduce_gt_zero_pair_power> {
    reduce_gt_zero_pair_power() : crule<reduce_gt_zero_pair_power>::crule("reduce_gt_zero_pair_power") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//0<c/x -> 0<x if c>0
//0<c/x -> 0<-1*x if c<0
//WARNING : this rule is deprecated, use reduce_gt_zero_div instead
struct reduce_gt_zero_const_div : public crule<reduce_gt_zero_const_div>
{
    reduce_gt_zero_const_div() : crule<reduce_gt_zero_const_div>::crule("reduce_gt_zero_const_div") {}
        void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

#ifndef ABS_LOG
// 0<log(x) -> 0<-1+x
struct reduce_gt_zero_log : public crule<reduce_gt_zero_log>
{
    reduce_gt_zero_log() : crule<reduce_gt_zero_log>::crule("reduce_gt_zero_log") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};
#endif

// 0<exp(x) -> true
struct reduce_gt_zero_exp : public crule<reduce_gt_zero_exp>
{
    reduce_gt_zero_exp() : crule<reduce_gt_zero_exp>::crule("reduce_gt_zero_exp") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//0<-1*exp(x) -> false
//WARNING : this rule is deprecated, use reduce_gt_zero_prod instead
struct reduce_gt_zero_minus_exp : public crule<reduce_gt_zero_minus_exp>
{
    reduce_gt_zero_minus_exp() : crule<reduce_gt_zero_minus_exp>::crule("reduce_gt_zero_minus_exp") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//0<y*exp(x) -> 0<y
//WARNING : this rule is deprecated, use reduce_gt_zero_prod instead
struct reduce_gt_zero_prod_exp : public crule<reduce_gt_zero_prod_exp> {
    reduce_gt_zero_prod_exp() : crule<reduce_gt_zero_prod_exp>::crule("reduce_gt_zero_prod_exp") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//0<c+sin(x) -> true if c>1
//0<c+sin(x) -> false if c<=1
//WARNING : this rule is deprecated, use reduce_gt_zero_sum_sin instead
struct reduce_gt_zero_const_sum_sin : public crule<reduce_gt_zero_const_sum_sin> {
    reduce_gt_zero_const_sum_sin() : crule<reduce_gt_zero_const_sum_sin>::crule("reduce_gt_zero_const_sum_sin") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

// 0<impulse(x) -> x   (remember x is boolean)
struct reduce_gt_zero_impulse : public crule<reduce_gt_zero_impulse> {
    reduce_gt_zero_impulse() : crule<reduce_gt_zero_impulse>::crule("reduce_gt_zero_impulse") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//prod impulse(x_i)^p_i -> prod impulse(x_i)
struct reduce_impulse_power : public crule<reduce_impulse_power> {
    reduce_impulse_power() : crule<reduce_impulse_power>::crule("reduce_impulse_power") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//prod impulse(x_i) * z -> impulse(and x_i) * z
struct reduce_impulse_prod : public crule<reduce_impulse_prod> {
    reduce_impulse_prod() : crule<reduce_impulse_prod>::crule("reduce_impulse_prod") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//0<(sum impulse(x_i)) -> or x_i
struct reduce_impulse_sum : public crule<reduce_impulse_sum> {
    reduce_impulse_sum() : crule<reduce_impulse_sum>::crule("reduce_impulse_sum") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//if(x 1 0) -> impulse(x)
//if(x 0 1) -> impulse(NOT(x))
struct reduce_contin_if_to_impulse : public crule<reduce_contin_if_to_impulse> {
    reduce_contin_if_to_impulse() : crule<reduce_contin_if_to_impulse>::crule("reduce_contin_if_to_impulse") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//if(true x y) -> x
//if(false x y) -> y
//if(x if(x y z) w) -> if(x y w)
//if(x y if(x z w)) -> if(x y w)
struct reduce_contin_if : public crule<reduce_contin_if> {
    reduce_contin_if() : crule<reduce_contin_if>::crule("reduce_contin_if") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//op(if(x y1 z1) if(x y2 z2)) -> if(x op(y1 y2) op(z1 z2))
struct reduce_op_contin_if : public crule<reduce_op_contin_if> {
    reduce_op_contin_if() : crule<reduce_op_contin_if>::crule("reduce_op_contin_if") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//contin_if(x op(y z) op(y w)) -> op(y contin_if(x z w))
//op in {+, *, /}. If op is / the order of argument is respected
struct reduce_contin_if_inner_op : public crule<reduce_contin_if_inner_op> {
    reduce_contin_if_inner_op() : crule<reduce_contin_if_inner_op>::crule("reduce_contin_if_inner_op") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//contin_if(x exp1[x] exp2[x]) -> if(x exp1[true] exp2[false])
struct reduce_contin_if_substitute_cond
    : public crule<reduce_contin_if_substitute_cond> 
{
    reduce_contin_if_substitute_cond() : crule<reduce_contin_if_substitute_cond>::crule("reduce_contin_if_substitute_cond") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//0<x+c1 and 0<x+c2 -> 0<x+min(c1, c2)
//or more generally
//0<sum x_i +c1 and 0<sum x_i +c2 and 0<sum x_i +c3...
//->sum x_i + min(c1, c2, c3, ...)
//
//and
//
//0<x+c1 or 0<x+c2 -> 0<x+max(c1, c2)
//or more generally
//0<sum x_i +c1 and 0<sum x_i +c2 and 0<sum x_i +c3...
//->sum x_i + max(c1, c2, c3, ...)
//WARNING : this is no longer necessary
//due to the extension of reduce_from_assumptions
//to deal with inequalities
struct reduce_junction_gt_zero_sum_constant
    : public crule<reduce_junction_gt_zero_sum_constant> 
{
    reduce_junction_gt_zero_sum_constant() : crule<reduce_junction_gt_zero_sum_constant>::crule("reduce_junction_gt_zero_sum_constant") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

// look up the assumptions and replace by true if present (or implies)
// or false if not(assum) is present (or implies to be false). The
// rule given in argument is used by reduce_from_assumptions::implies
// and can be fully recursive (I guess?).
struct reduce_from_assumptions : public crule<reduce_from_assumptions> {
    reduce_from_assumptions(const rule& r) : crule<reduce_from_assumptions>::crule("reduce_from_assumptions"), _reduction(&r) { }    
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
    bool implies(const combo_tree& tr, combo_tree::iterator it1, combo_tree::iterator it2) const;
protected:
    const rule* _reduction;
};

//if(x y z) -> if(NOT(x) z y)  if |rule(NOT(x))|<|rule(x)|
struct reduce_contin_if_not : public crule<reduce_contin_if_not> {
    reduce_contin_if_not(const rule& r) : crule<reduce_contin_if_not>::crule("reduce_contin_if_not"), _reduction(&r) { }
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
protected:
    const rule* _reduction;
};

//0<sum x_i -> true    if 0<x_i -> true forall i
//0<sum x_i -> false   if 0<x_i -> false forall i
struct reduce_gt_zero_sum : public crule<reduce_gt_zero_sum> {
    reduce_gt_zero_sum(const rule& r) : crule<reduce_gt_zero_sum>::crule("reduce_gt_zero_sum"), _reduction(&r) { }
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
protected:
    const rule* _reduction;
};

//0<prod x_i -> 0<prod x_i    with x_i=1 if 0<x_i -> true
//                            with x_i=-1 if 0<-x_i -> true
//0<prod x_i -> false     if exist x_i==0, 
//                           that is 0<x_i -> false and 0<-1*x_i -> false
struct reduce_gt_zero_prod : public crule<reduce_gt_zero_prod> {
    reduce_gt_zero_prod(const rule& r)
        : crule<reduce_gt_zero_prod>::crule("reduce_gt_zero_prod"), 
          _complete_reduction(&r), _reduction_without_itself(&r) { }
    reduce_gt_zero_prod(const rule& r1, const rule& r2) 
        : crule<reduce_gt_zero_prod>::crule("reduce_gt_zero_prod"), 
          _complete_reduction(&r1), _reduction_without_itself(&r2) { }
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
protected:
    const rule* _complete_reduction;
    const rule* _reduction_without_itself;
};

//0<x/y -> 0<x    if 0<y -> true
//0<x/y -> 0<y    if 0<x -> true
//0<x/y -> 0<-1*y if 0<-1*x -> true
//0<x/y -> 0<-1*x if 0<-1*y -> true
//0<x/y -> false  if x==0, that is not(0<x) -> true and not(0<-x) -> true
struct reduce_gt_zero_div : public crule<reduce_gt_zero_div> {
    reduce_gt_zero_div(const rule& r) : crule<reduce_gt_zero_div>::crule("reduce_gt_zero_div"), _reduction(&r) { }
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
protected:
    const rule* _reduction;
};

//0<x+sin(y) -> true  if 0<x-1 -> true
//0<x+sin(y) -> false if 0<x+1 -> false
struct reduce_gt_zero_sum_sin : public crule<reduce_gt_zero_sum_sin> {
    reduce_gt_zero_sum_sin(const rule& r) : crule<reduce_gt_zero_sum_sin>::crule("reduce_gt_zero_sum_sin"), _reduction(&r) { }
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
protected:
    const rule* _reduction;
};

//0<sin(y) -> true  if 0<y -> true and 0<pi-y -> true
//0<sin(y) -> false if 0<y -> false and 0<-(y+pi) ->false
struct reduce_gt_zero_sin : public crule<reduce_gt_zero_sin> {
    reduce_gt_zero_sin(const rule& r) : crule<reduce_gt_zero_sin>::crule("reduce_gt_zero_sin"), _reduction(&r) { }
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
protected:
    const rule* _reduction;
};


  
//reduce inequality from assumptions :
//try to see if an inequality is a positive linear conbination of any
//other inequalities present as assumptions
struct reduce_inequality_from_assumptions : public crule<reduce_inequality_from_assumptions> {
    reduce_inequality_from_assumptions() :
        crule<reduce_inequality_from_assumptions>::crule("reduce_inequality_from_assumptions") {}
    //double_matrix, a matrix of doubles
    //used by reduce_inequality_from_assumptions
    //each inner vector is a row, the outer vector is a list of rows
    typedef std::vector< std::vector<double> > double_matrix;
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
    //take a tree that has id::times at it.
    //Take off the coef and return it, modify 'it' accordigly
    //if there is only coef then 'it' is not valid
    contin_t splitCoefExpression(combo_tree& tr, combo_tree::iterator& it) const;
    //perform Gauss-Jordan elimination
    bool gaussJordanElimination(double_matrix& dm, double eps = 1.0e-10) const;
    //perform gaussian elimination
    void gaussianElimination(double_matrix& dm) const;
    //hand cooked gaussian elimination which is working better
    bool gaussianElimination2(double_matrix& dm, double eps = 1.0e-10) const;
    //take a matrix in row echelon form
    //and fill solution with the solution of the system by back-substitution
    //solution is assumed to be empty before calling the method
    //return true if the solution is a valid one (exist and unique)
    bool backSubstitution(double_matrix& dm, std::vector<double>& solution, double eps = 1.0e-10) const;
};
  
} // ~namespace reduct
} // ~namespace opencog

#endif
