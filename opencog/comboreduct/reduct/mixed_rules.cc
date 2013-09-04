/*
 * opencog/comboreduct/reduct/mixed_rules.cc
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
#include "mixed_rules.h"
#include <opencog/comboreduct/type_checker/type_tree.h>
#include <opencog/util/exceptions.h>
#include <map>

namespace opencog { namespace reduct {
typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;

// 0<c*x -> 0<x if c>0
// 0<c*x -> 0<-1*x if c<0
//WARNING : this rule is deprecated, use reduce_gt_zero_prod instead
void reduce_gt_zero_times_const::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(), 
                  "combo_tree node should have exactly one child (reduce_gt_zero_times_const).");
        pre_it it_child = it.begin();
        if(*it_child==id::times) {
            for(sib_it sib = it_child.begin(); sib != it_child.end();) {
                if(is_contin(*sib)) {
                    contin_t c = get_contin(*sib);
                    if(c>0) {
                        if(it_child.has_one_child()) {
                            tr.erase_children(it);
                            *it=id::logical_true;
                            return;
                        }
                        else sib = tr.erase(sib);
                    }
                    else if(c<0) {
                        if(it_child.has_one_child()) {
                            tr.erase_children(it);
                            *it=id::logical_false;
                            return;
                        }
                        else {
                            *sib=-1.0;
                            ++sib;
                        }
                    }
                    else {//that is c==0
                        tr.erase_children(it);
                        *it=id::logical_false;
                        return;
                    }
                }
                else ++sib;
            }
        }
    }
}

// 0<c*x*x -> false if c<0
// or more generally 0<c*x^p_x*y^p_y*exp(... where p_x, p_y... are divisible by 2
void reduce_gt_zero_pair_power::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    //associate for each subtree its parity
    typedef std::map<pre_it, bool,
                     opencog::lexicographic_subtree_order<vertex> > subtree_parity;
    typedef subtree_parity::iterator subtree_parity_it;
    if(*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_gt_zero_pair_power).");
        pre_it it_child = it.begin();
        bool is_neg_const = false;
        subtree_parity sp;
        if(*it_child==id::times) {
            for(sib_it sib = it_child.begin(); sib != it_child.end(); ++sib) {
                if(is_contin(*sib)) {
                    if(get_contin(*sib)<0)
                        is_neg_const = !is_neg_const;
                }
                else {
                    pre_it pre_sib = sib;
                    subtree_parity_it sp_it = sp.find(pre_sib);
                    if(sp_it==sp.end()) {
                        std::pair<pre_it, bool> p(pre_sib, false);
                        sp.insert(p);
                    }
                    else {
                        bool& par = sp_it->second;
                        par = !par;
                    }
                }
            }
            if(is_neg_const) {
                if(!sp.empty()) {
                    for(subtree_parity_it sp_it=sp.begin();sp_it!=sp.end();++sp_it) {
                        if(!sp_it->second) return;
                    }
                }
                //if still in the code then all sp_it are positive
                //and so the expression is negative
                tr.erase_children(it);
                *it=id::logical_false;
            }
        }
    }
}

// 0<c/x -> 0<x if c>0
// 0<c/x -> 0<-1*x if c<0
//WARNING : this rule is deprecated, use reduce_gt_zero_div instead
void reduce_gt_zero_const_div::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(), 
                  "combo_tree node should have exactly one child (reduce_gt_zero_const_div)."); 
        pre_it it_child = it.begin();
        if(*it_child==id::div) {
            OC_ASSERT(it_child.number_of_children() == 2,
                      "combo_tree child node should have exactly two children (reduce_gt_zero_const_div).");
            pre_it num = it_child.begin();
            if(is_contin(*num)) {
                contin_t c = get_contin(*num);
                if(c>0) {
                    pre_it denom = tr.child(it_child, 1);
                    tr.move_before(it_child, denom);
                    tr.erase(it_child);
                }
                else if(c<0) {
                    *num = -1.0;
                    *it_child = id::times;
                }
                else {//that is c==0
                    tr.erase_children(it);
                    *it=id::logical_false;
                    return;
                }
            }
        }
    }
}

#ifndef ABS_LOG
// 0<log(x) -> 0<-1+x  
void reduce_gt_zero_log::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),             
                  "combo_tree node should have exactly one child (reduce_gt_zero_log)."); 
        pre_it it_child = it.begin();
        if(*it_child==id::log) {
            OC_ASSERT(it_child.has_one_child(),
                      "combo_tree child node should have exactly one child (reduce_gt_zero_log).");
            *it_child = id::plus;
            tr.append_child(it_child, -1.0);
        }
    }
}
#endif

// 0<exp(x) -> true
void reduce_gt_zero_exp::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it == id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_gt_zero_exp)."); 
        pre_it it_child = it.begin();
        if (*it_child == id::exp) {
            OC_ASSERT(it_child.has_one_child(),
                      "combo_tree child node should have exactly one child (reduce_gt_zero_exp).");
            tr.erase_children(it);
            *it = id::logical_true;
        }
    }
}

// 0<-1*exp(x) -> false
// WARNING : this rule is deprecated, use reduce_gt_zero_prod instead
void reduce_gt_zero_minus_exp::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it == id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_gt_zero_minus_exp).");
        pre_it it_child = it.begin();
        if (*it_child == id::times) {
            bool is_neg_const = false;
            for (sib_it sib = it_child.begin(); sib != it_child.end(); ++sib) {
                if (is_contin(*sib)) {
                    if (get_contin(*sib) < 0)
                        is_neg_const = !is_neg_const;
                }
                else if (*sib!=id::exp)
                    return;
            }
            if (is_neg_const) {
                tr.erase_children(it);
                *it = id::logical_false;
            }
        }
    }
}

// 0<y*exp(x) -> 0<y
// WARNING : this rule is deprecated, use reduce_gt_zero_prod instead
void reduce_gt_zero_prod_exp::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(), 
                  "combo_tree node should have exactly one child.");
        pre_it it_child = it.begin();
        if(*it_child==id::times) {
            for(sib_it sib = it_child.begin(); sib != it_child.end();) {
                if(*sib==id::exp)
                    sib = tr.erase(sib);
                else
                    ++sib;
            }
            if(it_child.is_childless()) {
                tr.erase_children(it);
                *it = id::logical_true;
            }
        }
    }
}

// 0<c+sin(x) -> true if c>1
// 0<c+sin(x) -> false if c<=1
// WARNING : this rule is deprecated, use reduce_gt_zero_sum_sin instead
void reduce_gt_zero_const_sum_sin::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(), 
                  "combo_tree node should have exactly one child (reduce_gt_zero_const_sum_sin).");
        pre_it it_child = it.begin();
        if(*it_child==id::plus) {
            contin_t total = 0.0;
            int sin_count = 0;
            for(sib_it sib = it_child.begin(); sib != it_child.end(); ++sib) {
                if(is_contin(*sib))
                    total = total + get_contin(*sib);
                else if(*sib==id::sin)
                    ++sin_count;
                else return;
            }
            if(sin_count > 0) {
                if(total > (contin_t)sin_count) {
                    tr.erase_children(it);
                    *it = id::logical_true;
                }
                else if(total <= (contin_t)-sin_count) {
                    tr.erase_children(it);
                    *it = id::logical_false;
                }
            }
        }
    }
}

// 0<impulse(x) -> x
void reduce_gt_zero_impulse::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_gt_zero_impulse).");
        pre_it it_child = it.begin();
        if(*it_child==id::impulse) {
            OC_ASSERT(it_child.has_one_child(),
                      "combo_tree child node should have exactly one child (reduce_gt_zero_impulse).");
            pre_it new_it = it_child.begin();
            *it = *new_it;
            tr.reparent(it, new_it);
            tr.erase(it_child);
        }
    }
}

// prod impulse(x_i)^p_i -> prod impulse(x_i)
void reduce_impulse_power::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    typedef std::set<pre_it, opencog::lexicographic_subtree_order<vertex>
                     > subtree_quotient;
        if(*it==id::times) {
            subtree_quotient sq;
            for(sib_it sib = it.begin(); sib != it.end();) {
                if(*sib==id::impulse) {
                    OC_ASSERT(sib.has_one_child(),
                              "combo_tree sibling node should have exactly one child (reduce_impulse_power).");
                    if(sq.count(pre_it(sib)) > 0) {
                        sib = tr.erase(sib);
                    }
                    else {
                        sq.insert(pre_it(sib));
                        ++sib;
                    }
                }
                else ++sib;
            }
            if(it.has_one_child()) {
                *it = *it.begin();
                tr.erase(tr.flatten(it.begin()));
            }
        }
}

// prod impulse(x_i) * z -> impulse(and x_i) * z
void reduce_impulse_prod::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it==id::times) {
        pre_it fic; //child of the first impulse
        pre_it new_and;
        for(sib_it sib = it.begin(); sib != it.end();) {
            if(*sib==id::impulse) {
                OC_ASSERT(sib.has_one_child(),
                          "combo_tree sibling node should have exactly one child (reduce_impulse_prod).");
                if(tr.is_valid(fic)) {
                    if(!tr.is_valid(new_and))
                        new_and = tr.wrap(fic, id::logical_and);
                    tr.move_after(fic, pre_it(sib.begin()));
                    sib = tr.erase(sib);
                }
                else {
                    fic = pre_it(sib.begin());
                    ++sib;
                }
            }
            else ++sib;
        }
        if(it.has_one_child()) {
            *it = *it.begin();
            tr.erase(tr.flatten(it.begin()));
        }
    }
}

// 0<(sum impulse(x_i)) -> or x_i
void reduce_impulse_sum::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree sibling node should have exactly one child (reduce_impulse_sum).");
        pre_it it_child = it.begin();
        if(*it_child==id::plus) {
            for(sib_it sib = it_child.begin(); sib != it_child.end(); ++sib)
                if(*sib!=id::impulse) return;
            //we are now sure that all arguments of + are impulses
            *it = id::logical_or;
            tr.erase(tr.flatten(it_child));
            for(sib_it sib = it.begin(); sib != it.end(); ++sib) {
                sib = tr.erase(tr.flatten(sib));
            }
        }
    }
}

// if(x 1 0) -> impulse(x)
// if(x 0 1) -> impulse(NOT(x))
void reduce_contin_if_to_impulse::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::contin_if) {
        OC_ASSERT(tr.number_of_children(it)==3,
                  "combo_tree node should have exactly three children (reduce_contin_if_to_impulse).");
        pre_it cond = tr.child(it, 0);
        pre_it b1 = tr.child(it, 1);
        pre_it b2 = tr.child(it, 2);
        if(is_contin(*b1) && is_contin(*b2)) {
            contin_t cb1 = get_contin(*b1);
            contin_t cb2 = get_contin(*b2);
            if(cb1==1.0 && cb2==0.0) {
                *it = id::impulse;
                tr.erase(b1);
                tr.erase(b2);
            }
            else if(cb1==0.0 && cb2==1.0) {
                tr.wrap(cond, id::logical_not);
                *it = id::impulse;
                tr.erase(b1);
                tr.erase(b2);
            }
        }
    }
}

// if(true x y) -> x
// if(false x y) -> y
// if(x if(x y z) w) -> if(x y w)
// if(x y if(x z w)) -> if(x y w)
void reduce_contin_if::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it==id::contin_if) {
        OC_ASSERT(tr.number_of_children(it)==3,
                  "combo_tree node should have exactly three children (reduce_contin_if).");
        pre_it cond = tr.child(it, 0);
        pre_it b1 = tr.child(it, 1);
        pre_it b2 = tr.child(it, 2);
        if (*cond == id::logical_true) {
            *it = *b1;
            tr.flatten(b1);
            tr.erase(cond);
            tr.erase(b1);
            tr.erase(b2);
        }
        else if(*cond==id::logical_false) {
            *it = *b2;
            tr.flatten(b2);
            tr.erase(cond);
            tr.erase(b1);
            tr.erase(b2);
        }
        else if(*b1==id::contin_if) {
            OC_ASSERT(tr.number_of_children(b1)==3, 
                      "combo_tree node should have exactly three children (reduce_contin_if - b1).");
            pre_it b1_cond = tr.child(b1, 0);
            pre_it b1_b1 = tr.child(b1, 1);
            if(tr.equal_subtree(cond, b1_cond)) {
                tr.move_before(b1, b1_b1);
                tr.erase(b1);
            }
        }
        else if(*b2==id::contin_if) {
            OC_ASSERT(tr.number_of_children(b2)==3,
                      "combo_tree node should have exactly three children (reduce_contin_if - b2).");
            pre_it b2_cond = tr.child(b2, 0);
            pre_it b2_b2 = tr.child(b2, 2);
            if(tr.equal_subtree(cond, b2_cond)) {
                tr.move_before(b2, b2_b2);
                tr.erase(b2);
            }
        }
    }
}
  
// op(if(x y1 z1) if(x y2 z2)) -> if(x op(y1 y2) op(z1 z2))
void reduce_op_contin_if::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    typedef std::set<pre_it, opencog::lexicographic_subtree_order<vertex> > subtree_quotient;
    typedef subtree_quotient::const_iterator subtree_quotient_const_it;
    if(*it==id::div) {
        OC_ASSERT(tr.number_of_children(it)==2, 
                  "combo_tree node should have exactly two children (reduce_op_contin_if).");
        pre_it num = tr.child(it, 0);
        pre_it denum = tr.child(it, 1);
        if(*num==id::contin_if && *denum==id::contin_if) {
            OC_ASSERT(tr.number_of_children(num)==3,
                      "combo_tree node should have exactly two children (reduce_op_contin_if - num).");
            OC_ASSERT(tr.number_of_children(denum)==3,
                      "combo_tree node should have exactly two children (reduce_op_contin_if - denum).");
            pre_it num_cond = tr.child(num, 0);
            pre_it denum_cond = tr.child(denum, 0);
            if(tr.equal_subtree(num_cond, denum_cond)) {
                *it=id::contin_if;
                //move x as first child of the new if
                tr.move_before(num, num_cond);
                tr.erase(denum_cond);
                *num=id::div;
                *denum=id::div;
                tr.swap(pre_it(tr.child(num, 1)), pre_it(tr.child(denum, 0)));
            }
        }
    }
    else if(*it==id::plus || *it==id::times) {
        subtree_quotient sq;
        for(sib_it sib = it.begin(); sib != it.end(); ) {
            if(*sib==id::contin_if) {
                OC_ASSERT(tr.number_of_children(sib)==3,
                          "combo_tree sibling node should have exactly two children (reduce_op_contin_if).");
                pre_it sib_cond = tr.child(sib, 0);
                subtree_quotient_const_it sqi = sq.find(sib_cond);
                if(sqi==sq.end()) {
                    sq.insert(pre_it(sib_cond));
                    ++sib;
                }
                else {
                    pre_it pivot_cond = *sqi;
                    pre_it pivot_b1 = tr.next_sibling(pivot_cond);
                    pre_it pivot_b2 = tr.next_sibling(pivot_b1);
                    tr.erase(tr.child(sib, 0));
                    builtin it_builtin = get_builtin(*it);
                    if(*pivot_b1==it_builtin)
                        pivot_b1 = pivot_b1.begin();
                    else tr.wrap(pivot_b1, *it);
                    tr.move_before(pivot_b1, pre_it(tr.child(sib, 0)));
                    if(*pivot_b2==it_builtin)
                        pivot_b2 = pivot_b2.begin();
                    else tr.wrap(pivot_b2, *it);
                    tr.move_before(pivot_b2, pre_it(tr.child(sib, 0)));
                    sib = tr.erase(sib);
                }
            }
            else ++sib;
        }
        if(it.has_one_child()) {
            *it=*it.begin();
            tr.erase(tr.flatten(it.begin()));
        }
    }
}


// contin_if(x op(y z) op(y w)) -> op(y contin_if(x z w))
// op in {+, *, /}. If op is / the order of argument is respected
void reduce_contin_if_inner_op::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    typedef std::multiset<pre_it, opencog::lexicographic_subtree_order<vertex> > subtree_quotient;
    typedef subtree_quotient::iterator subtree_quotient_it;
    if(*it==id::contin_if) {
        OC_ASSERT(tr.number_of_children(it)==3,
                  "combo_tree node should have exactly three children (reduce_contin_if_inner_op).");
        pre_it b1 = tr.child(it, 1);
        pre_it b2 = tr.child(it, 2);
        if(is_builtin(*b1)) {
            builtin b1_builtin = get_builtin(*b1);
            if(b1_builtin==*b2) {
                if(b1_builtin==id::div) {
                    OC_ASSERT(tr.number_of_children(b1)==2,
                              "combo_tree node should have exactly two children (reduce_contin_if_inner_op - b1).");
                    OC_ASSERT(tr.number_of_children(b2)==2,
                              "combo_tree node should have exactly two children (reduce_contin_if_inner_op - b2).");	    pre_it b1_num = tr.child(b1, 0);
                    pre_it b1_denum = tr.child(b1, 1);
                    pre_it b2_num = tr.child(b2, 0);
                    pre_it b2_denum = tr.child(b2, 1);
                    if(tr.equal_subtree(b1_num, b2_num)) {
                        *it = id::div;
                        pre_it new_if = tr.prepend_child(it, id::contin_if);
                        tr.reparent(new_if, ++(it.begin()), it.end());
                        tr.move_before(new_if, b1_num);
                        tr.erase(b2_num);
                        tr.erase(tr.flatten(b1));
                        tr.erase(tr.flatten(b2));
                    }
                    else if(tr.equal_subtree(b1_denum, b2_denum)) {
                        *it = id::div;
                        pre_it new_if = tr.prepend_child(it, id::contin_if);
                        tr.reparent(new_if, ++(it.begin()), it.end());
                        tr.move_after(new_if, b1_denum);
                        tr.erase(b2_denum);
                        tr.erase(tr.flatten(b1));
                        tr.erase(tr.flatten(b2));
                    }
                }
                if(b1_builtin==id::plus || b1_builtin==id::times) {
                    subtree_quotient sq;
                    pre_it new_if;
                    //fill sq
                    for(sib_it sib = b1.begin(); sib != b1.end(); ++sib)
                        sq.insert(pre_it(sib));
                    //move redundant arguments of op out of contin_if
                    subtree_quotient_it sq_it;
                    for(sib_it sib = b2.begin(); sib != b2.end();) {
                        sq_it = sq.find(pre_it(sib));
                        if(sq_it!=sq.end()) {
                            if(!tr.is_valid(new_if)) {
                                *it = b1_builtin;
                                new_if = tr.prepend_child(it, id::contin_if);
                                tr.reparent(new_if, ++(it.begin()), it.end());
                            }
                            tr.move_after(new_if, *sq_it);
                            sib = tr.erase(sib);
                            sq.erase(sq_it);
                        }
                        else ++sib;
                    }
                    //check if old op is empty or contin_if is empty
                    if(b1.has_one_child())
                        tr.erase(tr.flatten(b1));
                    else if(b1.is_childless())
                        *b1 = (b1_builtin==id::plus?0.0:1.0);
                    if(b2.has_one_child())
                        tr.erase(tr.flatten(b2));
                    else if(b2.is_childless())
                        *b2 = (b1_builtin==id::plus?0.0:1.0);
                }
            }
        }
    }
}

// contin_if(x exp1[x] exp2[x]) -> if(x exp1[true] exp2[false])
void reduce_contin_if_substitute_cond::operator()(combo_tree& tr,
                                                  combo_tree::iterator it) const
{
    if (*it==id::contin_if) {
        OC_ASSERT(tr.number_of_children(it)==3,
                  "combo_tree node should have exactly three children (reduce_contin_if_substitute_cond).");
        pre_it cond = tr.child(it, 0);
        pre_it b1 = tr.child(it, 1);
        pre_it b2 = tr.child(it, 2);
        if(!may_have_side_effects(cond)) {
            if(*cond!=id::logical_true && *cond!=id::logical_false) {
                pre_it cond_in_b1 =
                    tr.find_subtree(cond, pre_it(b1.begin()), b2);
                while(tr.is_valid(cond_in_b1)) {
                    *cond_in_b1 = id::logical_true;
                    tr.erase_children(cond_in_b1);
                    cond_in_b1 =
                        tr.find_subtree(cond, ++cond_in_b1, b2);
                }
                pre_it cond_in_b2 =
                    tr.find_subtree(cond, pre_it(b2.begin()), pre_it(b2.end()));
                while(tr.is_valid(cond_in_b2)) {
                    *cond_in_b2 = id::logical_false;
                    tr.erase_children(cond_in_b2);
                    cond_in_b2 =
                        tr.find_subtree(cond, ++cond_in_b2, pre_it(b2.end()));
                }
            }
        }
    }
}
        
// 0<x+c1 and 0<x+c2 -> 0<x+min(c1, c2)
// or more generally
// 0<sum x_i +c1 and 0<sum x_i +c2 and 0<sum x_i +c3...
// ->sum x_i + min(c1, c2, c3, ...)
//
// and
//
// 0<x+c1 or 0<x+c2 -> 0<x+max(c1, c2)
// or more generally
// 0<sum x_i +c1 and 0<sum x_i +c2 and 0<sum x_i +c3...
// ->sum x_i + max(c1, c2, c3, ...)
void reduce_junction_gt_zero_sum_constant::operator()(combo_tree& tr,
                                                      combo_tree::iterator it) const
{
    // the first pre_it is the root of the subtree s obtained from
    // 0<s+c
    // such that s is a sum of non-constant term and c is a constant
    // the second argument, contin_t, is c
    typedef std::map<pre_it, contin_t, opencog::lexicographic_subtree_order<vertex> > subtree_quotient;
    typedef subtree_quotient::iterator subtree_quotient_it;
    if(*it==id::logical_and || *it==id::logical_or) {
        subtree_quotient sq;
        for(sib_it sib = it.begin(); sib != it.end();) {
            if(*sib==id::greater_than_zero) {
                OC_ASSERT(sib.has_one_child(),
                          "combo_tree node should have exactly one child (reduce_junction_gt_zero_const_sum_constante).");
                pre_it sib_child = sib.begin();
                if(*sib_child==id::plus) {
                    contin_t c = 0.0;
                    for(sib_it plus_sib = sib_child.begin(); plus_sib != sib_child.end();)
                        if(is_contin(*plus_sib)) {
                            c = c + get_contin(*plus_sib);
                            plus_sib = tr.erase(plus_sib);
                        }
                        else ++plus_sib;
                    if(sib_child.is_childless()) {
                        *sib_child = c;
                        ++sib;
                    }
                    else {
                        subtree_quotient_it sqi = sq.find(pre_it(sib_child));
                        if(sqi==sq.end()) {
                            std::pair<pre_it, contin_t> p(pre_it(sib_child), c);
                            sq.insert(p);
                            ++sib;
                        }
                        else {
                            contin_t& c_ref = sqi->second;
                            c_ref = (*it==id::logical_and?
                                     std::min(c, c_ref) : std::max(c, c_ref));
                            sib = tr.erase(sib);
                        }
                    }
                }
                else ++sib;
            }
            else ++sib;
        }
        //add the right constant [that is min/max(c1, c2, ....)] to each
        //reduced inequality
        for(subtree_quotient_it i = sq.begin(); i != sq.end(); ++i) {
            contin_t val = i->second;
            if(val != 0.0)
                tr.append_child(i->first, i->second);
        }
    }
}

// implies takes a tree and 2 iterators and tells if it1 implies it2
// for instance x implies x, because x==x if x!=y for instance x==2<z
// implies y==3<z
bool reduce_from_assumptions::implies(const combo_tree& tr,
                                      combo_tree::iterator it1,
                                      combo_tree::iterator it2) const
{
    // if there are syntactly equal clearly it1 implies it2
    if (tr.equal_subtree(it1, it2))
        return true;
    // otherwise deal with inequalities
    else {
        pre_it ine1_child; // child of inequality ot it1
        bool is_strict1 = true; // = true just to get rid of C++ warning
        pre_it ine2_child; // child of inequality of it2
        bool is_strict2 = true; // = true just to get rid of C++ warning
        // determine if ine1_child is a strict inequality
        if(*it1==id::greater_than_zero) {
            OC_ASSERT(it1.has_one_child(),
                      "combo_tree node should have exactly one child");
            ine1_child = it1.begin();
            is_strict1 = true;
        }
        // determine if ine1_child is a non-strict inequality
        else if(*it1==id::logical_not) {
            OC_ASSERT(it1.has_one_child(),
                      "combo_tree node should have exactly one child");
            pre_it it1_child = it1.begin();
            if(*it1_child==id::greater_than_zero) {
                OC_ASSERT(it1_child.has_one_child(),
                          "combo_tree child node should have exactly one child");
                ine1_child = it1_child.begin();
                is_strict1 = false;
            }
        }
        // determine if ine2_child is a strict inequality
        if(*it2==id::greater_than_zero) {
            OC_ASSERT(it2.has_one_child(),
                      "combo_tree node should have exactly one child");
            ine2_child = it2.begin();
            is_strict2 = true;
        }
        // determine if ine2_child is a non-strict inequality
        else if(*it2==id::logical_not) {
            OC_ASSERT(it2.has_one_child(),
                      "combo_tree node should have exactly one child");
            pre_it it2_child = it2.begin();
            if(*it2_child==id::greater_than_zero) {
                OC_ASSERT(it2_child.has_one_child(),
                          "combo_tree child node should have exactly one child");
                ine2_child = it2_child.begin();
                is_strict2 = false;
            }
        }
        // check if it1 and it2 are both inequalities
        if(tr.is_valid(ine1_child) && tr.is_valid(ine2_child)) {
            contin_t c1 = 0.0;
            contin_t c2 = 0.0;
            // tr1 will contain the non constant terms of inequality 1
            combo_tree tr1 = tr.subtree(ine1_child, tr.next_sibling(ine1_child));
            pre_it tr1_it = tr1.begin();
            // tr2 will contain the non constant terms of inequality 2
            combo_tree tr2 = tr.subtree(ine2_child, tr.next_sibling(ine2_child));
            pre_it tr2_it = tr2.begin();
            
            // the idea is that all constants under + will be
            // taken off from and tr1 and tr2 then tr1 and tr2
            // will be compared if there are equal (but
            // previously non strict equality will be
            // multiplied by -1 because non strict inequality
            // is represented using not(0<...) ) because of
            // this possible multiplication tr1 and tr2 will
            // be first reduced.
            
            // taken off constant terms from tr1 and put in c1
            // if non strict c1 is inverted because non strict
            // are represented with not(0<...)
            if(*tr1_it==id::plus) {
                for(sib_it sib1 = tr1_it.begin(); sib1 != tr1_it.end();) {
                    if(is_contin(*sib1)) {
                        c1 += (is_strict1? 1.0 : -1.0) * get_contin(*sib1);
                        sib1 = tr1.erase(sib1);
                    }
                    else ++sib1;
                }
            }
            // check if tr1 under the form -1*(x1+..+xn+c) and take off c
            else if(*tr1_it==id::times && tr1_it.number_of_children()==2) {
                pre_it child0 = tr1.child(pre_it(tr1_it), 0);
                pre_it child1 = tr1.child(pre_it(tr1_it), 1);
                // check if -1 in first child
                if(is_contin(*child0) && get_contin(*child0)==-1.0) {
                    // take off c
                    if(*child1==id::plus) {
                        for(sib_it s1p = child1.begin(); s1p != child1.end();)
                            if(is_contin(*s1p)) {
                                // this time c is not inverted
                                // if non strict because
                                // already inverted with
                                // -1*...
                                c1 += (is_strict1? -1.0 : 1.0) * get_contin(*s1p);
                                s1p = tr1.erase(s1p);
                            }
                            else ++s1p;
                    }
                }
                // check if -1 in the second child
                else if(is_contin(*child1) && get_contin(*child1)==-1.0) {
                    // take off c
                    if(*child0==id::plus) {
                        for(sib_it s1p = child0.begin(); s1p != child0.end();)
                            if(is_contin(*s1p)) {
                                // this time c is not inverted
                                // if non strict because
                                // already inverted with
                                // -1*...
                                c1 += (is_strict1? -1.0 : 1.0) * get_contin(*s1p);
                                s1p = tr1.erase(s1p);
                            }
                            else ++s1p;
                    }
                }
            }
            if(!is_strict1) // multiply tr1 by -1
                tr1.append_child(tr1.wrap(tr1_it, id::times), -1.0);
            
            // take off constant terms from tr2 and put in c2
            // if non strict c2 is inverted because non strict
            // are represented with not(0<...)
            if(*tr2_it==id::plus) {
                for(sib_it sib2 = tr2_it.begin(); sib2 != tr2_it.end();) {
                    if(is_contin(*sib2)) {
                        c2 += (is_strict2? 1.0 : -1.0) * get_contin(*sib2);
                        sib2 = tr2.erase(sib2);
                    }
                    else ++sib2;
                }
            }
            // check if tr2 under the form -1*(x1+..+xn+c) and take off c
            else if(*tr2_it==id::times && tr2_it.number_of_children()==2) {
                pre_it child0 = tr2.child(pre_it(tr2_it), 0);
                pre_it child1 = tr2.child(pre_it(tr2_it), 1);
                // check if -1 in first child
                if(is_contin(*child0) && get_contin(*child0)==-1.0) {
                    // take off c
                    if(*child1==id::plus) {
                        for(sib_it s2p = child1.begin(); s2p != child1.end();)
                            if(is_contin(*s2p)) {
                                // this time c is not inverted if non
                                // strict because already inverted with
                                // -1*...
                                c2 += (is_strict2? -1.0 : 1.0) * get_contin(*s2p);
                                s2p = tr2.erase(s2p);
                            }
                            else ++s2p;
                    }
                }
                // check if -1 in the second child
                else if(is_contin(*child1) && get_contin(*child1)==-1.0) {
                    // take off c
                    if(*child0==id::plus) {
                        for(sib_it s2p = child0.begin(); s2p != child0.end();)
                            if(is_contin(*s2p)) {
                                // this time c is not inverted if non
                                // strict because already inverted with
                                // -1*...
                                c2 += (is_strict2? -1.0 : 1.0) * get_contin(*s2p);
                                s2p = tr2.erase(s2p);
                            }
                            else ++s2p;
                    }
                }
            }
            if(!is_strict2) // multiply tr1 by -1
                tr1.append_child(tr2.wrap(tr2_it, id::times), -1.0);
            
            // reduce tr1 and tr2 and compare is equal
            (*_reduction)(tr1);
            (*_reduction)(tr2);
            
            if(tr1==tr2) {
                if(!is_strict1 && is_strict2)
                    return c1 < c2;
                else return c1 <= c2;
            }
            else return false;
        }
        // no equality, no inequalities
        else return false;
    }
}
        
// look up the assumptions and replace by true if present or false
// if not(assum) present
void reduce_from_assumptions::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if((get_output_type_tree(*it)==type_tree(id::boolean_type)
        && *it!=id::logical_true && *it!=id::logical_false)
       || is_argument(*it)) {
        sib_it assum = tr.next_sibling(tr.begin());
        if(tr.is_valid(assum)) {
            for(; assum != tr.end(); ++assum) {
                if(implies(tr, pre_it(assum), it)) {
                    *it = id::logical_true;
                    tr.erase_children(it);
                    return;
                }
                else if(*assum==id::logical_not) {
                    OC_ASSERT(assum.has_one_child(),
                              "combo_tree node should have exactly one child"
                              " (reduce_from_assumptions - assum).");
                    if(implies(tr, it, pre_it(assum.begin()))) {
                        *it = id::logical_false;
                        tr.erase_children(it);
                        return;
                    }
                }
                else if(is_argument(*assum) && is_argument(*it)) {
                    argument assum_arg = get_argument(*assum);
                    assum_arg.negate();
                    //don't need to treat the case when same idx because
                    //already treated in the beginning of the function
                    if(assum_arg.idx == get_argument(*it).idx) {
                        *it = id::logical_false;
                        tr.erase_children(it);
                        return;
                    }
                }
            }
        }
    }
}

// if(x y z) -> if(NOT(x) z y)  if |NOT(x)|<|x|
void reduce_contin_if_not::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::contin_if) {
        OC_ASSERT(tr.number_of_children(it)==3,
                  "combo_tree node should have exactly three children (reduce_contin_if_not).");
        pre_it cond = tr.child(it, 0);
        pre_it b1 = tr.child(it, 1);
        pre_it b2 = tr.child(it, 2);
        combo_tree cond_tr = tr.subtree(sib_it(cond), sib_it(b1));
        //copy old assumptions, begin
        sib_it bna = cond_tr.begin(); //before new assumption
        for(sib_it a = tr.next_sibling(tr.begin()); a != tr.end(); ++a)
            bna = cond_tr.insert_subtree_after(bna, a);
        //copy old assumptions, end
        pre_it cond_tr_it = cond_tr.begin();
        pre_it new_cond_it = cond_tr.prepend_child(cond_tr_it, *cond_tr_it);
        sib_it first_cond_child = cond_tr.next_sibling(sib_it(new_cond_it));
        *cond_tr_it = id::logical_not;
        cond_tr.reparent(new_cond_it, first_cond_child, cond_tr_it.end());
        (*_reduction)(cond_tr);
        (*_reduction)(tr, cond);
        if(cond_tr.size() < tr.subtree_size(cond)) {
            tr.replace(sib_it(cond), sib_it(b1),
                       sib_it(cond_tr.begin()), sib_it(cond_tr.end()));
            tr.swap(b1, b2);
        }
    }
}
        
// 0<sum x_i -> true    if 0<x_i -> true forall i
// 0<sum x_i -> false   if 0<x_i -> false forall i
// maybe TODO : 0<sum x_i -> true if exist i 0<x_i->true and forall other i 0<=x_i
void reduce_gt_zero_sum::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_gt_zero_sum).");
        pre_it it_child = it.begin();
        if(*it_child==id::plus) {
            OC_ASSERT(!it_child.is_childless(),
                      "combo_tree child node should have at least one child (reduce_gt_zero_sum).");
            bool res = true;// = true to get rid of stupid C++ wraning
            for(sib_it sib = it_child.begin(); sib != it_child.end(); ++sib) {
                combo_tree sib_tr = tr.subtree(sib, tr.next_sibling(sib));
                //copy old assumptions, begin
                sib_it bna = sib_tr.begin(); //before new assumption
                for(sib_it a = tr.next_sibling(tr.begin()); a != tr.end(); ++a)
                    bna = sib_tr.insert_subtree_after(bna, a);
                //copy old assumptions, end
                sib_tr.wrap(sib_tr.begin(), id::greater_than_zero);
                (*_reduction)(sib_tr);
                pre_it sib_tr_root = sib_tr.begin();
                if(is_boolean(*sib_tr_root)) {
                    if(tr.sibling_index(sib)==0)//getting the first reduction result as exemplar
                        res = vertex_to_bool(*sib_tr_root);
                    else if(res != vertex_to_bool(*sib_tr_root))
                        return;
                }
                else return;
            }
            *it = bool_to_vertex(res);
            tr.erase_children(it);
        }
    }
}

// 0<prod x_i -> 0<prod x_i    with x_i=1 if 0<x_i -> true
//                             with x_i=-1 if 0<-x_i -> true
// 0<prod x_i -> false     if exist x_i==0, 
//                            that is 0<x_i -> false and 0<-1*x_i -> false
void reduce_gt_zero_prod::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_gt_zero_prod).");
        pre_it it_child = it.begin();
        if(*it_child==id::times) {
            for(sib_it sib = it_child.begin(); sib != it_child.end();) {
                combo_tree sib_tr = tr.subtree(sib, tr.next_sibling(sib));
                //copy old assumptions, begin
                sib_it bna = sib_tr.begin(); //before new assumption
                for(sib_it a = tr.next_sibling(tr.begin()); a != tr.end(); ++a)
                    bna = sib_tr.insert_subtree_after(bna, a);
                //copy old assumptions, end
                sib_tr.wrap(sib_tr.begin(), id::greater_than_zero);
                //reduce 0<x_i
                combo_tree tr1 = sib_tr;
                (*_complete_reduction)(tr1);
                pre_it tr1_root = tr1.begin();
                if(*tr1_root==id::logical_true)
                    sib = tr.erase(sib);
                else {
                    //reduce 0<-1*x_i
                    combo_tree tr2 = sib_tr;
                    pre_it x_i = tr2.begin().begin();
                    tr2.append_child(tr2.wrap(x_i, id::times), -1.0);
                    //reduction without reduce_gt_zero_prod is used to avoid infinite
                    //recursion
                    (*_reduction_without_itself)(tr2);
                    pre_it tr2_root = tr2.begin();
                    if(*tr2_root==id::logical_true) {
                        *sib = -1.0;
                        tr.erase_children(sib);
                        ++sib;
                    }
                    //test if x_i==0.0
                    else if(*tr1_root==id::logical_false
                            && *tr2_root==id::logical_false) {
                        *it = id::logical_false;
                        tr.erase_children(it);
                        return;
                    }
                    else ++sib;
                }
            }
            if(it_child.is_childless()) {
                tr.erase_children(it);
                *it = id::logical_true;
            }
        }
    }
}

// 0<x/y -> 0<x    if 0<y -> true
// 0<x/y -> 0<y    if 0<x -> true
// 0<x/y -> 0<-1*y if 0<-1*x -> true
// 0<x/y -> 0<-1*x if 0<-1*y -> true
// 0<x/y -> false  if x==0, that is not(0<x) -> true and not(0<-x) -> true
void reduce_gt_zero_div::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_gt_zero_div).");
        pre_it it_child = it.begin();
        if(*it_child==id::div) {
            OC_ASSERT(it_child.number_of_children()==2,
                      "combo_tree child node should have exactly two children (reduce_gt_zero_div).");
            pre_it num = tr.child(it_child, 0);
            pre_it denom = tr.child(it_child, 1);
            //case of the numerator
            combo_tree num_tr = tr.subtree(sib_it(num), sib_it(denom));
            //copy old assumptions, begin
            sib_it bna = num_tr.begin(); //before new assumption
            for(sib_it a = tr.next_sibling(tr.begin()); a != tr.end(); ++a)
                bna = num_tr.insert_subtree_after(bna, a);
            //copy old assumptions, end
            num_tr.wrap(num_tr.begin(), id::greater_than_zero);
            //reduce 0<x
            combo_tree num_tr1 = num_tr;
            (*_reduction)(num_tr1);
            pre_it num_tr1_root = num_tr1.begin();
            if(*num_tr1_root==id::logical_true) {
                tr.erase(num);
                tr.erase(tr.flatten(it_child));
            }
            else {
                //reduce 0<-1*x
                combo_tree num_tr2 = num_tr;
                num_tr2.append_child(tr.wrap(num_tr2.begin().begin(), id::times),
                                     -1.0);
                (*_reduction)(num_tr2);
                pre_it num_tr2_root = num_tr2.begin();
                if(*num_tr2_root==id::logical_true) {
                    tr.erase_children(num);
                    *it_child = id::times;
                    *num = -1.0;
                }
                //test if x==0
                else if(*num_tr1_root==id::logical_false
                        && *num_tr2_root==id::logical_false) {
                    *it = id::logical_false;
                    tr.erase_children(it);
                }
                //case of denominateur
                else {
                    combo_tree denom_tr = tr.subtree(sib_it(denom), tr.next_sibling(denom));
                    //copy old assumptions, begin
                    pre_it bna = denom_tr.begin(); //before new assumption
                    for(sib_it a = tr.next_sibling(tr.begin()); a != tr.end(); ++a)
                        bna = denom_tr.insert_subtree_after(bna, a);
                    //copy old assumptions, end
                    denom_tr.wrap(denom_tr.begin(), id::greater_than_zero);
                    //reduce 0<y
                    combo_tree denom_tr1 = denom_tr;
                    (*_reduction)(denom_tr1);
                    pre_it denom_tr1_root = denom_tr1.begin();
                    if(*denom_tr1_root==id::logical_true) {
                        tr.erase(denom);
                        tr.erase(tr.flatten(it_child));
                    }
                    else {
                        //reduce 0<-1*y
                        combo_tree denom_tr2 = denom_tr;
                        denom_tr2.append_child(tr.wrap(denom_tr2.begin().begin(),
                                                       id::times), -1.0);
                        (*_reduction)(denom_tr2);
                        pre_it denom_tr2_root = denom_tr2.begin();
                        if(*denom_tr2_root==id::logical_true) {
                            tr.erase_children(denom);
                            *it_child = id::times;
                            *denom = -1.0;
                        }
                    }
                }
            }
        }
    }
}

// 0<x+sin(y) -> true  if 0<x-1 -> true
// 0<x+sin(y) -> false if 0<x+1 -> false
void reduce_gt_zero_sum_sin::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_gt_zero_sum_sin).");
        pre_it it_child = it.begin();
        if(*it_child==id::plus && tr.is_valid(it_child.find_child(id::sin))) {
            combo_tree copy_tr = tr.subtree(sib_it(it), tr.next_sibling(sib_it(it)));
            //copy old assumptions, begin
            sib_it bna = copy_tr.begin(); //before new assumption
            for(sib_it a = tr.next_sibling(tr.begin()); a != tr.end(); ++a)
                bna = copy_tr.insert_subtree_after(bna, a);
            //copy old assumptions, end
            pre_it copy_plus = copy_tr.begin().begin();
            int number_of_sin = 0;
            for(sib_it sib = copy_plus.begin(); sib != copy_plus.end();) {
                if(*sib==id::sin) {
                    number_of_sin++;
                    sib = copy_tr.erase(sib);
                }
                else ++sib;
            }
            if(copy_plus.is_childless())
                return;
            //check if 0<x+y..-number_of_sin -> true
            combo_tree copy1_tr = copy_tr;
            pre_it copy1_plus = copy1_tr.begin().begin();
            copy1_tr.append_child(copy1_plus, (contin_t)-number_of_sin);
            (*_reduction)(copy1_tr);
            if(*copy1_tr.begin()==id::logical_true) {
                *it = id::logical_true;
                tr.erase_children(it);
            }
            else {
                //check if 0<x+y..+number_of_sin -> false
                combo_tree copy2_tr = copy_tr;
                pre_it copy2_plus = copy2_tr.begin().begin();
                copy2_tr.append_child(copy2_plus, (contin_t)number_of_sin);
                (*_reduction)(copy2_tr);
                if(*copy2_tr.begin()==id::logical_false) {
                    *it = id::logical_false;
                    tr.erase_children(it);
                }
            }
        }
    }
}

// 0<sin(y) -> true  if 0<y -> true and 0<pi-y -> true
// 0<sin(y) -> false if 0<y -> false and 0<-(y+pi) -> false
void reduce_gt_zero_sin::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_gt_zero_sin).");
        pre_it it_child = it.begin();
        if(*it_child==id::sin) {
            OC_ASSERT(it_child.has_one_child(),
                      "combo_tree child node should have exactly one child (reduce_gt_zero_sin).");
            it_child.begin();
            combo_tree copy_tr = tr.subtree(sib_it(it), tr.next_sibling(sib_it(it)));
            //copy old assumptions, begin
            sib_it bna = copy_tr.begin(); //before new assumption
            for(sib_it a = tr.next_sibling(tr.begin()); a != tr.end(); ++a)
                bna = copy_tr.insert_subtree_after(bna, a);
            //copy old assumptions, end
            //turn copy_tr into 0<y and check if 0<y
            copy_tr.erase(copy_tr.flatten(copy_tr.begin().begin()));
            (*_reduction)(copy_tr);
            if(*copy_tr.begin()==id::logical_true) {
                //check if 0<pi-y -> true
                combo_tree copy1_tr = tr.subtree(sib_it(it), tr.next_sibling(sib_it(it)));
                //copy old assumptions, begin
                sib_it bna = copy1_tr.begin(); //before new assumption
                for(sib_it a = tr.next_sibling(tr.begin()); a != tr.end(); ++a)
                    bna = copy1_tr.insert_subtree_after(bna, a);
                //copy old assumptions, end
                pre_it sin1_it = copy1_tr.begin().begin();
                *sin1_it = id::times;
                copy1_tr.append_child(sin1_it, -1.0);
                pre_it new_plus = copy1_tr.wrap(sin1_it, id::plus);
                copy1_tr.append_child(new_plus, (contin_t)PI);
                (*_reduction)(copy1_tr);
                if(*copy1_tr.begin()==id::logical_true) {
                    *it = id::logical_true;
                    tr.erase_children(it);
                }
            }
            else if(*copy_tr.begin()==id::logical_false) {
                //check if 0<-(y+pi) -> false //TODO
                combo_tree copy2_tr = tr.subtree(sib_it(it), tr.next_sibling(sib_it(it)));
                //copy old assumptions, begin
                sib_it bna = copy2_tr.begin(); //before new assumption
                for(sib_it a = tr.next_sibling(tr.begin()); a != tr.end(); ++a)
                    bna = copy2_tr.insert_subtree_after(bna, a);
                //copy old assumptions, end
                pre_it sin2_it = copy2_tr.begin().begin();
                *sin2_it = id::plus;
                copy2_tr.append_child(sin2_it, (contin_t)PI);
                copy2_tr.append_child(copy2_tr.wrap(sin2_it, id::times), -1.0);
                (*_reduction)(copy2_tr);
                if(*copy2_tr.begin()==id::logical_false) {
                    *it = id::logical_false;
                    tr.erase_children(it);
                }
            }
        }
    }
}

  
contin_t reduce_inequality_from_assumptions::splitCoefExpression(combo_tree& tr, combo_tree::iterator& it) const
{
    contin_t coef = 1.0;
    if(*it==id::times) {
        for(sib_it sib = it.begin(); sib != it.end();) {
            if(is_contin(*sib)) {
                coef = coef * get_contin(*sib);
                sib = tr.erase(sib);
            }
            else ++sib;
        }
        if(it.has_one_child())
            it = tr.erase(tr.flatten(it));
        else if(it.is_childless()) {
            tr.erase(it);
            it = tr.end(); //to be recognized as not valid
        }
    }
    else if(is_contin(*it)) {
        coef = get_contin(*it);
        tr.erase(it);
        it = tr.end(); //to be recognized as not valid
    }
    return coef;
}

bool reduce_inequality_from_assumptions::gaussJordanElimination(double_matrix& dm, double eps) const
{
    unsigned int h = dm.size(); //number of rows
    OC_ASSERT(h>0, "Number of rows should be greater than 0 (gaussJordanElimitation)");
    unsigned int w = dm[h-1].size(); //number of columns
    for(unsigned int y = 0; y < h; ++y) {
        unsigned int maxrow = y;
        //find max pivot
        for(unsigned int y2 = y+1; y2 < h; ++y2)
            if(std::fabs(dm[y2][y]) > std::fabs(dm[maxrow][y]))
                maxrow = y2;
        std::vector<double> tmp_row = dm[y];
        dm[y] = dm[maxrow];
        dm[maxrow] = tmp_row;
        //check if singular
        if(std::fabs(dm[y][y]) <= eps)
            return false;
        //eliminate column y
        for(unsigned int y2 = y+1; y2 < h; ++y2) {
            double c = dm[y2][y]/dm[y][y];
            for(unsigned int x = y; x < w; ++x)
                dm[y2][x] -= c*dm[y][x];
        }
    }
    //backsubstitute
    for(int y = h-1; y >= 0; --y) {
        double c = dm[y][y];
        for(int y2 = 0; y2 < y; ++y2)
            for(int x = w-1; x > y-1; --x)
                dm[y2][x] -= dm[y][x]*dm[y2][y]/c;
        dm[y][y] /= c;
        for(unsigned x = (h>w?w:h); x < (h>w?h:w); ++x)
            dm[y][x] /= c;
    }
    return true;
}

void reduce_inequality_from_assumptions::gaussianElimination(double_matrix& dm) const
{
    unsigned int m = dm.size(); //number of rows
    OC_ASSERT(m>0, "Number of rows should be greater than 0 (gaussianElimitation)");
    unsigned int n = dm[m-1].size(); //number of columns
    for(unsigned int i = 0; i < m; ++i) {
        for(unsigned int j = 0; j < n; ++j) {
            unsigned int maxi = i;
            for(unsigned int k = i+1; k < m; ++k) {
                if(std::fabs(dm[k][j])>fabs(dm[maxi][j]))
                    maxi = k;
            }
            if(dm[maxi][j] != 0.0) {
                std::vector<double> tmp_row = dm[i];
                dm[i] = dm[maxi];
                dm[maxi] = tmp_row;
                double c = dm[i][j];
                OC_ASSERT(c != 0.0, "dm[%d][%d] == 0.0. Should be diferent", i, j);
                for(unsigned int k = 0; k < n; ++k)
                    dm[i][k] = dm[i][k] / c;
                for(unsigned int u = i+1; u < m; ++u) {
                    double d = dm[u][j];
                    for(unsigned int k = 0; k < n; ++k)
                        dm[u][k] = dm[u][k] - d*dm[i][k];
                }
            }
        }
    }
}

bool reduce_inequality_from_assumptions::gaussianElimination2(double_matrix& dm, double eps) const
{
    int m = dm.size(); //number of rows
    OC_ASSERT(m>0, "Number of rows should be greater than 0 (gaussianElimitation2)");
    int n = dm[m-1].size(); //number of columns
    OC_ASSERT(n>0, "Number of columns should be greater than 0 (gaussianElimitation2)");
    for(int i = 0; i < std::min(m, n-1); ++i) {
        //swap below until find non-zero
        int ipivot = -1;
        for(int i2  = i; i2 < m; ++i2)
            if(std::fabs(dm[i2][i]) > eps)
                ipivot = i2;
        //swap row i and row
        if(ipivot==-1)
            return false;
        std::vector<contin_t> tmp = dm[i];
        dm[i] = dm[ipivot];
        dm[ipivot] = tmp;
        //divide pivot by itself to be 1
        contin_t c = dm[i][i];
        for(int j = 0; j < n; ++j)
            dm[i][j] /= c;
        //reduce to 0 all column below
        for(int i2 = i+1; i2 < m; ++i2) {
            contin_t c = dm[i2][i];
            for(int j = 0; j < n; ++j)
                dm[i2][j] -= c*dm[i][j];
        }
    }
    return true;
}

bool reduce_inequality_from_assumptions::backSubstitution(double_matrix& dm, std::vector<double>& solution, double eps) const
{
    OC_ASSERT(solution.empty(), "There should not be any previous solution (backSubstitution)");
    int m = dm.size(); //number of rows
    OC_ASSERT(m>0, "Number of rows should be greater than 0 (backSubstitution)");
    int n = dm[m-1].size(); //number of columns
    OC_ASSERT(n>0, "Number of rows should be greater than 0 (backSubstitution)");
    solution.resize(n-1, 0.0);
    bool allNull = true;
    for(int i = m-1; i >= 0; --i) {
        //check if there is contradiction in the equationnal system
        for(int j = 0; j < n-1; ++j)
            if(std::fabs(dm[i][j]) > eps) 
                allNull = false;
        if(allNull && std::fabs(dm[i][n-1]) > eps) //something in contradiction
            return false;
        for(int j = 0; j < n-1; ++j) {	
            if(dm[i][j]==1.0) {
                solution[j] = dm[i][n-1];
                for(int i2 = i-1; i2 >= 0; --i2) {
                    dm[i2][n-1] -= solution[j]*dm[i2][j];
                    dm[i2][j] = 0;
                }
            }
        }
    }
    return true;
}

void reduce_inequality_from_assumptions::operator()
    (combo_tree& tr,combo_tree::iterator it) const
{
    // std::cout << "LINEAR BEFORE : " << tr << " IT : " << *it << std::endl;
    // associate a subtree and row in the matrix to solve
    // the size of a row is |assumptions|+1
    // the last column of the matrix contains the coef associated with its
    // expression
    typedef std::map<pre_it, std::vector<contin_t>, opencog::lexicographic_subtree_order<vertex> > subtree_row;
    typedef subtree_row::iterator subtree_row_it;
    typedef subtree_row::const_iterator subtree_row_const_it;
    //true if the main term is strictly positive false otherwise
    bool termStrictPositive = true;// = true just to get rid for the stupid C++ warning
    //that termStrictPositive might be used undefined
    //we make a copy of tr because the reductor is going
    //manipulate/change it to find expression/variables.
    combo_tree ctr = tr;
    //main term to find the linear decomposition
    pre_it term_it;
    //check if strict positive inequality or negative or null
    if(*it==id::greater_than_zero) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_inequality_from_assumptions).");
        term_it = ctr.begin().begin();
        termStrictPositive = true;
    }
    else if(*it==id::logical_not) {
        OC_ASSERT(it.has_one_child(),
                  "combo_tree node should have exactly one child (reduce_inequality_from_assumptions).");
        pre_it it_child = it.begin();
        if(*it_child==id::greater_than_zero) {
            OC_ASSERT(it_child.has_one_child(),
                      "combo_tree child node should have exactly one child (reduce_inequality_from_assumptions).");
            term_it = ctr.begin().begin().begin();
            termStrictPositive = false;
        }
    }
    //check if the main term is an inequality (positive or negative)
    if(ctr.is_valid(term_it)) {
        sib_it assum = ctr.next_sibling(ctr.begin());
        if(ctr.is_valid(assum)) {
            //select the set of inequality assumptions
            //if this set is empty then stop the reduction
            std::vector<pre_it> assumptions;
            //define a boolean vector such that
            //has ther size of number of assumptions + 1 (for the main term)
            //(the main term is at the last position
            //if an entry if true it means that the assumption (or the last term)
            //is strictly positive
            //otherwise it means that it is negative or null
            std::vector<bool> isStrictPositives;
            for(; assum != ctr.end(); ++assum) {
                if(*assum==id::greater_than_zero) {
                    assumptions.push_back(pre_it(assum));
                    isStrictPositives.push_back(true);
                }
                else if(*assum==id::logical_not) {
                    OC_ASSERT(assum.has_one_child(),
                              "combo_tree node should have exactly one child (reduce_inequality_from_assumptions - assum).");
                    pre_it assum_child = assum.begin();
                    if(*assum_child==id::greater_than_zero) {
                        assumptions.push_back(assum_child);
                        isStrictPositives.push_back(false);
                    }
                }
            }
            //the last entry of true because a constant is consider
            //strictly positive of a term 1
            isStrictPositives.push_back(true);
            if(!assumptions.empty()) {
                subtree_row sr;
                //coefs alone without term
                std::vector<contin_t> free_coefs(assumptions.size()+2, 0.0);
                //for the constant term in the linear combination
                free_coefs[assumptions.size()] = 1.0;
                //this reduction only handle linear combination
                //that's why the case when term_it is * is not treated
                //if the main term is c*+(...) with c positive
                //then c would be eliminated by previous reduction rules
                //however if c is negative it is then replaced by -1
                //so the only case that is treated when term_it is * is
                //-1*x
                contin_t main_coef = 1.0; //to keep in mind that they were -1*term_it
                if(*term_it==id::times && term_it.number_of_children()==2) {
                    pre_it child0 = ctr.child(pre_it(term_it), 0);
                    pre_it child1 = ctr.child(pre_it(term_it), 1);
                    if(is_contin(*child0) && get_contin(*child0)==-1.0) {
                        term_it = child1;
                        main_coef = -1.0;
                    }
                    else if(is_contin(*child1) && get_contin(*child1)==-1.0) {
                        term_it = child0;
                        main_coef = -1.0;
                    }
                }
                //std::cout << "MAIN COEF : " << main_coef << std::endl;
                if(*term_it==id::plus) {
                    for(sib_it sib = term_it.begin(); sib != term_it.end();) {
                        //identify paires coef*expression
                        pre_it expression = sib;
                        sib_it next_sib = ctr.next_sibling(sib);
                        contin_t coef = splitCoefExpression(ctr, expression) * main_coef;
                        sib = next_sib;
                        //add or uptdate (expression, coef) in the map
                        if(ctr.is_valid(expression)) {
                            subtree_row_it sr_it = sr.find(expression);
                            if(sr_it==sr.end()) {
                                std::vector<contin_t> v(assumptions.size()+1, 0.0);
                                v.push_back(coef);
                                std::pair<pre_it, std::vector<contin_t> > p(expression, v);
                                sr.insert(p);
                            }
                            else { //update coef
                                std::vector<contin_t>& v_sr = sr_it->second;
                                v_sr[assumptions.size()+1] = v_sr[assumptions.size()+1]+coef;
                            }
                        }
                        //add coef to the last entry of free_coefs
                        else free_coefs[assumptions.size()+1] 
                                 = free_coefs[assumptions.size()+1] + coef;
                        //print map
                        /*for(subtree_row_it sr_it = sr.begin(); sr_it != sr.end(); ++sr_it) {
                          std::cout << "MAIN VAR : " << *sr_it->first << " ";
                          for(std::vector<double>::iterator i = (sr_it->second).begin();
                          i != (sr_it->second).end(); ++i) {
                          std::cout << *i << " ";
                          }
                          std::cout << std::endl;
                          }*/
                    }
                }
                //main term is not a sum
                else {
                    //identify paires coef*expression
                    pre_it expression = term_it;
                    contin_t coef = splitCoefExpression(ctr, expression) * main_coef;
                    if(ctr.is_valid(expression)) {
                        std::vector<contin_t> v(assumptions.size()+1, 0.0);
                        v.push_back(coef);
                        std::pair<pre_it, std::vector<contin_t> > p(expression, v);
                        sr.insert(p);
                    }
                    //add coef to the last entry of free_coefs
                    else free_coefs[assumptions.size()+1] 
                             = free_coefs[assumptions.size()+1] + coef;
                }
                //complete sr and free_coefs with the assumptions
                //either by adding new expressions not found in ctr and coef 0.0
                //or by filling the matrix
                for(unsigned int i = 0; i < assumptions.size(); ++i) {
                    pre_it assum = assumptions[i];
                    OC_ASSERT(*assum==id::greater_than_zero && assum.has_one_child(),
                              "Assum diferent from greater_than_zero or has none or more than one child.");
                    pre_it assum_child = assum.begin();
                    //same remark as previously about -1*x
                    contin_t assum_coef = 1.0;
                    if(*assum_child==id::times && assum_child.number_of_children()==2){
                        pre_it child0 = ctr.child(pre_it(assum_child), 0);
                        pre_it child1 = ctr.child(pre_it(assum_child), 1);
                        if(is_contin(*child0) && get_contin(*child0)==-1.0) {
                            assum_child = child1;
                            assum_coef = -1.0;
                        }
                        else if(is_contin(*child1) && get_contin(*child1)==-1.0) {
                            assum_child = child0;
                            assum_coef = -1.0;
                        }
                    }
                    if(*assum_child==id::plus) {
                        for(sib_it pl_sib = assum_child.begin();
                            pl_sib != assum_child.end();) {
                            //identify paires coef*expression
                            pre_it expression = pl_sib;
                            sib_it next_pl_sib = ctr.next_sibling(pl_sib);
                            contin_t coef = 
                                splitCoefExpression(ctr, expression) * assum_coef;
                            pl_sib = next_pl_sib;
                            //add or uptdate (expression, rows) in the map
                            if(ctr.is_valid(expression)) {
                                subtree_row_it sr_it = sr.find(expression);
                                if(sr_it==sr.end()) {
                                    std::vector<contin_t> v(assumptions.size()+2, 0.0);
                                    v[i] = coef;
                                    std::pair<pre_it, std::vector<contin_t> >
                                        p(expression, v);
                                    sr.insert(p);
                                }
                                else { //update coef
                                    std::vector<contin_t>& v_sr = sr_it->second;
                                    v_sr[i] = v_sr[i] + coef;
                                }
                            }
                            //add coef to free_coefs
                            else free_coefs[i] = free_coefs[i] + coef;
                        }
                        /*for(subtree_row_it sr_it = sr.begin(); sr_it != sr.end(); ++sr_it) {
                          std::cout << "ASSUM VAR : " << *sr_it->first << " ";
                          for(std::vector<double>::iterator i = (sr_it->second).begin();
                          i != (sr_it->second).end(); ++i) {
                          std::cout << *i << " ";
                          }
                          std::cout << std::endl;
                          }*/
                    }
                    else { //no id::plus
                        //identify paires coef*expression
                        pre_it expression = assum_child;
                        contin_t coef =
                            splitCoefExpression(ctr, expression) * assum_coef;
                        //add or uptdate (expression, rows) in the map
                        if(ctr.is_valid(expression)) {
                            subtree_row_it sr_it = sr.find(expression);
                            if(sr_it==sr.end()) {
                                std::vector<contin_t> v(assumptions.size()+2, 0.0);
                                v[i] = coef;
                                std::pair<pre_it, std::vector<contin_t> > p(expression, v);
                                sr.insert(p);
                            }
                            else { //update coef
                                std::vector<contin_t>& v_sr = sr_it->second;
                                v_sr[i] = v_sr[i] + coef;
                            }
                        }
                        //add coef to free_coefs
                        else free_coefs[i] = free_coefs[i] + coef;
                    }  
                }
                //generate matrix to be solved
                std::vector< std::vector<contin_t> > mat;
                for(subtree_row_const_it sci = sr.begin(); sci != sr.end(); ++sci)
                    mat.push_back(sci->second);
                mat.push_back(free_coefs);
                //print matrix before
                /*std::cout << "MATRIX BEFORE :" << std::endl;
                  for(unsigned int i = 0; i < mat.size(); ++i) {
                  for(unsigned int j = 0; j < mat[i].size(); ++j)
                  std::cout << mat[i][j] << " ";
                  std::cout << std::endl;
                  }*/
                //perform Gaussian elimination, if fail return
                bool resGJ = gaussianElimination2(mat);
                //print matrix after
                /*std::cout << "MATRIX AFTER :" << std::endl;
                  for(unsigned int i = 0; i < mat.size(); ++i) {
                  for(unsigned int j = 0; j < mat[i].size(); ++j)
                  std::cout << mat[i][j] << " ";
                  std::cout << std::endl;
                  }*/
                //non linear decomposition
                if(!resGJ) {
                    return;
                }
                //check if the linear decomposition is positive or negative
                std::vector<contin_t> solution;
                bool BSres = backSubstitution(mat, solution);
                //print vector solution
                /*std::cout << "SOLUTION :" << std::endl;
                  for(unsigned int i = 0; i < solution.size(); ++i)
                  std::cout << solution[i] << " ";
                  std::cout << std::endl;
                  //print vector isStrictPositives
                  std::cout << "ISSTRICTPOSITIVES :" << std::endl;
                  for(unsigned int i = 0; i < isStrictPositives.size(); ++i)
                  std::cout << (bool)isStrictPositives[i] << " ";
                  std::cout << std::endl;*/
                if(!BSres) {
                    return;
                }
                bool existsStrictPositive = false;
                bool existsStrictFromStrictPositive = false;
                bool allPositiveOrNull = true;
                for(unsigned int i = 0; i < solution.size(); ++i) {
                    if(isStrictPositives[i]) {
                        if(solution[i] > 0) {
                            existsStrictPositive = true;
                            existsStrictFromStrictPositive = true;
                        }
                        else if(solution[i] < 0) {
                            allPositiveOrNull = false;
                        }
                    }
                    else {
                        if(solution[i] < 0)
                            existsStrictPositive = true;
                        else if(solution[i] > 0)
                            allPositiveOrNull = false;
                    }
                }
                /*std::cout << "existsStrictPositive : " << existsStrictPositive << std::endl;
                  std::cout << "existsStrictFromStrictPositive : " << existsStrictFromStrictPositive << std::endl;
                  std::cout << "allPositiveOrNull : " << allPositiveOrNull << std::endl;*/
                //positive reduction
                if(allPositiveOrNull && existsStrictFromStrictPositive) {
                    //std::cout << "POSITIVE REDUCTION" << std::endl;
                    *it = (termStrictPositive? id::logical_true : id::logical_false);
                    tr.erase_children(it);
                }
                //reduction to false
                else if(!existsStrictPositive) {
                    //std::cout << "NEGATIVE REDUCTION" << std::endl;
                    *it = (termStrictPositive? id::logical_false : id::logical_true);
                    tr.erase_children(it);	      
                }
            }
        }
    }
    //std::cout << "LINEAR AFTER : " << tr << " IT : " << *it << std::endl;
}

} // ~namespace reduct
} // ~namespace opencog
