/*
 * opencog/comboreduct/reduct/contin_rules.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2013 Poulin Holdings LLC
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
#include <map>
#include <opencog/util/exceptions.h>
#include <opencog/comboreduct/type_checker/type_tree.h>
#include "contin_rules.h"

namespace opencog { namespace reduct {

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::pre_order_iterator pre_it;

// x+0 -> x
// x*NAN -> NAN
void reduce_plus_zero::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it != id::plus)
        return;

    for (sib_it sib = it.begin(); sib != it.end();) {
        if (is_contin(*sib)) {
            contin_t c = get_contin(*sib);
            if (0.0 == c) {
                sib = tr.erase(sib);
            }
            else if (not isfinite(c)) {
                tr.erase_children(it);
                *it = FP_NAN;
                return;
            }
            else ++sib;
        }
        else ++sib;
    }
    if (it.is_childless())
        *it = 0.0;
}

// x*1 -> x
// x*0 -> 0
// x*NAN -> NAN
void reduce_times_one_zero::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it != id::times)
        return;
    for (sib_it sib = it.begin(); sib != it.end();) {
        if (is_contin(*sib)) {
            contin_t c = get_contin(*sib);
            if (c == 1.0) {
                sib = tr.erase(sib);
            }
            else if (c == 0.0) {
                tr.erase_children(it);
                *it = 0.0;
                return;
            }
            else if (not isfinite(c)) {
                tr.erase_children(it);
                *it = FP_INFINITE;
                return;
            }
            else ++sib;
        }
        else ++sib;
    }
    if (*it == id::times && it.is_childless())
        *it = 1.0;
}

//x/z+y/z -> (x+y)/z
//or more generally, sum 1/z*x_i + sum y_j -> 1/z*(sum x_i) + sum y_j
//when sevelar choices are possible the chosen one :
//1)is the one that shorten the most the expression
//2)if not unique, the lowest one according to the index order
void reduce_factorize_fraction::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    //equiv_subtrees is a set of equal subtrees
    //the first element of the pair contains the size of one examplar
    //the second element contains a vector of all iterators corresponding
    //to equal subtrees to the examplar excluding the examplar.
    typedef std::vector<pre_it> pre_it_vector;
    typedef pre_it_vector::iterator pre_it_vector_it;
    typedef std::pair<int, pre_it_vector> equiv_subtrees;
    //associated an examplar subtree to its size and
    //and all equal subtrees
    typedef std::map<pre_it, equiv_subtrees,
                     lexicographic_subtree_order<vertex>
                     > subtree_partition;
    typedef subtree_partition::iterator subtree_partition_it;
    if(*it==id::plus) {
        subtree_partition sp;
        OC_ASSERT(!it.is_childless(), 
                  "combo_tree node should not be childless (reduce_factorize_fraction)." );
        pre_it chosen_factor = tr.end();
        int reduced_size = 0; //size reduced when the factor is chosen_factor
        //------------------------------------
        //fill sp, create a partition of equal
        //subtrees and find the best factor
        //------------------------------------
        for(sib_it plus_child = it.begin(); plus_child != it.end();
            ++plus_child) {
            if(*plus_child==id::div) {
                OC_ASSERT(plus_child.number_of_children()==2, 
                          "combo_tree child node should have exactly two children (reduce_factorize_fraction)." );
                pre_it denom = tr.child(plus_child, 1);
                if(*denom==id::times) {
                    for(sib_it times_child = denom.begin();
                        times_child != denom.end(); ++times_child) {
                        pre_it pf = times_child; //potential factor
                        //1.0 is never a potential factor
                        //this is to avoid endless increasing size of the expression
                        if(is_contin(*pf) && get_contin(*pf)==1.0)
                            continue;
                        
                        subtree_partition_it spit = sp.find(pf);
                        if(spit == sp.end()) {
                            pre_it_vector ev;
                            equiv_subtrees es(tr.subtree_size(pf), ev);
                            std::pair<pre_it, equiv_subtrees> p(pf, es);
                            sp.insert(p);
                        }
                        else {
                            pre_it key = spit->first;
                            equiv_subtrees& es = spit->second;
                            int es_subtree_size = es.first;
                            pre_it_vector& es_vec = es.second;
                            pre_it last_it;
                            if(es_vec.empty())
                                last_it = key;
                            else last_it = es_vec.back();
                            //check that it is not the same parent to avoid factorizing x*x
                            pre_it last_it_parent = tr.parent(last_it);
                            OC_ASSERT(tr.is_valid(last_it_parent), 
                                      "last combo_tree node invalid (reduce_factorize_fraction).");
                            if(last_it_parent != denom) {
                                //potential reduced size
                                es_vec.push_back(pf);
                                int prs = es_vec.size()*es_subtree_size;
                                if(prs>reduced_size) {
                                    reduced_size = prs;
                                    chosen_factor = pf;
                                }
                            }
                        }
                    }
                }
                else {
                    pre_it pf = denom; //potential factor
                    //1.0 is never a potential factor
                    //this is to avoid endless increasing size of the expression
                    if(is_contin(*pf) && get_contin(*pf)==1.0)
                        break;
                    
                    subtree_partition_it spit = sp.find(pf);
                    if(spit == sp.end()) {
                        pre_it_vector ev;
                        equiv_subtrees es(tr.subtree_size(pf), ev);
                        std::pair<pre_it, equiv_subtrees> p(pf, es);
                        sp.insert(p);
                    }
                    else {
                        equiv_subtrees& es = spit->second;
                        int es_subtree_size = es.first;
                        pre_it_vector& es_vec = es.second;
                        es_vec.push_back(pf);
                        //potential reduced size
                        int prs = es_vec.size()*es_subtree_size;
                        if(prs>reduced_size) {
                            reduced_size = prs;
                            chosen_factor = pf;
                        }
                    }
                }
            }
        }
        //---------
        //factorize
        //---------
        //find the vector of iterators to factorize
        subtree_partition_it spit = sp.find(chosen_factor);
        if (spit != sp.end())
        { //check if there is something to factorize
            pre_it key = spit->first;
            equiv_subtrees es = spit->second;
            pre_it_vector es_vec = es.second;
            // pre_it first_one = tr.end(); //count number of copies of factor
            // when alone
            if(!es_vec.empty())
            { //check if there is something to factorize
                //------------
                //move the key
                //------------
                pre_it key_parent = tr.parent(key);
                OC_ASSERT(tr.is_valid(key_parent),
                          "key parent node is invalid (reduce_factorize_fraction).");
                pre_it kp_parent = tr.parent(key_parent);
                OC_ASSERT(tr.is_valid(kp_parent),
                          "kp parent node is invalid (reduce_factorize_fraction).");
                pre_it main_div = tr.end();
                pre_it remain_plus = tr.end();
                //move the key
                if(*key_parent==id::div) {
                    main_div = key_parent;
                    remain_plus = tr.wrap(tr.child(main_div, 0), id::plus);
                }
                else {
                    main_div = tr.append_child(it, id::div);
                    remain_plus = tr.append_child(main_div, id::plus);
                    tr.move_ontop(tr.append_child(main_div), key);
                    if(key_parent.is_childless()) {
                        tr.move_ontop(tr.append_child(remain_plus),
                                      pre_it(kp_parent.begin()));
                        tr.erase(kp_parent);
                    }
                    else tr.move_ontop(tr.append_child(remain_plus), kp_parent);
                }
                //---------------------------------------------------
                //fusion with the chosen factor the elements equal to
                //---------------------------------------------------
                for(pre_it_vector_it factor_it =  es_vec.begin();
                    factor_it != es_vec.end(); ++factor_it) {
                    pre_it factor = *factor_it;
                    pre_it factor_parent = tr.parent(factor);
                    OC_ASSERT(tr.is_valid(factor_parent), 
                              "factor parent node is invalid (reduce_factorize_fraction).");
                    pre_it fp_parent = tr.parent(factor_parent);
                    OC_ASSERT(tr.is_valid(fp_parent), 
                              "fp parent node is invalid (reduce_factorize_fraction).");
                    tr.erase(factor);
                    if(*factor_parent==id::div) {
                        pre_it new_loc = tr.append_child(remain_plus);
                        tr.move_ontop(new_loc, pre_it(factor_parent.begin()));
                        tr.erase(factor_parent);
                    }
                    else {
                        if(factor_parent.is_childless()) {
                            tr.move_ontop(tr.append_child(remain_plus),
                                          pre_it(fp_parent.begin()));
                            tr.erase(fp_parent);
                        }
                        else tr.move_ontop(tr.append_child(remain_plus), fp_parent);
                    }
                }
            }
        }
    }
}
    
//x*y+x*z -> x(y+z)
//or more generally, sum x*y_i + sum z_j -> x*(sum y_i) + sum z_j
//when sevelar choices are possible the chosen one :
//1)is the one that shorten the most the expression
//2)if not unique, the lowest one according to the index order
//Note : if x is a numerator of div, it works too
void reduce_factorize::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    //equiv_subtrees is a set of equal subtrees
    //the first element of the pair contains the size of one exemplar
    //the second element contains a vector of all iterators corresponding
    //to equal subtrees to the exemplar excluding the exemplar.
    typedef std::vector<pre_it> pre_it_vector;
    typedef pre_it_vector::iterator pre_it_vector_it;
    typedef std::pair<int, pre_it_vector> equiv_subtrees;
    //associated an exemplar subtree to its size and
    //and all equal subtrees
    typedef std::map<pre_it, equiv_subtrees,
                     lexicographic_subtree_order<vertex>
                     > subtree_partition;
    typedef subtree_partition::iterator subtree_partition_it;
    if(*it==id::plus) {
        subtree_partition sp;
        OC_ASSERT(!it.is_childless(), "combo_tree node should not be childless");
        pre_it chosen_factor = tr.end();
        int reduced_size = 0; //size reduced when the factor is chosen_factor
        //------------------------------------
        //fill sp, create a partition of equal
        //subtrees and find the best factor
        //------------------------------------
        for(sib_it plus_child = it.begin(); plus_child != it.end();
            ++plus_child) {
            //pos = position where to look at to find potential factor
            pre_it pos = *plus_child==id::div? tr.child(plus_child, 0):plus_child;
            if(*pos==id::times) {
                for(sib_it times_child = pos.begin();
                    times_child != pos.end(); ++times_child) {
                    pre_it pf = times_child; //potential factor
                    //1.0 is never a potential factor
                    //this is to avoid endless increasing size of the expression
                    if(is_contin(*pf) && get_contin(*pf)==1.0)
                        continue;
                    
                    subtree_partition_it spit = sp.find(pf);
                    if(spit == sp.end()) { // new potential factor
                        equiv_subtrees es(tr.subtree_size(pf), pre_it_vector());
                        sp.insert(make_pair(pf, es));
                    }
                    else { // existing potential factor
                        pre_it key = spit->first;
                        equiv_subtrees& es = spit->second;
                        int es_subtree_size = es.first;
                        pre_it_vector& es_vec = es.second;
                        pre_it last_it;
                        if(es_vec.empty())
                            last_it = key;
                        else last_it = es_vec.back();
                        //check that it is not the same parent to avoid factorizing x*x
                        pre_it last_it_parent = tr.parent(last_it);
                        OC_ASSERT(tr.is_valid(last_it_parent), 
                                  "last combo_tree node is invalid");
                        if(last_it_parent != pos) {
                            //potential reduced size
                            es_vec.push_back(pf);
                            int prs = es_vec.size()*es_subtree_size;
                            if(prs>reduced_size) {
                                reduced_size = prs;
                                chosen_factor = pf;
                            }
                        }
                    }
                }
            }
            else {
                pre_it pf = pos; //potential factor
                //1.0 is never a potential factor
                //this is to avoid endless increasing size of the expression
                if(is_contin(*pf) && get_contin(*pf)==1.0)
                    break;
                
                subtree_partition_it spit = sp.find(pf);
                if(spit == sp.end()) { // new potential factor
                    equiv_subtrees es(tr.subtree_size(pf), pre_it_vector());
                    sp.insert(make_pair(pf, es));
                }
                else { // existing potential factor
                    equiv_subtrees& es = spit->second;
                    int es_subtree_size = es.first;
                    pre_it_vector& es_vec = es.second;
                    es_vec.push_back(pf);
                    //potential reduced size
                    int prs = es_vec.size()*es_subtree_size;
                    if(prs>reduced_size) {
                        reduced_size = prs;
                        chosen_factor = pf;
                    }
                }
            }
        }
        //---------
        //factorize
        //---------
        //find the vector of iterators to factorize
        subtree_partition_it spit = sp.find(chosen_factor);
        if(spit != sp.end()) { //check if there is something to factorize
            pre_it key = spit->first;
            equiv_subtrees es = spit->second;
            pre_it_vector es_vec = es.second;
            pre_it first_one = tr.end(); //count number of copies of factor
            //when alone
            if(!es_vec.empty()) { //check if there is something to factorize
                //------------
                //move the key
                //------------
                //move the chosen factor under the top plus, if not already
                //then insert a '*' above it and a '+' beside it to put
                //the remaining part of what is factorized
                pre_it key_parent = tr.parent(key);
                OC_ASSERT(tr.is_valid(key_parent),
                          "key parent node is invalid (reduce_factorize).");
                pre_it kp_parent = tr.parent(key_parent);
                pre_it key_div = tr.end(); //position of div if key is under
                //determine key_div
                if(*key_parent==id::div)
                    key_div = key_parent;
                else if(tr.is_valid(kp_parent) && *kp_parent==id::div)
                    key_div = kp_parent;
                //if key is not already on place move it
                if(key_parent!=it)
                    tr.move_ontop(tr.append_child(it), key);
                tr.wrap(key, id::times);
                pre_it remain_plus = tr.insert_after(key, id::plus);
                //if after moving key key_parent is childless then
                //the operator pointed by key_parent must be erased
                //and key_parent must point to its parent
                if(key_parent.is_childless()) {
                    tr.erase(key_parent);
                    key_parent = kp_parent;
                }
                //if key was under div then 1.0 must be inserted where key was
                if(key_parent==key_div)
                    tr.prepend_child(key_parent, 1.0);
                //move the remainder of the key prod or put 1.0 if not exists
                if(key_parent==it)
                    first_one = tr.append_child(remain_plus, 1.0);
                else if(key_div!=tr.end())
                    tr.move_ontop(tr.append_child(remain_plus), key_div);
                else tr.move_ontop(tr.append_child(remain_plus), key_parent);
                //---------------------------------------------------
                //fusion with the chosen factor the elements equal to
                //---------------------------------------------------
                for(pre_it_vector_it factor_it =  es_vec.begin();
                    factor_it != es_vec.end(); ++factor_it) {
                    pre_it factor = *factor_it;
                    pre_it factor_parent = tr.parent(factor);
                    tr.erase(factor);
                    OC_ASSERT(tr.is_valid(factor_parent),
                              "factor parent node is invalid");
                    pre_it fp_parent = tr.parent(factor_parent);
                    pre_it factor_div = tr.end(); //position of div if factor is under
                    //determine factor_div
                    if(*factor_parent==id::div)
                        factor_div = factor_parent;
                    else if(tr.is_valid(fp_parent) && *fp_parent==id::div)
                        factor_div = fp_parent;
                    //if after erazing factor factor_parent is childless then
                    //the operator pointed by factor_parent must be erased
                    //and factor_parent_parent must point to its parent
                    if(factor_parent.is_childless()) {
                        tr.erase(factor_parent);
                        factor_parent = fp_parent;
                    }
                    //if factor was under div then 1.0 must be inserted where it was
                    if(factor_parent==factor_div)
                        tr.prepend_child(factor_parent, 1.0);
                    //move the remainder of the factor prod or put 1.0 if not exists
                    if(factor_parent==it) {
                        if(first_one==tr.end())
                            first_one = tr.append_child(remain_plus, 1.0);
                        else *first_one = get_contin(*first_one) + 1.0;
                    }
                    else if(factor_div!=tr.end())
                        tr.move_ontop(tr.append_child(remain_plus), factor_div);
                    else tr.move_ontop(tr.append_child(remain_plus), factor_parent);
                }
            }
        }
    }
}
    
void reduce_distribute::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it == id::times && it.number_of_children() >= 2) {
        sib_it plus_it = std::find(it.begin(), it.end(), vertex(id::plus));
        if(plus_it != it.end()) {
            unsigned int idx = 0;
            for(sib_it fac_it = it.begin();
                fac_it != it.end(); ++fac_it, ++idx) {
                if(*fac_it != id::plus) { // found a factor to distribute
                    combo_tree tr_copy(it);
                    unsigned int size_before_reduct = tr_copy.size();
                    pre_it tr_cr(tr_copy.begin()); //copy root
                    sib_it fac_copy_it = tr_copy.child(tr_cr, idx);
                    // distribute fac_copy_it
                    sib_it plus_ci = std::find(tr_cr.begin(), tr_cr.end(),
                                               vertex(id::plus));
                    for(sib_it child = plus_ci.begin();
                        child != plus_ci.end(); ++child) {
                        if(*child != id::times)
                            child = tr_copy.insert_above(child, id::times);
                        tr_copy.replace(tr_copy.append_child(child, vertex()),
                                        fac_copy_it);
                    }
                    tr_copy.erase(fac_copy_it);
                    (*_reduction)(tr_copy);
                    unsigned int size_after_reduct = tr_copy.size();
                    if(size_after_reduct <= size_before_reduct) {
                        replace_without_changing_it(tr, it, tr_cr);
                        return; //@todo: maybe the other
                        //combinations should be tried out
                        //to be complete
                    }
                }
            }
        }
    }
}

// x/c -> 1/c * x
// x/(c*y) -> 1/c *x/y
// 0/x -> 0
// x/0 -> NAN
void reduce_invert_constant::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it != id::div) 
        return;

    OC_ASSERT(it.number_of_children() == 2, 
              "combo_tree node should have exactly two children (reduce_invert_constant)." );
    sib_it sib = it.begin();
    if (is_contin(*sib) && get_contin(*sib) == 0.0) { // 0/x -> 0
        tr.erase_children(it);
        *it = 0.0;
        return;
    }

    ++sib;
    if (is_contin(*sib)) { // x/c -> 1/c * x
        contin_t divisor = get_contin(*sib);
        if (divisor != 0.0) {
            *sib = 1.0 / divisor;
            *it = id::times;
        } else {
            // Divide by zero
            tr.erase_children(it);
            *it = FP_INFINITE;
        }
        return;
    }

    if (*sib == id::times) { //x/(c*y) -> 1/c * x/y
        contin_t divisor = 1.0;
        for (sib_it mul_sib = sib.begin(); mul_sib != sib.end();) {
            if (is_contin(*mul_sib)) {
                divisor *= get_contin(*mul_sib);
                mul_sib = tr.erase(mul_sib);
            }
            else ++mul_sib;
        }
        if (divisor != 1.0) {
            if (divisor != 0.0) {
                *it = id::times;
                if (sib.is_childless())
                    *sib = 1.0 / divisor;
                else {
                    if(sib.has_one_child()) {
                        sib = tr.erase(tr.flatten(sib));
                    }
                    pre_it new_div = tr.append_child(it, id::div);
                    tr.reparent(new_div, it.begin(), sib_it(new_div));
                    tr.append_child(it, 1.0 / divisor);
                }
            }
            else {
                // Divide by zero
                // tr.append_child(sib, divisor); //divisor is put back
                tr.erase_children(it);
                *it = FP_INFINITE;
            }
        }
        else if (sib.is_childless())
            *sib = 1.0;
    }
}


//(x*y)/(x*z) -> y/z
//or more generally,
//(prod x_i*prod y_j)/(prod x_i*prod z_k)-> prod y_j/prod z_k
void reduce_fraction::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    //associate the number of instances of each subtree encountered
    //on the numerator of the fraction
    typedef std::multiset<pre_it, lexicographic_subtree_order<vertex> >
        num_counts;
    typedef num_counts::iterator num_counts_it;
    if(*it==id::div) {
        OC_ASSERT(it.number_of_children()==2, 
                  "combo_tree node should have exactly two children (reduce_fraction)." );
        pre_it numerator = tr.child(it, 0);
        pre_it denominator = tr.child(it, 1);
        num_counts nc;
        //fill num_counts
        if(*numerator==id::times)
            for(sib_it sib = numerator.begin(); sib != numerator.end(); ++sib)
                nc.insert(pre_it(sib));
        else nc.insert(numerator);
        //simplify fraction
        if(*denominator==id::times) {
            for(sib_it sib = denominator.begin();
                sib != denominator.end() && !nc.empty(); ++sib) {
                pre_it denom_el = sib;
                num_counts_it mirror_el_it = nc.find(denom_el);
                if(mirror_el_it != nc.end()) {
                    nc.erase(denom_el); //erase from the multiset
                    pre_it mirror_el = *mirror_el_it;
                    //erase up (or replace by 1.0)
                    if(mirror_el==numerator) {
                        tr.erase_children(mirror_el);
                        *mirror_el = 1.0;
                    }
                    else {
                        tr.erase(mirror_el);
                        if(numerator.is_childless())
                            *numerator = 1.0;
                    }
                    //erase down (or replace by 1.0)
                    tr.erase(denom_el);
                    if(denominator.is_childless())
                        *denominator = 1.0;
                }
            }
        }
        else {
            num_counts_it mirror_el_it = nc.find(denominator);
            if(mirror_el_it != nc.end()) {
                pre_it mirror_el = *mirror_el_it;
                //erase up (or replace by 1.0)
                if(mirror_el==numerator) {
                    tr.erase_children(mirror_el);
                    *mirror_el = 1.0;
                }
                else {
                    tr.erase(mirror_el);
                    if(numerator.is_childless())
                        *numerator = 1.0;
                }
                //replace by 1.0
                tr.erase_children(denominator);
                *denominator = 1.0;
            }
        }
    }
}

//(x/y)/z -> x/(y*z)
//x/(y/z) -> (x*z)/y
//x*(y/z) -> (x*y)/z,
//more generally prod x_i * prod y_j/z_j -> (prod x_i * prod y_j)/(prod z_j)
void reduce_times_div::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::div) {
        OC_ASSERT(it.number_of_children()==2,
                  "combo_tree node should have exactly two children (reduce_times_div)." );
        sib_it sib = it.begin();
        if(*sib==id::div) { //(x/y)/z -> x/(y*z)
            OC_ASSERT(sib.number_of_children()==2,
                      "combo_tree sibiling node should have exactly two children (reduce_times_div)." );
            pre_it y = sib.last_child();
            pre_it z = it.last_child();
            tr.insert_above(z, id::times);
            tr.move_after(z, y);
            tr.erase(tr.flatten(sib));
            return;
        }
        ++sib;
        if(*sib==id::div) { //x/(y/z) -> (x*z)/y
            OC_ASSERT(sib.number_of_children()==2,
                      "combo_tree sibiling node should have exactly two children (reduce_times_div).");
            pre_it x = it.begin();
            pre_it z = sib.last_child();
            tr.insert_above(x, id::times);
            tr.move_after(x, z);
            tr.erase(tr.flatten(sib));
        }
    }
    else if(*it==id::times) { //prod x_i * prod (y_j/z_j)
        //-> (prod x_i * prod y_j)/(prod z_j)
        pre_it denom = tr.end(); //denom is the first z_j in place, so the
        //other z_j will just have to go next it
        pre_it denom_times = tr.end(); //denom_times is the times of all
        //z_j, necessary if j>1
        pre_it new_times = tr.end(); //new_times is the times of
        //prod x_i * prod y_j
        for(sib_it sib = it.begin(); sib != it.end(); ++sib) {
            if(*sib==id::div) {
                if(*it!=id::div) { // div needs to be added to the top/root
                    OC_ASSERT(!it.is_childless(),
                              "combo_tree node should not be childless (reduce_time_div)." );
                    new_times = tr.prepend_child(it, id::times);
                    sib_it second_child = tr.child(it, 1);
                    tr.reparent(new_times, second_child, it.end());
                    *it = id::div;
                }
                OC_ASSERT(sib.number_of_children()==2, 
                          "combo_tree sibiling node should have exactly two children (reduce_times_div).");
                pre_it z_j = sib.last_child();
                if(denom==tr.end())
                    denom = tr.move_after(new_times, z_j);
                else {
                    if(denom_times==tr.end())
                        denom_times = tr.insert_above(denom, id::times);
                    tr.move_after(denom, z_j);
                }
                sib = tr.erase(tr.flatten(sib));
            }
        }
    }
}

//+(x) -> x
//*(x) -> x
void reduce_plus_times_one_child::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if((*it==id::plus || *it==id::times) && it.has_one_child()) {
        pre_it it_child = it.begin();
        *it=*it_child;
        tr.erase(tr.flatten(it_child));
    }
}
    
//log(x)+log(y) -> log(x*y),
//log(x)-log(y) -> log(x/y)
//or more generally
//sum log(x_i) - sum log(y_j) -> log((prod x_i)/(prod y_j))
//works only if at least one log(x_i) exists otherwise
//there would be a conflict with the rule log(c/x) -> -log((1/c)*x)
void reduce_sum_log::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it != id::plus)
        return;

    sib_it first_log = it.find_child(id::log);
    if (first_log == it.end())
        return;

    OC_ASSERT(first_log.has_one_child(),
              "combo_tree node should have exactly one child.");
    pre_it num = first_log.begin(); // sibling at the numerator of div
    pre_it denom = tr.end(); // sibling at the denominator of div
    pre_it num_times = tr.end(); // times at the numerator
    pre_it denom_times = tr.end(); // times at the denominator
    for (sib_it sib = it.begin(); sib != it.end();) {
        // detect log(x) to add to log(prod x_i)
        if (*sib == id::log && sib != first_log) {
            OC_ASSERT(sib.has_one_child(), 
                      "combo_tree sibiling node should have exactly one child.");
            if (num_times == tr.end())
                num_times = tr.wrap(num, id::times);
            tr.move_after(num, pre_it(sib.begin()));
            sib = tr.erase(sib);
        }
        else if (*sib == id::times && sib.number_of_children() == 2) {
            // check if there is -1*log
            sib_it minus1 = sib.end();
            sib_it log = sib.end();
            bool is_minus_log = (minus1 != sib.end() && log != sib.end());//false
            for (sib_it times_child = sib.begin();
                times_child != sib.end() && !is_minus_log; ++times_child) {
                if (minus1 == sib.end())
                    if (is_contin(*times_child) && get_contin(*times_child) == -1.0)
                        minus1 = times_child;
                if (log == sib.end())
                    if (*times_child == id::log)
                        log = times_child;
                is_minus_log = (minus1 != sib.end() && log != sib.end());
            }
            if (is_minus_log) { // there is -1*log
                if (denom == tr.end()) {
                    OC_ASSERT(first_log.has_one_child(),
                              "combo_tree node should have exactly one child");
                    pre_it tmp_child_log = first_log.begin();
                    pre_it div_node = tr.wrap(tmp_child_log, id::div);
                    OC_ASSERT(div_node.has_one_child(),
                              "combo_tree sibiling node should have exactly one child");
                    // below cannot be replaced by move_before because it has
                    // to be the second argument of div, i.e. the denominator
                    denom = tr.move_after(div_node.begin(), log.begin());
                    sib = tr.erase(sib);
                }
                else {
                    if (denom_times == tr.end()) {
                        denom_times = tr.wrap(denom, id::times);
                    }
                    OC_ASSERT(log.has_one_child(),
                              "combo_tree node (log) should have exactly one child");
                    tr.move_after(denom, pre_it(log.begin()));
                    sib = tr.erase(sib);
                }
            }
            else ++sib;
        }
        else ++sib;
    }
}

  
// log(c/x) -> -log(c^-1*x)
// log(0) -> NAN
// and also
// log(exp(x)*y) -> x+log(y)
// or more generally log(prod exp(x_i)*prod y_j) -> sum x_i +log(prod y_j)
void reduce_log_div_times::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it != id::log)
        return;

    OC_ASSERT(it.has_one_child(), 
              "combo_tree node id::log should have exactly one child");

    pre_it log_child = it.begin();
    if (*log_child == id::div) { // log(c/x) -> log(c^-1*x)
        OC_ASSERT(log_child.number_of_children() == 2,
                  "combo_tree node id::div should have exactly two children");
        pre_it num = log_child.begin();
        // pre_it denom = log_child.last_child();
        if (is_contin(*num)) {
            contin_t c = get_contin(*num);
            if (c != 0.0) {
                *num = 1.0 / c;
                *log_child = id::times;
                //trick to keep it as root of the subtree
                *it = id::times;
                tr.insert_after(tr.wrap(log_child, id::log), -1.0);
            }
            else {
                // log(0)
                tr.erase_children(it);
                *it = FP_INFINITE;
                return;
            }
        }
        return;
    }

    if (*log_child == id::exp) { // log(exp(x)) -> x
        OC_ASSERT(log_child.has_one_child(),
                  "combo_tree node id::exp should have exactly one child");
        tr.erase(tr.flatten(log_child));
        *it = *it.begin();
        tr.erase(tr.flatten(it.begin()));
        return;
    }

   if (*log_child == id::times) { // log(prod exp(x_i)*prod y_j)
        // -> sum x_i + log(prod y_j)
        pre_it new_log = tr.end();
        for (sib_it sib = log_child.begin(); sib != log_child.end();) {
            if (*sib == id::exp) {
                if (*it != id::plus) {
                    new_log = tr.insert_above(log_child, id::log);
                    *it = id::plus;
                }
                OC_ASSERT(sib.has_one_child(),
                          "combo_tree sibling node should have exactly one child");
                //the child of exp is moved under plus beside log
                tr.reparent(it, pre_it(sib));
                sib = tr.erase(sib);
            }
            else ++sib;
        }
        // sum x_i + log(*()) -> sum x_i
        if (new_log.has_one_child() && new_log.begin().is_childless())
            tr.erase(new_log);
        return;
    }
}


//prod exp(x_i) -> exp(sum x_i)
void reduce_exp_times::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it != id::times)
        return;

    pre_it x_1 = tr.end(); //first x_i detected
    for (sib_it sib = it.begin(); sib != it.end(); ) {
        if (*sib == id::exp) {
            OC_ASSERT(sib.has_one_child(), 
                      "combo_tree sibling node should have exactly one child (reduce_exp_times).");
                if (x_1 == tr.end()) {
                x_1 = sib.begin();
                ++sib;
            }
            else {
                if (*tr.parent(x_1) != id::plus)
                    tr.insert_above(x_1, id::plus);
                tr.move_after(x_1, pre_it(sib.begin()));
                sib = tr.erase(sib);
            }
        }
        else ++sib;
    }
}

   
// x/exp(y) -> x*exp(-y)
void reduce_exp_div::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if (*it != id::div)
        return;

    OC_ASSERT(it.number_of_children() == 2,
              "combo_tree node should have exactly two childrem (reduce_exp_div).");
    pre_it denom = it.last_child();
    if (*denom != id::exp)
        return;

    *it = id::times;
    OC_ASSERT(denom.has_one_child(),
              "combo_tree node should have exactly one child (reduce_exp_times - demon).");
    pre_it y = denom.begin();
    if (*y == id::times) {
        OC_ASSERT(!y.is_childless(),
                  "combo_tree sibling node should have exactly one child (reduce_exp_times - y).");
        y = y.begin();
    }
    else tr.insert_above(y, id::times);

    tr.insert_after(y, -1.0);
}

  
// the following rules is not valid if log has the semantics log(abs(x))
// the macro ABS_LOG is defined in file vertex.h
#ifndef ABS_LOG
// exp(log(x)+y) -> x*exp(y)
// or more generally, exp(sum log(x_i) + sum y_j) -> prod x_i * exp(sum y_j)
void reduce_exp_log::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::exp) {
        OC_ASSERT(it.has_one_child(), 
                  "combo_tree node should have exactly one child (reduce_exp_log).");
        pre_it exp_child = it.begin();
        if(*exp_child==id::plus) {
            pre_it x_1 = tr.end();
            pre_it new_exp = tr.end();
            OC_ASSERT(!exp_child.is_childless(),
                      "combo_tree child node should have exactly one child (reduce_exp_log).");
            for(sib_it sib = exp_child.begin(); sib != exp_child.end(); ) {
                if(*sib==id::log) {
                    OC_ASSERT(sib.has_one_child(), 
                              "combo_tree sibling node should have exactly one child (reduce_exp_log).");
                    if(x_1==tr.end()) {
                        *it = id::times;
                        x_1 = tr.move_after(exp_child, pre_it(sib.begin()));
                        new_exp = tr.insert_above(exp_child, id::exp);
                    }
                    else tr.move_after(x_1, pre_it(sib.begin()));
                    sib = tr.erase(sib);
                }
                else ++sib;
            }
            //if exp(+()) erase it
            if(new_exp!=tr.end() && new_exp.begin().is_childless()) {
                tr.erase(tr.flatten(new_exp.begin()));
                tr.erase(tr.flatten(new_exp));
            }
        }
        else if(*exp_child==id::log) {
            tr.erase(tr.flatten(exp_child));
            *it=*it.begin();
            tr.erase(tr.flatten(it.begin()));
        }
    }
}
#endif
  
// sin(x + c) -> sin(x + (c>pi? c-pi : (c<= pi? c+pi))
// or more generally
// sin(sum x_i + sum c_j) -> sin(sum x_i + ((sum c_j)+pi)%2pi -pi
//
// TODO:  sin(*(-1 x)) -> -sin(x)
// The above is frequently seen in real-life ...
void reduce_sin::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it != id::sin)
        return;

    OC_ASSERT(it.has_one_child(),
              "combo_tree node id::sin should have exactly one child (reduce_sin).");
    pre_it c_it = tr.end();
    pre_it sin_child = it.begin();
    if (*sin_child == id::plus) {
        OC_ASSERT(!sin_child.is_childless(), 
                  "combo_tree node id::plus should not be childless (reduce_sin).");
        for (sib_it sib = sin_child.begin(); sib != sin_child.end(); ) {
            if (is_contin(*sib)) {
                if (c_it == tr.end()) {
                    c_it = sib;
                    ++sib;
                }
                else {
                    *c_it = get_contin(*c_it) + get_contin(*sib);
                    sib = tr.erase(sib);
                }
            }
            else ++sib;
        }
    }
    else if (is_contin(*sin_child))
        c_it = sin_child;

    // Put the constant term into the range (-PI, PI]
    if (c_it != tr.end()) {
        OC_ASSERT(is_contin(*c_it),
                  "sin_child isn't of type contin (reduce_sin).");
        contin_t c = get_contin(*c_it);
        if (not isfinite(c)) {
            tr.erase_children(it);
            *it = FP_NAN;
            return;
        }
        if (c <= -PI || c > PI)
            *c_it = fmod((c+PI), 2.0*PI) - PI;
    }
}

// Apply boolean reduction to the argument of impulse.
void reduce_impulse_arg::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it != id::impulse)
        return;

    OC_ASSERT(it.has_one_child(),
              "impulse should have exactly one child (reduce_impulse_arg).\n%s\n",
             ({std::stringstream ss; ss<<tr; ss.str().c_str(); }));
    pre_it it_child = it.begin();
    logical_reduce(reduct_effort, tr, it_child, ignore_ops);
}


} // ~namespace reduct
} // ~namespace opencog

