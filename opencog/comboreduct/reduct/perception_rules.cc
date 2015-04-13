/*
 * opencog/comboreduct/reduct/perception_rules.cc
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
#include "perception_rules.h"
#include <opencog/comboreduct/combo/assumption.h>
#include <opencog/util/exceptions.h>

namespace opencog { namespace reduct {
typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;

void reduce_ultrametric::operator()(combo_tree& tr,combo_tree::iterator it) const {
    if(is_ultrametric(*it)) {
        OC_ASSERT(it.number_of_children()==2, 
                  "combo_tree node should have exactly two children (reduce_ultrametric).");
        std::vector<pre_it> it_vec;
        if(find_vertices_in_assumptions(tr, *it, it_vec)) {
            std::vector<pre_it> arg1;
            std::vector<pre_it> arg2;
            for(std::vector<pre_it>::iterator it_vec_it = it_vec.begin();
                it_vec_it != it_vec.end(); ++it_vec_it) {
                OC_ASSERT((*it_vec_it).number_of_children()==2,
                          "combo_tree vertice node should have exactly two children (reduce_ultrametric).");
                arg1.push_back(pre_it(tr.child(*it_vec_it, 0)));
                arg2.push_back(pre_it(tr.child(*it_vec_it, 1)));
            }
            for(unsigned int i2 = 0; i2 < arg2.size(); ++i2) {
                for(unsigned int i1 = 0; i1 < arg2.size(); ++i1) {
                    if(i1 != i2 && tr.equal_subtree(arg1[i2], arg2[i1])) {
                        //must build and add 0<f(a,c)-f(a,b) and 0<f(a,c)-f(b,c)
                        //that is :
                        //build and add the two new assumptions
                        //0<*it(arg1[i1],arg2[i2])-*it(arg1[i1],arg2[i1])
                        //0<*it(arg1[i1],arg2[i2])-*it(arg1[i2],arg2[i2])
                        {
                            combo_tree new_a1_tr;
                            pre_it ait1 = new_a1_tr.set_head(id::greater_than_zero);
                            pre_it pl1 = new_a1_tr.append_child(ait1, id::plus);
                            { //create *it(arg1[i1],arg2[i2])
                                pre_it it1 = new_a1_tr.append_child(pl1, *it);
                                tr.replace(tr.append_child(it1), arg1[i1]);
                                tr.replace(tr.append_child(it1), arg2[i2]);
                            }
                            { //create -*it(arg1[i1],arg2[i1])
                                pre_it new_times = new_a1_tr.append_child(pl1, id::times);
                                new_a1_tr.append_child(new_times, -1.0);
                                pre_it it2 = new_a1_tr.append_child(new_times, *it);
                                tr.replace(tr.append_child(it2), arg1[i1]);
                                tr.replace(tr.append_child(it2), arg2[i1]);
                            }		  
                            //insert first assumption
                            insert_assumption(tr, ait1);
                        }
                        {
                            combo_tree new_a2_tr;
                            pre_it ait2 = new_a2_tr.set_head(id::greater_than_zero);
                            pre_it pl2 = new_a2_tr.append_child(ait2, id::plus);
                            { //create *it(arg1[i1],arg2[i2])
                                pre_it it1 = new_a2_tr.append_child(pl2, *it);
                                tr.replace(tr.append_child(it1), arg1[i1]);
                                tr.replace(tr.append_child(it1), arg2[i2]);
                            }
                            { //create -*it(arg1[i2],arg2[i2])
                                pre_it new_times = new_a2_tr.append_child(pl2, id::times);
                                new_a2_tr.append_child(new_times, -1.0);
                                pre_it it2 = new_a2_tr.append_child(new_times, *it);
                                tr.replace(tr.append_child(it2), arg1[i2]);
                                tr.replace(tr.append_child(it2), arg2[i2]);
                            }		  
                            //insert first assumption
                            insert_assumption(tr, ait2);
                        }
                    }
                }
            }
        }
    }
}

void reduce_transitive::operator()(combo_tree& tr,combo_tree::iterator it) const {
    if(is_transitive(*it)) {
        OC_ASSERT(it.number_of_children()==2, 
                  "combo_tree node should have exactly two children (reduce_transitive).");
        pre_it main_tree = tr.begin();
        sib_it assum = tr.next_sibling(main_tree);
        if(tr.is_valid(assum)) {
            std::vector<pre_it> arg1;
            std::vector<pre_it> arg2;
            for(;assum != tr.end(); ++assum) {
                if(*assum==*it) {
                    OC_ASSERT(assum.number_of_children()==2,
                              "combo_tree node should have exactly two children (reduce_transitive - assum).");
                    arg1.push_back(pre_it(tr.child(assum, 0)));
                    arg2.push_back(pre_it(tr.child(assum, 1)));
                }
            }
            if(!arg1.empty() /*or arg2 not empty*/) {
                for(unsigned int i2 = 0; i2 < arg2.size(); ++i2) {
                    for(unsigned int i1 = 0; i1 < arg2.size(); ++i1) {
                        if(i1 != i2 && tr.equal_subtree(arg1[i2], arg2[i1])) {
                            //build new assumption *it(arg1[i1], arg2[i2])
                            combo_tree new_assum_tr;
                            pre_it na_it = new_assum_tr.set_head(*it);
                            tr.replace(new_assum_tr.append_child(na_it), arg1[i1]);
                            tr.replace(new_assum_tr.append_child(na_it), arg2[i2]);
                            //insert new assumption
                            insert_assumption(tr, na_it);
                        }
                    }
                }
            }
        }
    }
}

void reduce_reflexive::operator()(combo_tree& tr,combo_tree::iterator it) const {
    if(is_reflexive(*it)) {
        OC_ASSERT(it.number_of_children()==2,
                  "combo_tree node should have exactly two children (reduce_reflexive).");
        if(tr.equal_subtree(pre_it(tr.child(it, 0)), pre_it(tr.child(it, 1)))) {
            *it = id::logical_true;
            tr.erase_children(it);
        }
    }
}

void reduce_irreflexive::operator()(combo_tree& tr,combo_tree::iterator it) const {
    if(is_irreflexive(*it)) {
        OC_ASSERT(it.number_of_children()==2,
                  "combo_tree node should have exactly two children (reduce_irreflexive).");
        if(tr.equal_subtree(pre_it(tr.child(it, 0)), pre_it(tr.child(it, 1)))) {
            *it = id::logical_false;
            tr.erase_children(it);
        }
    }
}

void reduce_symmetric::operator()(combo_tree& tr,combo_tree::iterator it) const {
    if(is_symmetric(*it)) {
        OC_ASSERT(it.number_of_children()==2,
                  "combo_tree node should have exactly two children (reduce_symmetric).");
        //look at the set assumptions to find near(arg1, arg2)
        //and each time add near(arg2, arg1)
        pre_it main_tree = tr.begin();
        sib_it assum = tr.next_sibling(main_tree);
        if(tr.is_valid(assum)) {
            std::vector<pre_it> arg1;
            std::vector<pre_it> arg2;
            for(;assum != tr.end(); ++assum) {
                if(*assum==*it) {
                    OC_ASSERT(assum.number_of_children()==2,
                              "combo_tree node should have exactly two children (reduce_symmetric - assum).");
                    arg1.push_back(pre_it(tr.child(assum, 0)));
                    arg2.push_back(pre_it(tr.child(assum, 1)));
                }
            }
            if(!arg1.empty() /*or arg2 not empty*/) {
                OC_ASSERT(arg1.size()==arg2.size(),
                          "arg1 and arg2 vectors should have the same size.");
                for(unsigned int i = 0; i < arg1.size(); ++i) {
                    if(!tr.equal_subtree(arg1[i], arg2[i])) {
                        //build new assumption *it(arg2[i], arg1[i])
                        combo_tree new_assum_tr;
                        pre_it na_it = new_assum_tr.set_head(*it);
                        tr.replace(new_assum_tr.append_child(na_it), arg2[i]);
                        tr.replace(new_assum_tr.append_child(na_it), arg1[i]);
                        //insert new assumption
                        insert_assumption(tr, na_it);
                    }
                }
            }
        }
    }
}


void reduce_identity_of_indiscernibles::operator()(combo_tree& tr,combo_tree::iterator it) const {
    if(is_identity_of_indiscernibles(*it)) {
        OC_ASSERT(it.number_of_children()==2,
                  "combo_tree node should have exactly two children (reduce_identity_of_indiscernibles).");
        if(tr.equal_subtree(pre_it(tr.child(it, 0)), pre_it(tr.child(it, 1)))) {
            *it = 0.0;
            tr.erase_children(it);
        }
    }
}

} // ~namespace reduct
} // ~namespace opencog
