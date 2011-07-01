/*
 * opencog/comboreduct/combo/assumption.h
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
#include <opencog/util/exceptions.h>

#include "assumption.h"
#include <iostream>
#include <vector>

namespace opencog { namespace combo {

typedef combo_tree::iterator pre_it;
typedef combo_tree::sibling_iterator sib_it;

void insert_assumption(combo_tree& tr, combo_tree::iterator assum_it) {
  typedef opencog::lexicographic_subtree_order<vertex> Comp;
  pre_it main_tree = tr.begin();
  OC_ASSERT(tr.is_valid(main_tree), "combo_tree ins't valid (insert_assumption).");
  int max_pos = tr.number_of_siblings(main_tree);
  if(max_pos>1) {
    int min_pos = 1;
    int diff = (max_pos-min_pos)/2;
    int cur_pos = min_pos + diff;
    sib_it cur_sib = main_tree;
    cur_sib += cur_pos;
    Comp comp;
    while(min_pos<max_pos) {
  if(comp(pre_it(cur_sib), assum_it)) {
    min_pos = cur_pos+1;
    diff = 1+(max_pos-min_pos)/2;
    cur_pos += diff;
    cur_sib += diff;
  }
  else if(comp(assum_it, pre_it(cur_sib))) {
    max_pos = cur_pos-1;
    diff = 1+(max_pos-min_pos)/2;
    cur_pos -= diff;
    cur_sib -= diff;
  }
  else return;
    }
    if(max_pos<min_pos)
  tr.insert_subtree_after(cur_sib, assum_it);
    else if(min_pos==max_pos) {
  if(comp(pre_it(cur_sib), assum_it)) {
    tr.insert_subtree_after(cur_sib, assum_it);
  }
  else if(comp(assum_it, pre_it(cur_sib))) {
    tr.insert_subtree(cur_sib, assum_it);
  }
    }
    else OC_ASSERT(false, "should never get here (insert_assumption).");
  }
  else tr.insert_subtree_after(main_tree, assum_it);
}

bool find_vertices_in_assumptions(const combo_tree& tr, vertex v,
  			    std::vector<combo_tree::iterator>& res) {
  pre_it tr_it = tr.begin();
  OC_ASSERT(tr.is_valid(tr_it), "combo_tree isn't valid (find_vertices_in_assumptions).");
  pre_it assum_it = tr.next_sibling(tr_it);
  bool is_there = false;
  if(tr.is_valid(assum_it)) {
    for(; assum_it != tr.end(); ++assum_it) {
  if(*assum_it == v) {
    res.push_back(assum_it);
    is_there = true;
  }
    }
  }
  return is_there;
}

bool equal_assumptions(const combo_tree& tr1, const combo_tree& tr2) {
  OC_ASSERT(!tr1.empty() && !tr2.empty(), "assumptions should not be empty (equal_assumptions).");
  sib_it assum1 = tr1.next_sibling(tr1.begin());
  sib_it assum2 = tr2.next_sibling(tr2.begin());
  if(tr1.is_valid(assum1) && tr2.is_valid(assum2)) {
    for(; assum1 != tr1.end() && assum2 != tr2.end(); ++assum1, ++assum2)
  if(!tr1.equal_subtree(assum1, assum2))
    return false;
    if(assum1==tr1.end() && assum2==tr2.end())
  return true;
    else return false;
  }
  else return true;
}

void delete_all_assumptions(combo_tree& tr) {
  OC_ASSERT(!tr.empty(), "combo_tree should not be empty (delete_all_assumptions).");
  sib_it assum = tr.next_sibling(tr.begin());
  if(tr.is_valid(assum))
    for(; assum != tr.end();) 
  assum = tr.erase(assum);
}

}} // ~namespaces combo opencog
