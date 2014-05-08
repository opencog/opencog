/*
 * opencog/learning/moses/eda/local_structure.cc
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

#include <boost/iterator/indirect_iterator.hpp>

#include <opencog/util/numeric.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/selection.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>

#include "local_structure.h"

namespace opencog {
namespace moses {

// true if the variable at index idx is identical over the range of
// instances [l, u)
bool local_structure_model::is_uniform_on(iptr_iter l, iptr_iter u, int idx) const
{
    return (adjacent_find(boost::make_indirect_iterator(l), boost::make_indirect_iterator(u),
                          bind(std::not_equal_to<disc_t>(),
                               bind(&field_set::get_raw, &_fields, _1, idx),
                               bind(&field_set::get_raw, &_fields, _2, idx))) ==
            boost::make_indirect_iterator(u));
}

void local_structure_model::rec_split_term(iptr_iter l, iptr_iter u,
                                           int src_idx, int idx,
                                           dtree::iterator node,
                                           term_tree::iterator osrc)
{
    int raw_arity = osrc.number_of_children() + 1;

    if (is_uniform_on(l, u, idx) || raw_arity == 1) {
        // make us a leaf
        *node = dtree_node(raw_arity + 1, 0);
        return;
    }

    OC_ASSERT(node->size() == 1, "dtree node size should be equal to 1.");
    (begin() + idx)->append_children(node, *node, raw_arity);
    node->front() = src_idx;
    (*node.begin()) = dtree_node(2, 0);

    if (src_idx < idx - 1) { // important - needs to match term_spec::Left/Right
        vector<iptr_iter> pivots(raw_arity);
        pivots.back() = u;
        n_way_partition(l, u,
                        bind(&field_set::get_raw, &_fields,
                             bind(valueof<const instance>, _1), src_idx),
                        raw_arity, pivots.begin());

        // we don't recurse on the leftmost partition - it's term_spec::Stop
        // instead just make it a (null) leaf
        for_each(pivots.begin(), --pivots.end(), ++pivots.begin(),
                 make_counting_iterator(++node.begin()),
                 make_counting_iterator(osrc.begin()),
                 bind(&local_structure_model::rec_split_term, this, _1, _2,
                      src_idx + 1, idx, _3, _4));
    } else {
        //we're the parent of leaves
        dtree::sibling_iterator dsib = ++node.begin();
        for(term_tree::sibling_iterator osib = osrc.begin(); osib != osrc.end();
            ++osib, ++dsib) {
            *dsib = dtree_node(raw_arity + 1, 0);
        }
    }
}

void local_structure_model::rec_split_contin(iptr_iter l, iptr_iter u,
                                             int src_idx, int idx,
                                             dtree::iterator node)
{
    if(is_uniform_on(l, u, idx))
        return;

    split(src_idx, idx, node);

    if (src_idx < idx - 1) { // important - needs to match contin_spec::Left/Right
        vector<iptr_iter> pivots(2);
        n_way_partition(l, u,
                        bind(&field_set::get_raw, &_fields,
                             bind(valueof<const instance>, _1), src_idx),
                        3, pivots.begin());
        rec_split_contin(pivots[0], pivots[1], src_idx + 1, idx, ++node.begin());
        rec_split_contin(pivots[1], u, src_idx + 1, idx, node.last_child());
    }
}

void local_structure_model::make_dtree(super::iterator dtr, int arity)
{
    // plus one: the first arity will hold counts, the last will hold
    // the grand-total.
    dtr->set_head(dtree_node(arity + 1, 0));
}

void local_structure_model::split(int src_idx, int tgt_idx,
                                  dtree::iterator tgt)
{
    OC_ASSERT(tgt.is_childless(), "dtree node should be childless (split)");
    (begin() + tgt_idx)->append_children(tgt, *tgt, tgt->size() - 1);
    *tgt = dtree_node(1, src_idx);
}

// sample from a model
instance local_structure_model::operator()() const
{
    vector<disc_t> tmp(size());
    for (unsigned int idx : _ordering) {
        sample((begin() + idx)->begin(), tmp[idx], tmp);
    }
    instance res(_instance_length);
    _fields.pack(tmp.begin(), res.begin());

    return res;
}

void local_structure_model::sample(dtree::iterator dtr, disc_t& dst,
                                   const vector<disc_t>& margs) const
{
    if (dtr.is_childless()) { //a leaf
        if (dtr->back() > 0)
            dst = distance(dtr->begin(),
                           roulette_select(dtr->begin(), --(dtr->end()),
                                           dtr->back()));
        else
            dst = 0;//rng.randint(dtr->size()-1); //if no data, do uniform selection    WHY IS THIS COMMENTED OUT???
    } else {
        OC_ASSERT(dtr->size() == 1,
                  "dtree node size should be equals to 1.");
        OC_ASSERT(dtr.number_of_children() > margs[dtr->front()],
                  "dtree node children number grearter than margs.");
        dtree::sibling_iterator child = dtr.begin();
        child += margs[dtr->front()];
        sample(child, dst, margs);
    }
}

} // ~namespace moses
} // ~namespace opencog
