/*
 * opencog/comboreduct/reduct/reduct.h
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
#ifndef _REDUCT_RULE_H
#define _REDUCT_RULE_H

#include <opencog/util/RandGen.h>

#include "using.h"
#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace reduct {

using namespace opencog::combo;
    
struct rule {
    typedef combo_tree argument_type; // useful for cache rule

    rule(std::string _name) : name(_name) {}
    virtual ~rule() {}
    virtual void operator()(combo_tree&,combo_tree::iterator) const=0;
    virtual rule* clone() const=0;

    void operator()(combo_tree& tr) const { 
        if (!tr.empty())
            (*this)(tr,tr.begin());
    }

    std::string get_name() const {
        return name;
    }
protected:
    std::string name;
};
reduct::rule* new_clone(const reduct::rule& r);

template<typename T>
struct crule : public rule {
    crule(std::string _name) : rule(_name) {}
    rule* clone() const { return new T(*((T*)this)); }
};

const rule& ann_reduction();
const rule& logical_reduction(int effort);

// ignore_ops is the set of operator to ignore
const rule& contin_reduction(const vertex_set& ignore_ops, opencog::RandGen& rng,
                             unsigned int cache_size = 0);

const rule& mixed_reduction(opencog::RandGen& rng);
const rule& full_reduction(opencog::RandGen& rng);
const rule& action_reduction();
const rule& perception_reduction();

const rule& clean_reduction();
//const rule& clean_and_full_reduction();

inline void logical_reduce(int effort, combo_tree& tr,combo_tree::iterator it) {
    logical_reduction(effort)(tr,it);
}
inline void logical_reduce(int effort, combo_tree& tr) {
    logical_reduction(effort)(tr);
}

inline void contin_reduce(combo_tree& tr,combo_tree::iterator it, const vertex_set& ignore_ops, opencog::RandGen& rng) {
    contin_reduction(ignore_ops, rng)(tr,it);
}
inline void contin_reduce(combo_tree& tr, const vertex_set& ignore_ops,
                          opencog::RandGen& rng, unsigned int cache_size = 0) {
    contin_reduction(ignore_ops, rng, cache_size)(tr); 
}
inline void mixed_reduce(combo_tree& tr,combo_tree::iterator it, opencog::RandGen& rng) {
    mixed_reduction(rng)(tr,it);
}
inline void mixed_reduce(combo_tree& tr, opencog::RandGen& rng) { mixed_reduction(rng)(tr); }

inline void full_reduce(combo_tree& tr,combo_tree::iterator it, opencog::RandGen& rng) {
    full_reduction(rng)(tr,it);
}
inline void full_reduce(combo_tree& tr, opencog::RandGen& rng) { full_reduction(rng)(tr); }

inline void ann_reduce(combo_tree& tr) {ann_reduction()(tr); }

/**
 * clean_reduce removes null vertices
 */
inline void clean_reduce(combo_tree& tr,combo_tree::iterator it) {
    clean_reduction()(tr,it);
}
inline void clean_reduce(combo_tree& tr) { clean_reduction()(tr); }

inline void clean_and_full_reduce(combo_tree& tr,combo_tree::iterator it, opencog::RandGen& rng) {
    //clean_and_full_reduction()(tr,it);
    clean_reduce(tr,it);
    full_reduce(tr,it,rng);
}
inline void clean_and_full_reduce(combo_tree& tr, opencog::RandGen& rng) { 
    //clean_and_full_reduction()(tr,tr.begin()); 
    clean_reduce(tr);
    full_reduce(tr,rng);
}

//action
inline void action_reduce(combo_tree& tr, combo_tree::iterator it) {
    action_reduction()(tr,it);
}

inline void action_reduce(combo_tree& tr) { action_reduction()(tr); }

//perception
inline void perception_reduce(combo_tree& tr, combo_tree::iterator it) {
    perception_reduction()(tr,it);
}

inline void perception_reduce(combo_tree& tr) { perception_reduction()(tr); }

// helper to replace a subtree by another subtree (belonging to the
// same or another tree without changing the iterator of the new tree,
// this is because the reduct engine assumes that the rules do not
// change the iterator they take in argument. That is dst is not
// changed (expect possibly it's content of course).
// tr is the tree containing dst iterator
inline void replace_without_changing_it(combo_tree& tr,
                                        combo_tree::iterator dst,
                                        combo_tree::iterator src) {
    *dst = *src;
    if(src.is_childless())
        tr.erase_children(dst);
    else {
        // it is assumed (for now) that dst and src have the same
        // number of children
        tr.replace(dst.begin(), dst.end(), src.begin(), src.end());
    }    
}

} // ~namespace reduct
} // ~namespace opencog

#endif
