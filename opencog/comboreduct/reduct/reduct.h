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

#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace reduct {

using namespace opencog::combo;

struct rule
{
    typedef combo_tree argument_type;

    rule(std::string _name) : name(_name) {}
    virtual ~rule() {}
    virtual void operator()(combo_tree&, combo_tree::iterator) const=0;
    virtual rule* clone() const=0;

    void operator()(combo_tree& tr) const
    {
        if (!tr.empty())
            (*this)(tr, tr.begin());
    }

    std::string get_name() const
    {
        return name;
    }

protected:
    std::string name;
};

// new_clone() is needed by boost ptr_vector in struct sequential
// (and other places) but is not otherwise used anywhere.
reduct::rule* new_clone(const reduct::rule& r);

template<typename T>
struct crule : public rule
{
    crule(std::string _name) : rule(_name) {}
    rule* clone() const { return new T(*((T*) this)); }
};

const rule& ann_reduction();

// @ignore_ops is the set of operators to ignore
// logical_reduction doesn't use it itself, but does pass it
// on to the contin_reduction step of any predicates encountered.
// Reduction levels of 2 and higher must specify ignore_ops.
struct logical_reduction
{
    logical_reduction();
    logical_reduction(const logical_reduction&);
    logical_reduction(const vertex_set& ignore_ops);
    logical_reduction& operator=(const logical_reduction&);
    ~logical_reduction();

    const rule& operator()(int effort = 2);
private:
    void do_init();
    const rule* p_medium;
    const rule* p_complexe;
public:
    static rule* p_extra_simple;
    static rule* p_simple;
};

// @ignore_ops is the set of operators to ignore
const rule& contin_reduction(int reduct_effort,
                             const vertex_set& ignore_ops);

const rule& fold_reduction();
const rule& mixed_reduction();
const rule& full_reduction();
const rule& action_reduction();
const rule& perception_reduction();

const rule& clean_reduction();
//const rule& clean_and_full_reduction();

/**
 * reduce trees containing logical operators, boolean constants
 * and boolean-typed literals.
 *
 * For effort==2 and greater, this will also recurse down into
 * predicates, and reduce those. Currently, only the greater-than-zero
 * predicate is supported; when such a predicate is encountered,
 * the contin terms inside of it are reduced.
 */
inline void logical_reduce(int effort, combo_tree& tr,
                           combo_tree::iterator it,
                           const vertex_set& ignore_ops)
{
    logical_reduction r(ignore_ops);
    r(effort)(tr, it);
}

inline void logical_reduce(int effort, combo_tree& tr,
                           const vertex_set& ignore_ops)
{
    logical_reduction r(ignore_ops);
    r(effort)(tr);
}

inline void logical_reduce(int effort, combo_tree& tr)
{
    logical_reduction r;
    r(effort)(tr);
}

/**
 * reduce trees containing only arithmetic operators, float-point
 * constants and contin-typed literals.
 */
inline void contin_reduce(combo_tree& tr, combo_tree::iterator it,
                          int reduct_effort,
                          const vertex_set& ignore_ops)
{
    contin_reduction(reduct_effort, ignore_ops)(tr, it);
}

inline void contin_reduce(combo_tree& tr,
                          int reduct_effort,
                          const vertex_set& ignore_ops)
{
    contin_reduction(reduct_effort, ignore_ops)(tr);
}

inline void fold_reduce(combo_tree& tr, combo_tree::iterator it)
{
    fold_reduction()(tr, it);
}

inline void fold_reduce(combo_tree& tr)
{
    fold_reduction()(tr);
}

inline void mixed_reduce(combo_tree& tr, combo_tree::iterator it)
{
    mixed_reduction()(tr, it);
}

inline void mixed_reduce(combo_tree& tr)
{
    mixed_reduction()(tr);
}

/**
 * reduce trees containing mixtures of logical operators, boolean
 * constants, boolean-valued predicates, boolean-valued literals,
 * arithmetic operators, contin-typed constants, and contin-valued
 * literals.  (Elsewhere, we call these "predicate trees").
 */
inline void full_reduce(combo_tree& tr, combo_tree::iterator it)
{
    full_reduction()(tr, it);
}

inline void full_reduce(combo_tree& tr)
{
    full_reduction()(tr);
}

inline void ann_reduce(combo_tree& tr)
{
    ann_reduction()(tr);
}

/**
 * clean_reduce removes null vertices
 */
inline void clean_reduce(combo_tree& tr,combo_tree::iterator it)
{
    clean_reduction()(tr, it);
}

inline void clean_reduce(combo_tree& tr)
{
    clean_reduction()(tr);
}

inline void clean_and_full_reduce(combo_tree& tr,
                                  combo_tree::iterator it)
{
    // clean_and_full_reduction()(tr,it);
    clean_reduce(tr, it);
    full_reduce(tr, it);
}

inline void clean_and_full_reduce(combo_tree& tr)
{
    // clean_and_full_reduction()(tr,tr.begin());
    clean_reduce(tr);
    full_reduce(tr);
}

//action
inline void action_reduce(combo_tree& tr, combo_tree::iterator it)
{
    action_reduction()(tr, it);
}

inline void action_reduce(combo_tree& tr)
{
    action_reduction()(tr);
}

//perception
inline void perception_reduce(combo_tree& tr, combo_tree::iterator it)
{
    perception_reduction()(tr, it);
}

inline void perception_reduce(combo_tree& tr)
{
    perception_reduction()(tr);
}

// helper to replace a subtree by another subtree (belonging to the
// same or another tree without changing the iterator of the new tree,
// this is because the reduct engine assumes that the rules do not
// change the iterator they take in argument. That is dst is not
// changed (expect possibly it's content of course).
// tr is the tree containing dst iterator
inline void replace_without_changing_it(combo_tree& tr,
                                        combo_tree::iterator dst,
                                        combo_tree::iterator src)
{
    *dst = *src;
    if (src.is_childless())
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
