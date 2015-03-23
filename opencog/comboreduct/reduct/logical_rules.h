/*
 * opencog/comboreduct/reduct/logical_rules.h
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
#ifndef _REDUCT_LOGICAL_RULES_H
#define _REDUCT_LOGICAL_RULES_H

#include <opencog/util/exceptions.h>

#include "reduct.h"
#include "flat_normal_form.h"
#include "../type_checker/type_tree.h"

namespace opencog { namespace reduct {

/// Ensure that all arguments and 'or' nodes have an 'and' node as their parent.
/// This is important so that other normalizations can catch all cases.
struct insert_ands : public crule<insert_ands>
{
    insert_ands() : crule<insert_ands>::crule("insert_ands") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

/// Remove any logical operators that have only one child.
/// (i.e. raise up the child in the tree).
struct remove_unary_junctors : public crule<remove_unary_junctors>
{
    remove_unary_junctors()
        : crule<remove_unary_junctors>::crule("remove_unary_junctors") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

/// Simplify any predicate terms that are found in the tree.
///
/// Predicate terms are boolean-valued functions whose arguments
/// are not boolen.  Currently, the only predicate we have is
/// greater_than_zero. This rule will simplify the contin-valued
/// arguments of all such predicates.
///
struct simplify_predicates : public crule<simplify_predicates>
{
    simplify_predicates(int effort,
                        const vertex_set& ignore_ops_)
        : crule<simplify_predicates>::crule("simplify_predicates"),
          reduct_effort(effort), ignore_ops(ignore_ops_) {}

    simplify_predicates(const simplify_predicates& rhs)
        : crule<simplify_predicates>::crule("simplify_predicates"),
          reduct_effort(rhs.reduct_effort), ignore_ops(rhs.ignore_ops) {}

    void operator()(combo_tree& tr, combo_tree::iterator it) const;

protected:
    int reduct_effort;
    const vertex_set& ignore_ops;
};

/// Remove operators 'and', 'or' and 'not' that have no argument. Note
/// that this rule does not preserve semantics it is used so that
/// logical formulae exibit more diversity
struct remove_dangling_junctors : public crule<remove_dangling_junctors>
{
    remove_dangling_junctors() : crule<remove_dangling_junctors>::crule("remove_dangling_junctors") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

// and(true X)->X, or(true X)->true
// and(false X)->false, or(false X)->X
// or(X)->X
// and(or X) -> and(X), or(and X) -> or(X)
struct eval_logical_identities : public crule<eval_logical_identities>
{
    eval_logical_identities()
        : crule<eval_logical_identities>::crule("eval_logical_identities") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

// !!a->a,!(a&&b)->(!a||!b),!(a||b)->(!a&&!b),
struct reduce_nots : public crule<reduce_nots>
{
    reduce_nots() : crule<reduce_nots>::crule("reduce_nots") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

//and(x_1 x_2 x_3 ...)
//reduce x_1 assuming x_2, x_3 ...
//reduce x_2 assuming x_1, x_3 ...
//...
//and choose the one that shorten the most the expression
struct reduce_and_assumptions : public crule<reduce_and_assumptions>
{
    reduce_and_assumptions(const rule& r)
        : crule<reduce_and_assumptions>::crule("reduce_and_assumptions"),
         _reduction(&r) { }

    reduce_and_assumptions()
        : crule<reduce_and_assumptions>::crule("reduce_and_assumptions"),
        _reduction(this) { }

    void operator()(combo_tree& tr,combo_tree::iterator it) const;

protected:
    const rule* _reduction;
};

// reduce or(x_1 x_2 x_3 ...) by reducing to not(and(not(x_1)...))
// using the reduce_and_assumptions
struct reduce_or_assumptions : public crule<reduce_or_assumptions>
{
    reduce_or_assumptions(const rule& r)
        : crule<reduce_or_assumptions>::crule("reduce_or_assumptions"),
         _reduction(&r) { }

    reduce_or_assumptions()
        : crule<reduce_or_assumptions>::crule("reduce_or_assumptions"),
         _reduction(this) { }

    void operator()(combo_tree& tr, combo_tree::iterator it) const;

protected:
    const rule* _reduction;
};

//heuristic reduction of ORs based on complementary pairs:
// 1) pairwise implications of conjuncts (X&&a)||((Y&&!a) impl  X&&Y
// 2) for all pairs of conjuncts, including implications (in X)
//    if X is a subset of (or equal to) Y, remove Y
// Also, true||X -> true, false||X -> X
struct reduce_ors : public crule<reduce_ors>
{
    reduce_ors() : crule<reduce_ors>::crule("reduce_ors") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//heuristic reduction of ANDs based on complementary pairs:
// 1) pairwise implications of conjuncts (X||a)&&((Y||!a) impl (X||Y)
// 2) for all pairs of conjuncts, including implications (in X)
//    if X is a subset of (or equal to) Y, remove X
// Also, true&&X -> X, false&&X -> false
struct reduce_ands : public crule<reduce_ands>
{
    reduce_ands() : crule<reduce_ands>::crule("reduce_ands") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

/// Reduction to enf - reduce to elegant normal form.
/// Note that reduce_nots() and eval_logical_identities() and level()
/// should be called first to ensure proper reduction.
//
struct subtree_to_enf : public crule<subtree_to_enf>
{
    subtree_to_enf()
        : crule<subtree_to_enf>::crule("subtree_to_enf") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const
    {
        reduce_to_enf(tr, it);
    }

protected:
    struct reduce_to_enf
    {
        enum Result { Delete, Disconnect, Keep };
        typedef vertex value_type;
        typedef combo_tree::sibling_iterator sib_it;
        typedef combo_tree::upwards_iterator up_it;
        typedef opencog::lexicographic_subtree_order<value_type> Comp;
        typedef std::set<sib_it,Comp> subtree_set;

        combo_tree& tr;
        Comp comp;

        reduce_to_enf(combo_tree& tr_, combo_tree::iterator it)
            : tr(tr_)
        {
             (*this)(it);
        }

        void operator()(sib_it it);
        bool consistent(const subtree_set& s);
        bool and_cut(sib_it child);
        void or_cut(sib_it current);
        Result reduce(sib_it current,const subtree_set& dominant,
                      const subtree_set& command);
        Result reduce_and(sib_it current,const subtree_set& dominant,
                          const subtree_set& command);
        Result reduce_or(sib_it current,const subtree_set& dominant,
                         const subtree_set& command);

        template<typename Out>
        void build_subtree_sets_upwards(up_it up,Out dom_out,Out cmd_out) const
        {
            static const type_tree boolean_type_tree = type_tree(id::boolean_type);

            for (up_it p = tr.parent(up);
                 p != tr.end_upwards() && is_logical_operator(*p); ++p) {
                if (*p == id::logical_and) {
                    for (sib_it sib = p.begin(); sib != p.end(); ++sib)
                        if (sib != up)
                            *dom_out++ = sib;
                } else {
                    OC_ASSERT(*p == id::logical_or);
                    for (sib_it sib = p.begin(); sib != p.end(); ++sib)
                        if (sib != up && is_argument(*sib))
                            *cmd_out++ = sib;
                }
                up = p;
            }
        }

        template<typename Dst>
        void push_back_negated_arguments(const subtree_set& s,Dst& negated)
        {
            for (sib_it sib : s) {
                if (is_argument(*sib)) {
                    negated.push_back(combo_tree(sib));
                    get_argument(*negated.back().begin()).negate();
                }
            }
        }

        struct tree_eraser
        {
            tree_eraser(combo_tree& t) : tr(t) { }
            combo_tree& tr;
            template<typename It>
            void operator()(It it) { tr.erase(*it); }
        };

        struct subtree_set_eraser
        {
            subtree_set_eraser(subtree_set& s) : se(s) { }
            subtree_set& se;
            template<typename It>
            void operator()(It it) { se.erase(it); }
        };

        struct tree_inserter
        {
            tree_inserter(combo_tree& t,sib_it i) : tr(t),it(i) { }
            combo_tree& tr;
            sib_it it;
            template<typename It1,typename It2>
            void operator()(It1 it1, It2 it2)
            {
                if (it1->node == 0)
                    tr.append_child(it, it2);
                else
                    tr.insert_subtree(*it1, it2);
            }
        };
    };
};

// (suggested by Moshe) check if the truth table is the same after
// removal of any subtree
struct reduce_remove_subtree_equal_tt
    : public crule<reduce_remove_subtree_equal_tt>
{
    reduce_remove_subtree_equal_tt()
        : crule<reduce_remove_subtree_equal_tt>::crule("reduce_remove_subtree_equal_tt") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

} // ~namespace reduct
} // ~namespace opencog

#endif
