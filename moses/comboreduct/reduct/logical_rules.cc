/*
 * opencog/comboreduct/reduct/logical_rules.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings
 * All Rights Reserved
 *
 * Written by Moshe Looks
 * Add boolean-valued predicate support -- Linas Vepstas
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
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/indirect_iterator.hpp>

#include <opencog/util/exceptions.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/comboreduct/combo/assumption.h>
#include "logical_rules.h"
#include "../table/table.h"

namespace opencog { namespace reduct {
typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::iterator pre_it;

using boost::make_counting_iterator;

// Ensure that all arguments and 'or' nodes have an 'and' node as their
// parent.  This is needed, so that other normalizations can catch all
// cases.
void insert_ands::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if ((is_argument(*it) || *it == id::logical_or) &&
        // If it's got a parent, then it better be something boolean
        // valued.  We don't want to insert ands in front of contin
        // valued args... 
        (!tr.is_valid(tr.parent(it)) || 
         *tr.parent(it) == id::logical_or ||
         *tr.parent(it) == id::impulse ||
         *tr.parent(it) == id::logical_not) )
    {
        tr.prepend_child(it, *it);
        *it = id::logical_and;
        tr.reparent(it.begin(), ++it.begin(), it.end());
    }
}

// If a predicate is found, then simplify it.
// At this time, there is only one predicate: gt_zero, whose
// argument is a contin. Thus, simplify the contin.
void simplify_predicates::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (!is_predicate(it)) return;

    if (*it == id::logical_not)
          it = it.begin();

    // it now points to a the predicate, with any leading logical not
    // stripped off. it.begin() points to the contin-valued expression.
    combo_tree::iterator cit = it.begin();
    if (is_argument(*cit)) return;

    contin_reduce(tr, cit, reduct_effort, ignore_ops);

    // After the above step, we can still have some pathological
    // expressions, such as "0<(1)" which can be reduced to true,
    // or "0<(/(1 $1))",  which can be reduced to "0<($1)" .. so do these.
    mixed_reduce(tr, it);
}

void remove_unary_junctors::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if ((*it == id::logical_and || *it == id::logical_or)
        && it.has_one_child())
    {
        *it = *it.begin();
        tr.erase(tr.flatten(it.begin()));
    }
}

void remove_dangling_junctors::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    // Most nodes take simple lists; but not cond. Cond takes clauses,
    // which are pairs. If we remove the condition, we must also remove
    // the consequent.
// XXX TODO: I don't understand why this is not damaging contin_if  !??
// But .. umm, maybe build_knobs is not creating any kinds of contin_if's
// that can be damaged... well, no matter, because thes if's will be
// replaced by cond... 
    if (*it != id::cond) {
        for (sib_it sib = it.begin(); sib != it.end(); )
            if ((*sib == id::logical_and ||
                 *sib == id::logical_or ||
                 *sib==id::logical_not) && sib.is_childless())
                tr.erase(sib++);
            else
                ++sib;
    } else {
        for (sib_it sib = it.begin(); sib != it.end(); )
            if ((*sib == id::logical_and ||
                 *sib == id::logical_or ||
                 *sib==id::logical_not) && sib.is_childless()) {
                tr.erase(sib++);
                tr.erase(sib++);
            } else {
                ++sib;
                ++sib;
            }
    }
}

/// and(true X)->X,  or(true X)->true
/// and(false X)->false, or(false X)->X
/// or(X)->X
/// and(or X) -> and(X), or(and X) -> or(X)
void eval_logical_identities::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it == id::logical_and) {
        for (sib_it sib = it.begin(); sib != it.end(); ) {
            if ((*sib == id::logical_or && sib.is_childless())
                || *sib == id::logical_true)
            {
                sib = tr.erase(sib);
            }
            else if (*sib == id::logical_false) {
                *it = *sib;
                tr.erase_children(it);
                break;
            } else {
                ++sib;
            }
        }
    }
    else if (*it == id::logical_or) {
        if (it.has_one_child()) {
            *it = *it.begin();
            tr.erase(tr.flatten(it.begin()));
        }
        else {
            for (sib_it sib = it.begin(); sib != it.end(); ) {
                if ((*sib == id::logical_and && sib.is_childless())
                    || *sib == id::logical_false)
                {
                    sib = tr.erase(sib);
                }
                else if (*sib == id::logical_true)
                {
                    *it = *sib;
                    tr.erase_children(it);
                    break;
                }
                else
                {
                    ++sib;
                }
            }
        }
    }
}

//* reduce nots: apply distributive law to push not to children.
//* !!a->a,   !(a&&b)->(!a||!b),   !(a||b)->(!a&&!b),
void reduce_nots::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it == id::logical_not) {
        if (*it.begin() == id::logical_not) {
            //!!a->!a
            tr.erase(tr.flatten(it.begin()));
            //!a->a
            *it = *it.begin();
            tr.erase(tr.flatten(it.begin()));
            operator()(tr,it);
        }
        else if (*it.begin() == id::logical_and ||
                   *it.begin() == id::logical_or) {
            // !(a&&b) -> (!a||!b), and !(a||b) -> (!a&&!b)
            // first transform to comp_op(a,b)
            tr.flatten(it.begin());
            *it = (*it.begin() == id::logical_and ?
                 id::logical_or :
                 id::logical_and);
            tr.erase(it.begin());
            // now to comp_op(!a,!b)
            for (sib_it sib = it.begin(); sib != it.end(); ++sib)
                sib=tr.insert_above(sib,id::logical_not);
        }
        else if (is_argument(*it.begin())) {
            *it = *it.begin();
            tr.erase_children(it);
            get_argument(*it).negate();
        }
        // These two never occur in ordinary logical expressions, but
        // do show up when reducing mixed predicates (i.e. when the
        // predicate has reduced to a simple true/false).
        else if (*it.begin() == id::logical_true) {
            tr.erase_children(it);
            *it = id::logical_false;
        }
        else if (*it.begin() == id::logical_false) {
            tr.erase_children(it);
            *it = id::logical_true;
        }
    }
}


// and(x_1 x_2 x_3 ...)
// reduce x_1 assuming x_2, x_3 ...
// reduce x_2 assuming x_1, x_3 ...
// ...
// and choose the one that shortens the expression the most.
void reduce_and_assumptions::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    int best_diff = 0;
    int best_index = -1;
    combo_tree best_tree;
    if(*it==id::logical_and) {
        for(sib_it sib = it.begin(); sib != it.end(); ++sib) {
            //copy the tree to reduce
            combo_tree copy_tr = tr.subtree(sib, tr.next_sibling(sib));
            //copy old assumptions
            sib_it bna = copy_tr.begin(); //before new assumption
            for(sib_it a = tr.next_sibling(tr.begin()); a != tr.end(); ++a)
                bna = copy_tr.insert_subtree_after(bna, a);
            //add new assumptions
            for(sib_it a = it.begin(); a != it.end(); ++a) {
                if(a!=sib)
                    insert_assumption(copy_tr, pre_it(a));
            }
            //subtree_size on copy_tr.begin() to not add size of assumptions
            int s_before = copy_tr.subtree_size(copy_tr.begin());
            (*_reduction)(copy_tr);
            //subtree_size on copy_tr.begin() to not add size of assumptions
            int s_after = copy_tr.subtree_size(copy_tr.begin());
            int diff = s_after - s_before;
            if(diff < best_diff) {
                best_diff = diff;
                best_index = tr.sibling_index(sib);
                best_tree = copy_tr;
            }
        }
        //replace by the best reduction if there is
        if(best_index>-1)
            tr.replace(tr.child(it, best_index), best_tree.begin());
    }
}

//reduce or(x_1 x_2 x_3 ...) by reducing assuming not(and(not(x_1)...))
//using the reduce_and_assumptions
void reduce_or_assumptions::operator()(combo_tree& tr,combo_tree::iterator it) const
{
    if(*it==id::logical_or) {
        OC_ASSERT(!it.is_childless(), "combo_tree node should not be childless");
        *it = id::logical_not;
        pre_it new_or = tr.prepend_child(it, id::logical_or);
        tr.reparent(new_or, tr.next_sibling(it.begin()), it.end());
        pre_it first_not = tr.wrap(new_or, id::logical_not);
        (*_reduction)(tr, first_not);
        //to avoid an infinit recursion not(and(x y...) is reduced to or(not(x) ...)
        //then all not(x), not(y) ... are reduced
        reduce_nots()(tr, it);
        for(sib_it sib = it.begin(); sib != it.end(); ++sib) {
            (*_reduction)(tr, pre_it(sib));
        }
    }
}

/// Heuristic reduction of ORs based on complementary pairs:
/// 1) pairwise implications of conjuncts (X && a) || ((Y && !a) impl  X && Y
/// 2) for all pairs of conjuncts, including implications (in X)
///    if X is a subset of (or equal to) Y, remove Y
/// Also, true||X -> true, false||X -> X
void reduce_ors::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it != id::logical_or)
        return;

    // First construct a mapping between items and ints,
    // and convert all conjunctions to this format.
    nf_mapper<vertex> mapper;
    nf f(mapper.add_dnf(tr.begin(it), tr.end(it)));
    int sz = number_of_literals(f);

    // Before this method will work, need to eliminate A && !A clauses.
    f.remove_if(bind(tautology, _1));

    // Remove clauses which are subsets of others.
    pairwise_erase_if(f, bind(subset_eq, _1, _2));

    // Create implications (and remove subsets).  We could be cleverer
    // by trying implications with implications, and/or not updating
    // right a way, but it makes things more complicated and in practice
    // doesn't seem to lead to appreciable further reduction.
    for (nf::iterator c1 = f.begin(); c1 != f.end(); ++c1) {
        for (nf::iterator c2 = f.begin(); c2 != c1; ++c2) {
            nf impls;
            implications(*c1, *c2, std::back_inserter(impls));

            for (nf::const_iterator impl = impls.begin();
                 impl != impls.end(); ++impl) {
                for (nf::iterator c = f.begin(); c != f.end();)
                    if (c != c1 && c != c2 && subset_eq(*impl, *c))
                        c = f.erase(c);
                    else
                        ++c;
                if (subset(*impl, *c1))
                    *c1 = *impl;
                if (subset(*impl, *c2))
                    *c2 = *impl;
            }
        }
    }

    if (sz != number_of_literals(f))
        mapper.extract_dnf(f.begin(), f.end(), tr, it);
}

/// heuristic reduction of ANDs based on complementary pairs:
/// 1) pairwise implications of conjuncts (X||a)&&((Y||!a) impl (X||Y)
/// 2) for all pairs of conjuncts, including implications (in X)
///    if X is a subset of (or equal to) Y, remove X
/// Also, true&&X -> X, false&&X -> false
void reduce_ands::operator()(combo_tree& tr, combo_tree::iterator it) const
{
    if (*it != id::logical_and)
        return;

    // First construct a mapping between items and ints,
    // and convert all conjunctions to this format.
    nf_mapper<vertex> mapper;
    nf f(mapper.add_cnf(tr.begin(it), tr.end(it)));
    int sz = number_of_literals(f);

    // negate
    for (nf::iterator c = f.begin(); c != f.end(); ++c) {
        clause tmp;
        std::transform(c->begin(), c->end(), std::inserter(tmp, tmp.begin()),
                       std::negate<int>());
        *c=tmp;
    }

    // Before this method will work, need to eliminate A&&!A clauses.
    f.remove_if(bind(tautology, _1));

    if (f.empty()) { //tautological expression
        tr.erase_children(it);
        *it = id::logical_true;
        return;
    }

    // Remove clauses which are supersets of others.
    pairwise_erase_if(f, bind(subset_eq, _1, _2));

    // Create implications (and remove subsets).
    for (nf::iterator c1 = f.begin(); c1 != f.end(); ++c1) {
        for (nf::iterator c2 = f.begin(); c2 != c1; ++c2) {
            nf impls;
            implications(*c1, *c2, std::back_inserter(impls));
            for (nf::const_iterator impl = impls.begin();
                 impl != impls.end(); ++impl)
            {
                for (nf::iterator c = f.begin(); c != f.end(); )
                    if (c != c1 && c != c2 && subset_eq(*impl, *c))
                        c = f.erase(c);
                    else
                        ++c;
                if (subset(*impl, *c1))
                    *c1 = *impl;
                if (subset(*impl, *c2))
                    *c2 = *impl;
            }
        }
    }

    clause intersect;
    if (f.size()>1 && f.front().size()>1) {
        intersect=f.front();
        for (nf::const_iterator c=++f.begin();c!=f.end() && !intersect.empty();
             ++c) {
            if (c->size()==1) {
                intersect.clear();
                break;
            }
            clause tmp;
            std::set_intersection(intersect.begin(),intersect.end(),
                                  c->begin(),c->end(),inserter(tmp,tmp.begin()),
                                  intersect.key_comp());
            intersect=tmp;
        }
    }

    if (sz!=number_of_literals(f) || !intersect.empty()) {
        for (nf::iterator c=f.begin();c!=f.end();++c) {
            for (clause::const_iterator it=intersect.begin();
                 it!=intersect.end();++it)
                c->erase(*it);

            clause tmp;
            std::transform(c->begin(),c->end(),std::inserter(tmp,tmp.begin()),
                           std::negate<int>());
            *c=tmp;
        }

        clause tmp;
        std::transform(intersect.begin(),intersect.end(),
                       std::inserter(tmp,tmp.begin()),std::negate<int>());
        intersect=tmp;

        mapper.extract_cnf(f.begin(),f.end(),tr,it);
    }

    if (!intersect.empty()) {
        *it=id::logical_or;
        if (!it.is_childless()) {
            tr.prepend_child(it,id::logical_and);
            tr.reparent(it.begin(),++it.begin(),it.end());
        }
        mapper.extract_conjunction(intersect,tr,tr.append_child(it));
        if (intersect.size()>1)
            tr.erase(tr.flatten(it.last_child()));
    }
}

void subtree_to_enf::reduce_to_enf::operator()(sib_it it)
{
    static const type_tree boolean_type_tree = type_tree(id::boolean_type);
    if (!is_logical_operator(*it))
        return;

    tr.sort_on_subtrees(it.begin(), it.end(), comp, true);
    for (up_it p = ++tr.begin_upwards(it);
         p != tr.end_upwards() && is_logical_operator(*p); ++p)
    {
        if (!opencog::is_sorted(make_counting_iterator(p.begin()),
                                make_counting_iterator(p.end()), comp))
            tr.sort_on_subtrees(p.begin(), p.end(), comp);
    }
    subtree_set dominant, command;
    build_subtree_sets_upwards(it,
                               inserter(dominant, dominant.end()),
                               inserter(command, command.end()));

    switch (reduce(it, dominant, command))
    {
    case Delete: // a contradiction
        tr.erase_children(it);
        *it = id::logical_false;
        break;
    case Disconnect: // a tautology
        tr.erase_children(it);
        *it = id::logical_true;
        break;
    case Keep: // neither
        break;
    }
}

bool subtree_to_enf::reduce_to_enf::consistent(const subtree_set& s)
{
    return std::adjacent_find(boost::make_indirect_iterator(s.begin()),
                              boost::make_indirect_iterator(s.end()),
                              is_complement) == boost::make_indirect_iterator(s.end());
}

bool subtree_to_enf::reduce_to_enf::and_cut(sib_it child)
{
    bool adopted = false;
    for (sib_it gchild = child.begin(); gchild != child.end(); )
    {
        // OK. We expect each of the gchilds to be either a logic op,
        // or an argument, or a (possibly negated) predicate.  If its
        // either of the later two, do nothing, and look at the next
        // gchild. But if gchild is a logic op, and that logic op has
        // just one child, then flatten and pull it up to our level.
        if (is_logical_operator(*gchild))
        {
            // Well, if may have one child, and that child is the 
            // logical_not in front of a predicate. Rule that out.
            if (gchild.has_one_child() &&
                (*gchild == id::logical_and || *gchild == id::logical_or))
            {
                if (*gchild.begin() == id::logical_or)
                {
                    tr.erase(tr.flatten(gchild.begin()));
                    if (!adopted) // is child adopting a terminal 1-constrant AND, x?
                    {
                        for (sib_it x = gchild.begin(); x != gchild.end(); ++x)
                            if (x.has_one_child()) {
                                adopted = true;
                                break;
                            }
                    }
                }

                // Allow predicates; allow arguments, complain about
                // anything else. Predicates will be reduced below;
                // we don't need to reduce them here.
                else if (!is_predicate(gchild.begin()) &&
                         !is_argument(*gchild.begin()))
                {
                    std::stringstream ss;
                    ss << "Logical reduction: unexpected operator: ";
                    ss << *gchild.begin();
                    OC_ASSERT(0, ss.str());
                }
                gchild = tr.erase(tr.flatten(gchild));
                continue;
            }
        }
        else if (!is_argument(*gchild) &&
                 !is_predicate(gchild))
        {
            std::stringstream ss;
            ss << "Logical reduction: unexpected operator: ";
            ss << *gchild;
            OC_ASSERT(0, ss.str());
        }
        ++gchild;
    }
    return adopted;
}

void subtree_to_enf::reduce_to_enf::or_cut(sib_it current)
{
    for (sib_it child = current.begin(); child != current.end(); )
    {
        if (child.has_one_child() &&
            (*child == id::logical_and || *child == id::logical_or))
        {
            if (*child.begin() == id::logical_or)
            {
                opencog::insert_set_complement
                    (tree_inserter(tr,current),
                     make_counting_iterator(current.begin()),
                     make_counting_iterator(current.end()),
                     make_counting_iterator(child.begin().begin()),
                     make_counting_iterator(child.begin().end()),comp);
                tr.erase(tr.flatten(child.begin()));
                child = tr.erase(tr.flatten(child));
            }
            // If its a boolean-type literal (well, is_argument doesn't
            // actually check the argument type, but it should ...)
            // or if it is a boolean-valued term (i.e. a predicate)
            // then just pull it out from under the logic op.
            else if (is_argument(*child.begin()) ||
                     is_predicate(child.begin()))
            {
                child = tr.erase(tr.flatten(child));
            }
            else
            {
                // If we are here, the child just has one child, and
                // that child is not an argument, not a predicate, and
                // is not logical_or. We conclude (without really
                // checking) that its a logical_and with a childless
                // logical_and under it.  So delete it.  Yes, this
                // actually happens, in the unit test cases.  Yes, the
                // above chain of assumptions seems dangerous to me ...
                child = tr.erase(child);
            }
        }
        else {
            ++child;
        }
    }
}

subtree_to_enf::reduce_to_enf::Result
subtree_to_enf::reduce_to_enf::reduce(sib_it current,
                                      const subtree_set& dominant,
                                      const subtree_set& command)
{
#if DEBUG
    // For performance, skip this check ...
    OC_ASSERT(opencog::is_sorted(dominant.begin(), dominant.end(), comp),
              "dominant subtree_set should be sorted (reduce)");
#endif

    // First, remove duplicate children.  This loop assumes that
    // current is in sorted order, and thus, duplicate children are
    // next to each other. This allows a single O(n) loop instead
    // of a nested O(N^2) loop.
    if (!current.is_childless()) {
        for (sib_it sib1 = current.begin(), sib2 = ++current.begin();
             sib2 != current.end(); )
        {
            if (tr.equal_subtree(sib1, sib2)) {
                sib2 = tr.erase(sib2);
            } else {
                sib1 = sib2;
                ++sib2;
            }
        }
    }

#if DEBUG
    // We skip this to improve performance.
    OC_ASSERT(opencog::is_sorted(dominant.begin(), dominant.end(), comp),
              "dominant subtree_set should be sorted (reduce)");
    OC_ASSERT(opencog::is_sorted(command.begin(), command.end(), comp),
              "command subtree_set should be sorted (reduce).");
#endif

    if (*current == id::logical_and)
        return reduce_and(current, dominant, command);
    if (*current == id::logical_or)
        return reduce_or(current, dominant, command);

    // Should be an argument or something ... ??
    //OC_ASSERT(is_argument(*current));
    return Keep;
}

subtree_to_enf::reduce_to_enf::Result
subtree_to_enf::reduce_to_enf::reduce_and(sib_it current,
                                          const subtree_set& dominant,
                                          const subtree_set& command)
{
    opencog::erase_set_intersection(tree_eraser(tr),
                                    make_counting_iterator(current.begin()),
                                    make_counting_iterator(current.end()),
                                    dominant.begin(), dominant.end(), comp);


    std::vector<combo_tree> negated;
    push_back_negated_arguments(command, negated);
    std::sort(negated.begin(), negated.end(), comp);

    opencog::erase_set_intersection(tree_eraser(tr),
                                    make_counting_iterator(current.begin()),
                                    make_counting_iterator(current.end()),
                                    negated.begin(),negated.end(),comp);

    if (current.is_childless())
        return Disconnect; //0subsume

    if (!opencog::has_empty_intersection
        (make_counting_iterator(current.begin()),
         make_counting_iterator(current.end()),
         command.begin(),command.end(),comp))
        return Delete;     //1subsume

    std::list<sib_it> prev_guard_set;
    do {
        prev_guard_set=
            std::list<sib_it>(make_counting_iterator(current.begin()),
                              make_counting_iterator(current.end()));

#if DEBUG
        // stub out, for performance.
        OC_ASSERT(opencog::is_sorted(dominant.begin(),dominant.end(), comp),
                  "dominant subtree_set should be sorted (reduce_and)");
#endif

        if (!opencog::is_sorted(make_counting_iterator(current.begin()),
                                make_counting_iterator(current.end()), comp))
            tr.sort_on_subtrees(current.begin(), current.end(), comp);

        subtree_set handle_set; //broom handle
        std::set_union(dominant.begin(),dominant.end(),
                       make_counting_iterator(current.begin()),
                       make_counting_iterator(current.end()),
                       inserter(handle_set, handle_set.begin()), comp);

        if (!consistent(handle_set)) {
            return Delete;
        }

        for (sib_it child = current.begin(); child != current.end(); ) {
            if (child.is_childless() || (*child != id::logical_and &&
                                         *child != id::logical_or)) {
                ++child;
                continue;
            }

            tr.validate(child);
            tr.validate();

#if DEBUG
            // stubbed out for performance
            OC_ASSERT(opencog::is_sorted(command.begin(),command.end(),comp),
                      "command subtree_set should be sorted (reduce_and)");
            OC_ASSERT(opencog::is_sorted(handle_set.begin(),handle_set.end(),comp),
                      "handle_set subtree_set should be sorted (reduce_and)");
#endif
            subtree_set::iterator tmp_it = handle_set.find(child);
            sib_it tmp;
            bool addIt = true;
            if (tmp_it != handle_set.end()) {
                tmp = *tmp_it;
                handle_set.erase(tmp_it);
            } else {
                addIt = false;
            }

            switch(reduce(child, handle_set, command)) {
            case Delete:
                return Delete; //since current is in all selections with child
            case Disconnect:
                child = tr.erase(child);
                addIt = false;
                break;
            case Keep:

                if (!opencog::is_sorted(make_counting_iterator(current.begin()),
                                        make_counting_iterator(current.end()),
                                        comp))
                    tr.sort_on_subtrees(current.begin(), current.end(), comp);

                OC_ASSERT(child.begin() != child.end(),
                          "child should have siblings"); //not sure if this is ok..

                // Make res the disjunction of child's children's guard sets
                // important: we use references to the last child because we are
                // later going to iterate (left-to-right) removing these iterators
                // from the tree - unless we have references *only* to the
                // last-to-be-removed items, res may end up containing iterators
                // pointing at tree nodes that already have been removed

                // The last_child won't have children if its an argument.
                // Treat predicates as if they were arguments too: they're
                // also effectively "childless".
                subtree_set res;
                if (!is_predicate(child.last_child())) {
                    for (sib_it pit = child.last_child().begin();
                         pit != child.last_child().end(); ++pit)
                    {
                        res.insert(pit);
                    }
                }

                for (sib_it sib = child.begin(); sib != child.last_child(); ++sib)
                    opencog::erase_set_difference(subtree_set_eraser(res),
                                                  res.begin(), res.end(),
                                                  make_counting_iterator(sib.begin()),
                                                  make_counting_iterator(sib.end()),
                                                  comp);

                if (!res.empty())
                {
                    opencog::insert_set_complement
                        (tree_inserter(tr, current),
                         make_counting_iterator(current.begin()),
                         make_counting_iterator(current.end()),
                         res.begin(), res.end(), comp);

                    for (sib_it gchild = child.begin(); gchild != child.end(); ++gchild)
                    {
                        // Do NOT erase children of predicates!
                        if (is_predicate(gchild)) continue;
                        opencog::erase_set_intersection
                            (tree_eraser(tr),
                             make_counting_iterator(gchild.begin()),
                             make_counting_iterator(gchild.end()),
                             res.begin(), res.end(), comp);
                    }

                    // Try to apply and-cut to child's children.
                    and_cut(child);

                    // guard_set has been enlarged, so we need to reprocess all kids
                    child = current.begin();
                } else {
                    if (!and_cut(child)) //if child is adopting a terminal
                        ++child;           //1-constrant AND, need to reprocess it
                }
            }
            if (addIt)
                handle_set.insert(tmp);
        }

        // try to apply or-cut to current's children
        or_cut(current);
    } while (current.number_of_children() != prev_guard_set.size() ||
             !std::equal(prev_guard_set.begin(), prev_guard_set.end(),
                         make_counting_iterator(current.begin())));

    if (current.is_childless())
        return Disconnect; //0-subsumption
    return Keep;
}

subtree_to_enf::reduce_to_enf::Result
subtree_to_enf::reduce_to_enf::reduce_or(sib_it current,
                                         const subtree_set& dominant,
                                         const subtree_set& command)
{
    for (sib_it child = current.begin(); child != current.end(); )
    {
        subtree_set child_command(command);
        for (sib_it sib = current.begin(); sib != current.end(); ++sib)
        {
            if (sib != child && sib.has_one_child() && !is_predicate(sib))
            {
                child_command.insert(sib.begin());
            }
        }

        switch(reduce(child, dominant, child_command))
        {
        case Delete:
            if (current.is_childless() || current.has_one_child())
                return Delete;
            else
                child = tr.erase(child);
            break;
        case Disconnect:
            return Disconnect;
        case Keep:
            ++child;
        }
    }
    return Keep;
}

void reduce_remove_subtree_equal_tt::operator()(combo_tree& tr,
                                                combo_tree::iterator it) const
{
    // Cannot construct complete truth tables of expressions containing
    // predicates, since arguments of predicates may range over contin
    // values, and who knows what those might be.
    for (pre_it pit = tr.begin(); pit != tr.end(); ++pit) {
        if (is_predicate(pit))
            return;
    }

    complete_truth_table tr_tt(tr);
    for (pre_it pit = it.begin(); pit != it.end();) {
        pit = tr.insert_above(pit, id::null_vertex);
        combo_tree amputated_tr(tr);
        clean_reduce(amputated_tr);
        if (tr_tt.same_complete_truth_table(amputated_tr)) // amputate tr
            pit = tr.erase(pit);
        else { // remove only the previously added null_vertex
            pit = tr.erase(tr.flatten(pit));
            ++pit;
        }
    }
}

} // ~namespace reduct
} // ~namespace opencog

