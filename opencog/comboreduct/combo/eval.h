/*
 * opencog/comboreduct/combo/eval.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
 *            Moshe Looks
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
#ifndef _COMBO_EVAL_H
#define _COMBO_EVAL_H

#include <opencog/util/tree.h>
#include <opencog/util/numeric.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/foreach.h>
#include <opencog/util/mt19937ar.h>

#include <opencog/comboreduct/crutil/exception.h>
#include "vertex.h"
#include "type_tree.h"
#include "using.h"
#include "variable_unifier.h"

#include <exception>
#include <boost/unordered_map.hpp>

namespace opencog { namespace combo {

struct Evaluator {
    virtual ~Evaluator() { }
    virtual vertex eval_action(combo_tree::iterator, variable_unifier&) = 0;
    virtual vertex eval_percept(combo_tree::iterator, variable_unifier&) = 0;
    // @todo : this could be generic and not in the Evaluator
    // there would be a way to specify with the procedure is lazy or note
    // in order to be fully compatible with the way it is already used
    virtual vertex eval_procedure(combo_tree::iterator, variable_unifier&) = 0;
    // eval_indefinite_object takes no arguments because it is assumed
    // that it has no child, this assumption may change over time
    virtual vertex eval_indefinite_object(indefinite_object,
                                          variable_unifier&) = 0;
};

// there are 2 ways of binding input arguments to a combo_tree
//
// 1) associate the variable arguments $1, $2, etc with there values
// using a binding_map, and then eval will use that mapping to
// evaluate the variable arguments on the fly
//
// or
//
// 2) lazily - substituting directly the values in the combo_tree
// statically (be careful because it modifies the combo_tree) using
// set_bindings

// Associate the index of the argument (starting from 1) to the
// iterator of the combo tree to return. It returns a combo tree
// instead of a vertex to support lazy evaluation.
typedef boost::unordered_map<arity_t,
                             boost::variant<vertex,
                                            combo_tree::iterator> > binding_map;

// This binding is not thread-safe (because it is static)
inline boost::variant<vertex, combo_tree::iterator>& binding(int idx)
{
    static binding_map map;
    return map[idx];
}

// binding arguments to function calls
// That is it replaces all variable arguments ($1, $2, etc)
// by the provided arguments and append the implicit arguments at the
// childfree operators.
// explicit_arity corresponding to the highest argument idx, it is given
// because it is necessary to know it in order to append
// the implicit arguments to the free (without children) operators
void set_bindings(combo_tree& tr, combo_tree::iterator it,
                  const std::vector<vertex>&,
                  arity_t explicit_arity);
// like above but can bind arguments that are subtrees
// of arg_parent rather than vertex
void set_bindings(combo_tree& tr, combo_tree::iterator it,
                  combo_tree::iterator arg_parent,
                  arity_t explicit_arity);

// like above but it applies on the entire tree
// and explicit_arity is calculated automatically
void set_bindings(combo_tree& tr, const std::vector<vertex>&);
void set_bindings(combo_tree& tr, combo_tree::iterator arg_parent);

#define ALMOST_DEAD_EVAL_CODE 1
#if ALMOST_DEAD_EVAL_CODE
/// @todo that one should be removed and replaced by eval_throws_binding
/// Right now, as far as I can tell, only embodiment code does this.
/// I'm hoping that some embodiment re-write will make this go away.
/// Note, however, emobodiment seems to use the variable unifiers, so
/// I'm, not sure about that.
/// Anyway, the code below is no longer maintained, and is missing 
/// support for newer & better stuff.
template<typename It>
vertex eval_throws(It it, Evaluator* pe = NULL,
                   combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU())
    throw(EvalException, ComboException,
          AssertionException, std::bad_exception)
{

    // std::cout << "EVAL: " << combo_tree(it) << std::endl;

    // std::cout << "VU: " << vu.toString() << std::endl;

    typedef typename It::sibling_iterator sib_it;
    vertex& v = *it;

    if (const argument* a = boost::get<argument>(&v)) {
        int idx = a->idx;
        // assumption : when idx is negative the argument is necessary boolean
        if (idx > 0) {
            if (const vertex* v = boost::get<const vertex>(&binding(idx)))
                return * v;
            else
                return eval_throws(boost::get<combo_tree::iterator>(binding(idx)),
                                   pe, vu);
        } else {
            if (const vertex* v = boost::get<const vertex>(&binding(-idx)))
                return negate_vertex(*v);
            else
                return negate_vertex(eval_throws(boost::get<combo_tree::iterator>(binding(-idx)),
                                                 pe, vu));
        }
    }
    // builtin
    else if (const builtin* b = boost::get<builtin>(&v)) {
        switch (*b) {
            // boolean operators
        case id::logical_true :
            return v;
        case id::logical_false :
            return v;
        case id::logical_and :
            for (sib_it sib = it.begin();sib != it.end();++sib) {

                if (vu.empty()) { // no wild_card case
                    if (eval_throws(sib, pe, vu) == id::logical_false) {
                        return id::logical_false;
                    }
                } else { // wild_card case
                    combo::variable_unifier work_vu(vu);
                    work_vu.setUpdated(false);
                    bool is_false = (eval_throws(sib, pe, work_vu)
                                     == id::logical_false); 
                    vu.unify(combo::UNIFY_AND, work_vu);
                    if(is_false) {
                        return id::logical_false;
                    }
                }
            }

            OC_ASSERT(vu.empty() || vu.isOneVariableActiveTMP(),
                      "Since it returns logical_true from that point"
                      " there should be at least one active variable");

            return id::logical_true;
        case id::logical_or :
            // default case, no wild card
            if (vu.empty()) {
                for (sib_it sib = it.begin();sib != it.end();++sib) {
                    if (eval_throws(sib, pe, vu) == id::logical_true) {
                        return id::logical_true;
                    }
                }
                return id::logical_false;                
            }
            // wild card case
            else {
                
                OC_ASSERT(vu.isOneVariableActiveTMP(),
                                  "the OR wild_card case relies on the fact"
                                  " that at least one variable is active");

                combo::variable_unifier work_vu(vu);
                work_vu.setUpdated(false);
                for (sib_it sib = it.begin();sib != it.end();++sib) {

                    combo::variable_unifier return_vu(work_vu);
                    return_vu.setUpdated(false);
                    bool res = vertex_to_bool(eval_throws(sib, pe, return_vu));

                    if(!return_vu.isUpdated()) {
                        if(res) { // then all activated entities are
                                  // necessarily valid
                            return id::logical_true;
                        }
                        else { // then all activated entities remains
                               // to be checked to which can directly
                               // go back to the next loop
                            continue;
                        }
                    }

                    work_vu.unify(combo::UNIFY_OR, return_vu);


                    // if no variable remains to be checked
                    // after OR unification it means that
                    // the unification has succeeded
                    // ASSUMING THAT unify has done actually something
                    // which is only the case if updated is true
                    if(!work_vu.isOneVariableActiveTMP()) {
                        // OC_ASSERT(res,
                        //           "res should be true because work_vu"
                        //           " should start the iteration with"
                        //           " at least one active variable");
                        vu.unify(combo::UNIFY_OR, work_vu);
                        return id::logical_true;
                    }
                }
                work_vu.setUpdated(true);
                vu.unify(combo::UNIFY_OR, work_vu);
                return (vu.isOneVariableActiveTMP()?
                        id::logical_true:
                        id::logical_false);
            }
        case id::logical_not : {
            OC_ASSERT(it.has_one_child(),
                      "logical_not should have exactly one child,"
                      " instead it has %u", it.number_of_children());
            if (vu.empty()) {
                return negate_vertex(eval_throws(it.begin(), pe, vu));
            }

            // variable unifier not empty, need to unify

            // eval variable unifier
            combo::variable_unifier work_vu(vu);
            work_vu.setUpdated(false);

            vertex vx = eval_throws(it.begin(), pe, work_vu);
            vu.unify(combo::UNIFY_NOT, work_vu);

            if (work_vu.isUpdated()) {
                return bool_to_vertex(vu.isOneVariableActive());
            }
            return negate_vertex(vx);
        }

        // conditional operators
        case id::contin_if : {
            OC_ASSERT(it.number_of_children() == 3,
                      "combo_tree node should have exactly three children"
                      " (id::xxx_if)");
            sib_it sib = it.begin();
            vertex vcond = eval_throws(sib, pe, vu);
            OC_ASSERT(is_boolean(vcond), "vertex should be a boolean.");
            ++sib;
            if (vcond == id::logical_true) {
                return eval_throws(sib, pe, vu);
            } else {
                ++sib;
                return eval_throws(sib, pe, vu);
            }
        }

        // mixed operators
        case id::greater_than_zero : {
            OC_ASSERT(it.has_one_child(),
                      "combo_tree node should have exactly three children"
                      " (id::greater_than_zero).");
            sib_it sib = it.begin();
            vertex x = eval_throws(sib, pe, vu);
            OC_ASSERT(is_contin(x),
                      "vertex should be a contin.");
            return bool_to_vertex(0 < get_contin(x));
        }
        case id::impulse : {
            vertex i;
            OC_ASSERT(it.has_one_child(),
                      "combo_tree node should have exactly one child"
                      " (id::impulse).");
            i = eval_throws(it.begin(), pe, vu);
            OC_ASSERT(is_boolean(i),
                      "vetex should be a boolean).");
            return (i == id::logical_true ? 1.0 : 0.0);
        }
        // continuous operator
        case id::plus : {
            contin_t res = 0;
            // assumption : plus can have 1 or more arguments
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                vertex vres = eval_throws(sib, pe, vu);
                OC_ASSERT(is_contin(vres), "vertex should be a contin.");
                res += get_contin(vres);
            }
            return res;
        }
        case id::rand :
            return randGen().randfloat();
        case id::times : {
            contin_t res = 1;
            // assumption : times can have 1 or more arguments
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                vertex vres = eval_throws(sib, pe, vu);
                OC_ASSERT(is_contin(vres), "vertex should be a contin");
                res *= get_contin(vres);
            }
            return res;
        }
        case id::div : {
            contin_t x, y;
            OC_ASSERT(it.number_of_children() == 2,
                      "combo_tree node should have exactly two children"
                      " (id::div).");
            sib_it sib = it.begin();
            vertex vx = eval_throws(sib, pe, vu);
            OC_ASSERT(is_contin(vx),
                      "vertex should be a contin.");
            x = get_contin(vx);
            ++sib;
            vertex vy = eval_throws(sib, pe, vu);
            OC_ASSERT(is_contin(vy),
                      "vertex should be a contin.");
            y = get_contin(vy);
            contin_t res = x / y;
            if (isnan(res) || isinf(res))
                throw EvalException(vertex(res));
            return res;
        }
        case id::log : {
            OC_ASSERT(it.has_one_child(),
                      "combo_tree node should have exactly one child"
                      " (id::log).");
            vertex vx = eval_throws(it.begin(), pe, vu);
            OC_ASSERT(is_contin(vx),
                      "vertex should be a contin");
#ifdef ABS_LOG
            contin_t res = log(std::abs(get_contin(vx)));
#else
            contin_t res = log(get_contin(vx));
#endif
            if (isnan(res) || isinf(res))
                throw EvalException(vertex(res));
            return res;
        }
        case id::exp : {
            OC_ASSERT(it.has_one_child(),
                      "combo_tree node should have exactly one child"
                      " (id::exp)");
            vertex vx = eval_throws(it.begin(), pe, vu);
            OC_ASSERT(is_contin(vx),
                      "vertex should be an contin");
            contin_t res = exp(get_contin(vx));
            // this may happen in case the argument is too high, then exp will be infty
            if (isinf(res)) throw EvalException(vertex(res));
            return res;
        }
        case id::sin : {
            OC_ASSERT(it.has_one_child(),
                      "combo_tree node should have exactly one child"
                      " (id::sin)");
            vertex vx = eval_throws(it.begin(), pe, vu);
            OC_ASSERT(is_contin(vx),
                      "vertex should be a contin.");
            return sin(get_contin(vx));
        }
        default :
            OC_ASSERT(false,
                      "That case is not handled");
            return v;
        }
    }
    // action
    else if (is_action(*it) && pe) {
        OC_ASSERT(pe, "Non null Evaluator must be provided");
        return pe->eval_action(it, vu);
    }
    // perception
    else if (is_perception(*it) && pe) {
        OC_ASSERT(pe, "Non null Evaluator must be provided");
        return pe->eval_percept(it, vu);
    }
    // procedure
    else if (is_procedure_call(*it) && pe) {
        OC_ASSERT(pe, "Non null Evaluator must be provided");
        return pe->eval_procedure(it, vu);
    }
    // indefinite objects are evaluated by the pe
    else if (const indefinite_object* io = boost::get<indefinite_object>(&v)) {
        OC_ASSERT(pe, "Non null Evaluator must be provided");
        return pe->eval_indefinite_object(*io, vu);
    }
    // definite objects evaluate to themselves
    else if (is_definite_object(*it)) {
        OC_ASSERT(it.is_childless(),
                  "combo_tree node should be childless (definite_object '%s').",
                  get_definite_object(*it).c_str());
        return v;
    }
    // contin constant
    else if (const contin_t* c = boost::get<contin_t>(&v)) {
      if (isnan(*c) || isinf(*c))
          throw EvalException(vertex(*c));
      return v;
    }
    // action symbol
    else if (is_action_symbol(v)) {
        return v;
    } else {
        // @todo: strangely this cannot compile
        // std::cerr << "unrecognized expression " << combo_tree(it) << std::endl;
        throw EvalException(*it);
        return v;
    }
}

template<typename It>
vertex eval(It it)
     throw(ComboException,
           AssertionException, std::bad_exception)
{
    try {
        return eval_throws(it);
    } catch (EvalException e) {
        return e.get_vertex();
    }
}

template<typename T>
vertex eval(const tree<T>& tr)
     throw(StandardException, std::bad_exception)
{
    return eval(tr.begin());
}

template<typename T>
vertex eval_throws(const tree<T>& tr)
     throw(EvalException,
           ComboException,
           AssertionException,
           std::bad_exception)
{
    return eval_throws(tr.begin());
}
#endif

/// Like above but ignore the variable_unifier and uses
/// binding_map. It also removes any type checking as it is the job of
/// static type checker. The Evaluator is ignored till vu is
/// completely removed
// vertex eval_throws_binding(binding_map& bmap,
//                            combo_tree::iterator it, Evaluator* pe = NULL)
vertex eval_throws_binding(const vertex_seq& bmap,
                           combo_tree::iterator it, Evaluator* pe = NULL)
    throw(EvalException, ComboException,
          AssertionException, std::bad_exception);

vertex eval_binding(const vertex_seq& bmap, combo_tree::iterator it)
     throw(ComboException, AssertionException, std::bad_exception);

vertex eval_binding(const vertex_seq& bmap, const combo_tree& tr)
    throw(StandardException, std::bad_exception);

vertex eval_throws_binding(const vertex_seq& bmap, const combo_tree& tr)
     throw(EvalException, ComboException, AssertionException,
           std::bad_exception);

// return the arity of a tree
template<typename T>
arity_t arity(const tree<T>& tr)
{
    arity_t a = 0;
    for (typename tree<T>::iterator it = tr.begin();
         it != tr.end(); ++it)
        if (is_argument(*it))
            a = std::max(a, (arity_t)std::abs(get_argument(*it).idx));
    return a;
}

}} // ~namespaces combo opencog

#endif
