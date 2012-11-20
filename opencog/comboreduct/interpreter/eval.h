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

#include <exception>
#include <boost/unordered_map.hpp>

#include <opencog/util/tree.h>
#include <opencog/util/numeric.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/mt19937ar.h>

#include "../combo/vertex.h"
#include "../combo/using.h"
#include "../combo/variable_unifier.h"
#include "../crutil/exception.h"
#include "../type_checker/type_tree.h"

namespace opencog { namespace combo {

struct Evaluator {
    virtual ~Evaluator() { }
    virtual vertex eval_action(combo_tree::iterator, variable_unifier&) = 0;
    virtual vertex eval_percept(combo_tree::iterator, variable_unifier&) = 0;
    // @todo : this could be generic and not in the Evaluator
    // there would be a way to specify with the procedure is lazy or note
    // in order to be fully compatible with the way it is already used
    virtual vertex eval_procedure(combo_tree::iterator, variable_unifier&) = 0;
    virtual combo_tree eval_procedure_tree(const vertex_seq& bmap, combo_tree::iterator it)
    {
        OC_ASSERT(false, "eval_procedure_tree not supported");
    }
    // eval_indefinite_object takes no arguments because it is assumed
    // that it has no child, this assumption may change over time
    virtual vertex eval_indefinite_object(indefinite_object,
                                          variable_unifier&) = 0;
};

/// A subclass of Evaluator which only handles procedure calls. The other things
/// are only for Embodiment. (This class is based on the RunningComboProcedure class in
/// the Embodiment codebase.) Its functionality should probably be moved inside the eval* functions
/// if possible.
struct ProcedureEvaluator : public Evaluator {
    ~ProcedureEvaluator() { }
    ProcedureEvaluator(const combo::combo_tree& tr) : _tr(tr) { }

    vertex eval_action(combo_tree::iterator, variable_unifier&) { }
    vertex eval_percept(combo_tree::iterator, variable_unifier&) { }
    vertex eval_procedure(combo_tree::iterator, variable_unifier&);
    combo_tree eval_procedure_tree(const vertex_seq& bmap, combo_tree::iterator it);
    // eval_indefinite_object takes no arguments because it is assumed
    // that it has no child, this assumption may change over time
    vertex eval_indefinite_object(indefinite_object,
                                          variable_unifier&) { }

protected:
    /// for evaluating procedures inplace
    void expand_procedure_call(combo::combo_tree::iterator) throw
        (ComboException, AssertionException, std::bad_exception);

    combo::combo_tree _tr;
};

#define ALMOST_DEAD_EVAL_CODE 1
#if ALMOST_DEAD_EVAL_CODE
/// @todo all users of the code below should switch to using
/// eval_throws_binding() instead.
///
/// Right now, as far as I can tell, only embodiment code uses this.
/// I'm hoping that some embodiment re-write will make this go away.
/// Note, however, emobodiment seems to use the variable unifiers, so
/// I'm, not sure about that.
/// Anyway, the code below is no longer maintained, and is missing 
/// support for newer & better stuff.
//
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

// Used by Embodiment. Previously supported a tacky variable unification system, but now just calls the normal evaluator.
template<typename It>
vertex eval_throws(It it, Evaluator* pe = NULL,
                   combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU())
    throw(EvalException, ComboException,
          AssertionException, std::bad_exception)
{

    vertex_seq empty;
    return eval_throws_binding(empty, it, pe);
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
#endif /* ALMOST_DEAD_EVAL_CODE */

/// eval_throws_binding -- evaluate a combo tree, using the argument
/// values supplied in the vertex_seq list.
///
/// This proceedure does not do any type-checking; the static type-checker
/// should be used for this purpose.
/// The Evaluator is currently unused; we're waiting for variable unification
/// to be made obsolete (!?)
vertex eval_throws_binding(const vertex_seq& bmap,
                           combo_tree::iterator it, Evaluator* pe = NULL)
    throw(EvalException, ComboException,
          AssertionException, std::bad_exception);

vertex eval_throws_vertex(const vertex_seq& bmap,
                           combo_tree::iterator it, Evaluator* pe = NULL)
    throw(EvalException, ComboException,
          AssertionException, std::bad_exception);

vertex eval_throws_binding(const vertex_seq& bmap, const combo_tree& tr)
     throw(EvalException, ComboException, AssertionException,
           std::bad_exception);

// As above, but returns combo tree instead ov vertex.  The above,
// non-tree variants cannot be used when using lists, since the only
// way to represent a list is as a tree.
combo_tree eval_throws_tree(const vertex_seq& bmap, const combo_tree& tr)
     throw(EvalException, ComboException, AssertionException,
           std::bad_exception);

combo_tree eval_throws_tree(const vertex_seq& bmap,
                           combo_tree::iterator it, Evaluator* pe = NULL)
     throw(EvalException, ComboException, AssertionException,
           std::bad_exception);

// As above, but EvalException is never thrown.
vertex eval_binding(const vertex_seq& bmap, combo_tree::iterator it)
    throw(ComboException, AssertionException, std::bad_exception);

vertex eval_binding(const vertex_seq& bmap, const combo_tree& tr)
    throw(StandardException, std::bad_exception);

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
