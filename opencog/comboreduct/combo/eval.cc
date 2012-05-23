/*
 * opencog/comboreduct/combo/eval.cc
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
#include "eval.h"
#include <iostream>

namespace opencog { namespace combo {

using namespace std;

void set_bindings(combo_tree& tr, combo_tree::iterator it,
                  const std::vector<vertex>& args, arity_t explicit_arity)
{
    combo_tree::iterator end = it;
    end.skip_children();
    ++end;
    arity_t implicit_idx = explicit_arity;
    arity_t ap_args = args.size();
    for (combo_tree::iterator at = it;at != end;++at) {
        if (at.is_childless()) {

            //if argument #idx then substitute it by
            //args[idx-1]
            if (is_argument(*at)) {
                argument& arg = get_argument(*at);
                if (arg.is_negated()) {
                    tr.append_child(at, *at);
                    *at = id::logical_not;
                    at = at.begin();
                    arg.negate();
                }
                *at = args[arg.idx-1];
            } else {
                arity_t a = get_arity(*at);
                if (a != 0) {
                    arity_t ama = abs_min_arity(a);
                    arity_t rest_ap_arg = ap_args - implicit_idx;
                    if (ama <= rest_ap_arg) {
                        arity_t idx_bound;
                        if (a > 0) //that is the arity is fixed
                            idx_bound = implicit_idx + ama;
                        else //that is at uses arg_list
                            idx_bound = ap_args;
                        for (; implicit_idx < idx_bound; ++implicit_idx)
                            tr.append_child(at, args[implicit_idx]);
                    } else { //raise an assert
                        std::stringstream ss;
                        ss << *at;
                        OC_ASSERT(false,
                                  "There is not enough arguments given"
                                  " in input, %s needs at least %d"
                                  " arguments and only %d are provided",
                                  ss.str().c_str(),
                                  static_cast<int>(ama),
                                  static_cast<int>(rest_ap_arg));
                    }
                }
            }
        }
    }
}

void set_bindings(combo_tree& tr, combo_tree::iterator it,
                  combo_tree::iterator arg_parent, arity_t explicit_arity)
{
    std::vector<combo_tree::iterator>
    args(boost::make_counting_iterator(arg_parent.begin()),
         boost::make_counting_iterator(arg_parent.end()));
    combo_tree::iterator end = it;
    end.skip_children();
    ++end;
    arity_t implicit_idx = explicit_arity;
    arity_t ap_args = args.size();
    for (combo_tree::iterator at = it;at != end;++at) {
        if (at.is_childless()) {
            //if argument #idx then substitute it by
            //args[idx-1]
            if (is_argument(*at)) {
                argument& arg = get_argument(*at);
                if (arg.is_negated()) {
                    tr.append_child(at, *at);
                    *at = id::logical_not;
                    at = at.begin();
                    arg.negate();
                }
                combo_tree tmp(args[arg.idx-1]);
                at = tr.move_ontop(at, tmp.begin());
                at.skip_children();
            } else {
                arity_t a = get_arity(*at);
                if (a != 0) {
                    arity_t ama = abs_min_arity(a);
                    arity_t rest_ap_arg = ap_args - implicit_idx;
                    if (ama <= rest_ap_arg) {
                        arity_t idx_bound;
                        if (a > 0) //that is the arity is fixed
                            idx_bound = implicit_idx + ama;
                        else //that is at uses arg_list
                            idx_bound = ap_args;
                        for (; implicit_idx < idx_bound; ++implicit_idx) {
                            combo_tree tmp(args[implicit_idx]);
                            tr.move_ontop(tr.append_child(at), tmp.begin());
                        }
                        at.skip_children();
                    } else { //raise an assert
                        std::stringstream ss;
                        ss << *at;
                        OC_ASSERT(false,
                                  "There is not enough arguments given"
                                  " in input, %s needs at least %d"
                                  " arguments and only %d are provided",
                                  ss.str().c_str(),
                                  static_cast<int>(ama),
                                  static_cast<int>(rest_ap_arg));
                    }
                }
            }
        }
    }
}

void set_bindings(combo_tree& tr, const std::vector<vertex>& args)
{
    if (!tr.empty())
        set_bindings(tr, tr.begin(), args, explicit_arity(tr));
}

void set_bindings(combo_tree& tr, combo_tree::iterator arg_parent)
{
    if (!tr.empty())
        set_bindings(tr, tr.begin(), arg_parent, explicit_arity(tr));
}

// debug printing
void print_binding_map(const binding_map& bmap) {
    std::cout << "bmap = {";
    foreach(const binding_map::value_type& vt, bmap) {
        std::cout << vt.first << ":";
        if (const vertex* v = boost::get<const vertex>(&vt.second))
            std::cout << *v;
        else
            std::cout << boost::get<combo_tree::iterator>(&vt.second);
        std::cout << ",";
    }
    std::cout << "}" << std::endl;
}

/// Like above but ignore the variable_unifier and uses vertex_map
/// (faster and more direct than binding_map as it is an information
/// already computed in the ITable or CTable). It also removes any
/// type checking as it is the job of static type checker. The
/// Evaluator is ignored till vu is completely removed
vertex eval_throws_binding(const vertex_seq& bmap,
                           combo_tree::iterator it, Evaluator* pe)
    throw(EvalException, ComboException,
          AssertionException, std::bad_exception)
{
    // {
    //     stringstream ss;
    //     ss << "bmap = ";
    //     ostreamContainer(ss, bmap);
    //     logger().fine(ss.str());
    // }
    // {
    //     stringstream ss;
    //     ss << "eval_throws_binding: tr = " << combo_tree(it);
    //     logger().fine(ss.str()); 
    // }

    typedef combo_tree::sibling_iterator sib_it;
    const vertex& v = *it;

    if (const argument* a = boost::get<argument>(&v)) {
        arity_t idx = a->idx;
        // Assumption : when idx is negative, the argument is
        // necessarily boolean, and must be negated.
        return idx > 0? bmap[idx - 1] : negate_vertex(bmap[-idx - 1]);
    }
    // builtin
    else if (const builtin* b = boost::get<builtin>(&v)) {
        switch (*b) {

        // Boolean operators
        case id::logical_false :
        case id::logical_true :
            return v;

        case id::logical_and :
            for (sib_it sib = it.begin(); sib != it.end(); ++sib)
                if (eval_throws_binding(bmap, sib, pe) == id::logical_false)
                    return id::logical_false;
            return id::logical_true;

        case id::logical_or :
            for (sib_it sib = it.begin(); sib != it.end(); ++sib)
                if (eval_throws_binding(bmap, sib, pe) == id::logical_true)
                    return id::logical_true;
            return id::logical_false;                

        case id::logical_not :
            return negate_vertex(eval_throws_binding(bmap, it.begin(), pe));

        // Mixed operators

        case id::greater_than_zero : {
            sib_it sib = it.begin();
            vertex x;
            try {
                // A divide-by-zero in the contin could throw. We want
                // to return a boolean in this case, anyway.  Values
                // might be +inf -inf or nan and we can still get a
                // sign bit off two of these cases...
                x = eval_throws_binding(bmap, sib, pe);
            } catch (EvalException e) {
                x = e.get_vertex();
            }
            return bool_to_vertex(0 < get_contin(x));
        }

        case id::impulse : {
            vertex i;
            i = eval_throws_binding(bmap, it.begin(), pe);
            return (i == id::logical_true ? 1.0 : 0.0);
        }

        // Continuous operators
        case id::plus : {
            contin_t res = 0;
            //assumption : plus can have 1 or more arguments
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                vertex vres = eval_throws_binding(bmap, sib, pe);
                res += get_contin(vres);
            }
            return res;
        }

        case id::times : {
            contin_t res = 1;
            //assumption : times can have 1 or more arguments
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                vertex vres = eval_throws_binding(bmap, sib, pe);
                res *= get_contin(vres);
                if (0.0 == res) return res;  // avoid pointless evals
            }
            return res;
        }

        case id::div : {
            contin_t x, y;
            sib_it sib = it.begin();
            vertex vx = eval_throws_binding(bmap, sib, pe);
            x = get_contin(vx);
            if (0.0 == x) return x;  // avoid pointless evals
            ++sib;
            vertex vy = eval_throws_binding(bmap, sib, pe);
            y = get_contin(vy);
            contin_t res = x / y;
            if (isnan(res) || isinf(res))
                throw EvalException(vertex(res));
            return res;
        }

        case id::log : {
            vertex vx = eval_throws_binding(bmap, it.begin(), pe);
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
            vertex vx = eval_throws_binding(bmap, it.begin(), pe);
            contin_t res = exp(get_contin(vx));
            // this may happen in case the argument is too high, then
            // exp will be infty
            if (isinf(res)) throw EvalException(vertex(res));
            return res;
        }

        case id::sin : {
            vertex vx = eval_throws_binding(bmap, it.begin(), pe);
            return sin(get_contin(vx));
        }

        case id::rand :
            return randGen().randfloat();


        // Control operators

        // XXX TODO: contin_if should go away.
        case id::contin_if :
        case id::cond : {
            sib_it sib = it.begin();
            while (1) {
                OC_ASSERT (sib != it.end(), "Error: mal-formed cond statement");

                vertex vcond = eval_throws_binding(bmap, sib, pe);
                ++sib;  // move past the condition

                // The very last value is the universal "else" clause,
                // taken when none of the previous predicates were true.
                if (sib == it.end())
                    return vcond;

                // If condition is true, then return the consequent
                // (i.e. the value immediately following.) Else, skip
                // the consequent, and loop around again.
                if (vcond == id::logical_true)
                    return eval_throws_binding(bmap, sib, pe);

                ++sib;  // move past the consequent
            }
        }
        case id::equ : {
            OC_ASSERT(false, "The equ operator is not handled yet...");
            return v;
        }
        default :
            OC_ASSERT(false, "That case is not handled");
            return v;
        }
    }

    // contin constant
    else if (const contin_t* c = boost::get<contin_t>(&v)) {
        if (isnan(*c) || isinf(*c))
            throw EvalException(vertex(*c));
        return v;
    }

    // enums are constants too
    else if (is_enum_type(v)) {
        return v;
    }

    // PetBrain stuff
    // action
    else if (is_action(v) && pe) {
        OC_ASSERT(pe, "Non null Evaluator must be provided");
        return pe->eval_action(it, combo::variable_unifier::DEFAULT_VU());
    }
    // perception
    else if (is_perception(v) && pe) {
        OC_ASSERT(pe, "Non null Evaluator must be provided");
        return pe->eval_percept(it, combo::variable_unifier::DEFAULT_VU());
    }
    // procedure
    else if (is_procedure_call(v) && pe) {
        OC_ASSERT(pe, "Non null Evaluator must be provided");
        return pe->eval_procedure(it, combo::variable_unifier::DEFAULT_VU());
    }
    // indefinite objects are evaluated by the pe
    else if (const indefinite_object* io = boost::get<indefinite_object>(&v)) {
        OC_ASSERT(pe, "Non null Evaluator must be provided");
        return pe->eval_indefinite_object(*io, combo::variable_unifier::DEFAULT_VU());
    }
    // definite objects evaluate to themselves
    else if (is_definite_object(v)) {
        return v;
    }
    // action symbol
    else if (is_action_symbol(v)) {
        return v;
    } else {
        // std::cerr << "unrecognized expression " << v << std::endl;
        throw EvalException(v);
        return v;
    }
}

vertex eval_binding(const vertex_seq& bmap, combo_tree::iterator it)
    throw (ComboException, AssertionException, std::bad_exception)
{
    try {
        return eval_throws_binding(bmap, it);
    } catch (EvalException e) {
        return e.get_vertex();
    }
}

vertex eval_binding(const vertex_seq& bmap, const combo_tree& tr)
    throw (StandardException, std::bad_exception)
{
    return eval_binding(bmap, tr.begin());
}

vertex eval_throws_binding(const vertex_seq& bmap, const combo_tree& tr)
    throw (EvalException, ComboException, AssertionException, std::bad_exception)
{
    return eval_throws_binding(bmap, tr.begin());
}


}} // ~namespaces combo opencog
