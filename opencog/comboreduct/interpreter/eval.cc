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

#if ALMOST_DEAD_EVAL_CODE
// @todo all users of the code below should switch to using
// eval_throws_binding() instead.
//
// Right now, as far as I can tell, only embodiment code uses this.
//
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
    for (const binding_map::value_type& vt : bmap) {
        std::cout << vt.first << ":";
        if (const vertex* v = boost::get<const vertex>(&vt.second))
            std::cout << *v;
        else
            std::cout << boost::get<combo_tree::iterator>(&vt.second);
        std::cout << ",";
    }
    std::cout << "}" << std::endl;
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
        // return idx > 0? v : negate_vertex(v);
        return idx > 0? bmap[idx - 1] : negate_vertex(bmap[-idx - 1]);
    }
    // builtin
    else if (const builtin* b = boost::get<builtin>(&v)) {
        switch (*b) {

        // Boolean operators
        case id::logical_false :
        case id::logical_true :
            return v;

        case id::logical_and : {
            if (it.begin() == it.end()) // For correct foldr behaviour
                return id::logical_and;
            for (sib_it sib = it.begin(); sib != it.end(); ++sib)
                if (eval_throws_binding(bmap, sib, pe) == id::logical_false)
                    return id::logical_false;
            return id::logical_true;
        }

        case id::logical_or : {
            if (it.begin() == it.end())  // For correct foldr behaviour
                return id::logical_or;
            for (sib_it sib = it.begin(); sib != it.end(); ++sib)
                if (eval_throws_binding(bmap, sib, pe) == id::logical_true)
                    return id::logical_true;
            return id::logical_false;
        }
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
            // if plus does not have any argument, return plus operator
            if (it.begin() == it.end()) // For correct foldr behaviour
                return v;

            contin_t res = 0;
            //assumption : plus can have 1 or more arguments
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                vertex vres = eval_throws_binding(bmap, sib, pe);
                res += get_contin(vres);
            }
            return res;
        }

        case id::times : {
            //if times does not have any argument, return times operator
            if (it.begin() == it.end())  // For correct foldr behaviour
                return v;

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

        // Almost all list ops are not supported by this function...
        case id::list :
        case id::cdr :
        case id::cons :
        case id::foldr :
        case id::foldl :
            throw ComboException(TRACE_INFO,
                "Cannot handle lists; use eval_throws_tree() instead.");

        // car returns the first elt of a list. The list better not
        // be empty, and it's first elt better not be a list...
        case id::car : {
            sib_it lp = it.begin();

            combo_tree evo;
            if (*lp != id::list) {
                evo = eval_throws_tree(bmap, lp, pe);
                lp = evo.begin();
            }
            if (*lp != id::list)
                throw ComboException(TRACE_INFO, "not a list!");

            // If the list is empty, throw; user should have called
            // eval_throws_tree(), and not eval_throws_binding().
            if (lp.begin() == lp.end())
                throw ComboException(TRACE_INFO,
                   "Must not pass empty list to eval_throws_binding().");
            return eval_throws_binding(bmap, lp.begin(), pe);
        }

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

vertex eval_throws_binding(const vertex_seq& bmap, const combo_tree& tr)
    throw (EvalException, ComboException, AssertionException, std::bad_exception)
{
    return eval_throws_binding(bmap, tr.begin());
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

combo_tree eval_throws_tree(const vertex_seq& bmap,
                           combo_tree::iterator it, Evaluator* pe)
    throw(EvalException, ComboException,
          AssertionException, std::bad_exception)
{
    typedef combo_tree::sibling_iterator sib_it;
    typedef combo_tree::iterator pre_it;
    const vertex& v = *it;

    if (const builtin* b = boost::get<builtin>(&v)) {
        switch (*b) {

        // list constructor
        case id::list : {

            combo_tree tr(id::list);
            pre_it loc = tr.begin();

            for (sib_it sib = it.begin(); sib != it.end(); sib++) {
                // tr.append_child(loc, eval_throws_tree(bmap, sib, pe).begin());
                combo_tree rr = eval_throws_tree(bmap, sib, pe);
                tr.append_child(loc, rr.begin());
             }

            return tr;
        }

        // car takes a list and returns head of the list
        case id::car : {
            sib_it lp = it.begin();

            combo_tree evo;
            if (*lp != id::list) {
                evo = eval_throws_tree(bmap, lp, pe);
                lp = evo.begin();
            }
            if (*lp != id::list)
                throw ComboException(TRACE_INFO, "not a list!");

            // If the list is empty; then return empty list!
            // That is, use an empty list to represent nil.
            if (lp.begin() == lp.end())
                return combo_tree(id::list);
            return eval_throws_tree(bmap, lp.begin(), pe);
        }

        // cdr takes a list and returns the tail of the list
        case id::cdr : {
            sib_it top = it.begin();

            combo_tree evo;
            if (*top != id::list) {
                evo = eval_throws_tree(bmap, top, pe);
                top = evo.begin();
            }
            if (*top != id::list)
                throw ComboException(TRACE_INFO, "not a list!");

            sib_it sib = top.begin();

            // Skip over the first elt
            sib ++;

            combo_tree tr(id::list);
            pre_it loc = tr.begin();
            for (; sib != top.end(); sib++) {
                combo_tree rest = eval_throws_tree(bmap, sib, pe);
                tr.append_child(loc, rest.begin());
            }

            return tr;
        }

        // cons takes an element and a list,
        // and adds the element to the head of the list
        case id::cons : {
            combo_tree tr(id::list);
            pre_it loc = tr.begin();
            if(it.begin()==it.end()){
              vertex vx(id::cons);
              return combo_tree(vx);
            }
            sib_it head = it.begin();
            combo_tree ht = eval_throws_tree(bmap, head, pe);
            tr.append_child(loc, ht.begin());

            head++;
            combo_tree rest = eval_throws_tree(bmap, head, pe);

            sib_it lst = rest.begin();
            for (sib_it sib = lst.begin(); sib != lst.end(); sib++)
                // tr.append_child(loc, eval_throws_tree(bmap, sib, pe).begin());
                tr.append_child(loc, (pre_it) sib);

            return tr;
        }

        case id::foldr : {
            combo_tree tr(it);
 
            // base case: foldr(f v list) = v
            // i.e. list is empty list.
            sib_it itend = tr.begin().end();
            itend--;
            if (itend.begin() == itend.end()) {
                itend--;
                return eval_throws_tree(bmap, itend, pe);
            }

            // new tree: f(car foldr(f v cdr))

            sib_it f = it.begin();
            combo_tree cb_tr(f);
            sib_it loc = cb_tr.begin();
            combo_tree car_lst(id::car); 
            sib_it car_lst_it = car_lst.begin();
            car_lst.append_child(car_lst_it, itend);
            car_lst = eval_throws_tree(bmap,car_lst_it,pe);
            car_lst_it = car_lst.begin();
            cb_tr.append_child(loc, car_lst_it);

            combo_tree cdr_lst(id::cdr);
            sib_it cdr_lst_it = cdr_lst.begin();
            cdr_lst.append_child(cdr_lst_it, itend);
            tr.erase(itend);
            sib_it tr_it = tr.begin();
            cdr_lst = eval_throws_tree(bmap, cdr_lst_it,pe);
            cdr_lst_it = cdr_lst.begin();
            tr.append_child(tr_it, cdr_lst_it);
            tr = eval_throws_tree(bmap, tr_it, pe);
            tr_it = tr.begin();
            cb_tr.append_child(loc, tr_it);

            return eval_throws_tree(bmap, loc, pe); 
        }

        case id::foldl : {
            combo_tree tr(it);

            // base case: foldl(f v list) = v
            // i.e. list is empty list.
            sib_it itend = tr.begin().end();
            itend--;
            if (itend.begin() == itend.end()) {
                itend--;
                return eval_throws_tree(bmap, itend, pe);
            }

            // new tree: foldl(f f(v car) cdr)
            sib_it f = it.begin();
            combo_tree lst = eval_throws_tree(bmap, itend, pe);
            sib_it lst_it = lst.begin();
            sib_it tr_it = tr.begin();
            tr.erase(itend);
            itend--;
            tr.erase(itend);
	    
            combo_tree rec(f);
            sib_it rec_it = rec.begin();
            sib_it v = ++f;
            rec.append_child(rec_it, v);
            combo_tree car_lst(id::car);
            sib_it car_lst_it = car_lst.begin();
            car_lst.append_child(car_lst_it, lst_it);
            car_lst = eval_throws_tree(bmap, car_lst_it, pe);
            car_lst_it = car_lst.begin();
            rec.append_child(rec_it, car_lst_it);  //f(v car)
            tr.append_child(tr_it, rec_it);

            combo_tree cdr_lst(id::cdr);
            sib_it cdr_lst_it = cdr_lst.begin();
            cdr_lst.append_child(cdr_lst_it, lst_it);
            cdr_lst = eval_throws_tree(bmap, cdr_lst_it, pe);
            cdr_lst_it = cdr_lst.begin();
            tr.append_child(tr_it, cdr_lst_it);

            return eval_throws_tree(bmap, tr_it , pe); 
        }

        // lambda constructor
        case id::lambda : {
            combo_tree tr(it);
            return tr;
        }

        case id::apply : {
            combo_tree tr(it);
            sib_it tr_it = tr.begin().begin();
            sib_it lambda_it = tr_it.end();
            lambda_it --;
            combo_tree exp_tr(lambda_it);

            vector<vertex> al; // list of arguments
            tr_it++;
            for(; tr_it!=tr.begin().end(); tr_it++){
                al.push_back(*tr_it);
            }
            set_bindings(exp_tr, exp_tr.begin(), al, explicit_arity(exp_tr));
            return eval_throws_tree(bmap, exp_tr);
        }

        default:
            break;
        }
    }

    // If we got the here, its not a list operator, so just return
    // a tree with a lone, simple type in it.
    return combo_tree(eval_throws_binding(bmap, it, pe));
}

combo_tree eval_throws_tree(const vertex_seq& bmap, const combo_tree& tr)
    throw (EvalException, ComboException, AssertionException, std::bad_exception)
{
    return eval_throws_tree(bmap, tr.begin());
}


}} // ~namespaces combo opencog
