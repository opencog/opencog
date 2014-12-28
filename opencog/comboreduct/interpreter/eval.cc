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
#include <iterator>

namespace opencog { namespace combo {

using namespace std;

combo_tree eval_procedure_tree(const vertex_seq& bmap,
                               combo::combo_tree::iterator it)
{
    // sanity checks
    if (!is_procedure_call(*it)) {
        throw ComboException(TRACE_INFO,
                                      "ProcedureEvaluator - combo_tree node does not represent a combo procedure call.");
    }
    procedure_call pc = get_procedure_call(*it);

    arity_t ar = pc->arity();
    bool fixed_arity = ar >= 0; //the procedure gets fix number of
    //input arguments
    combo::arity_t exp_arity = combo::abs_min_arity(ar);
    arity_t ap_args = it.number_of_children();
    //OC_ASSERT(ar>=0, "It is assumed that arity is 0 or above, if not in that case the procedure must contain operator with arbitrary arity like and_seq and no variable binding to it, it is probably an error but if you really want to deal with that then ask Nil to add the support of it");
    if (fixed_arity) {
        if (ap_args != ar) {
            throw ComboException(TRACE_INFO,
                                          "ProcedureEvaluator - %s arity differs from no. node's children. Arity: %d, number_of_children: %d",
                                          get_procedure_call(*it)->get_name().c_str(), ar, ap_args);
        }
    } else {
        if (ap_args < exp_arity) {
            throw ComboException(TRACE_INFO,
                                          "ProcedureEvaluator - %s minimum arity is greater than no. node's children. Minimum arity: %d, number_of_children: %d",
                                          get_procedure_call(*it)->get_name().c_str(), exp_arity, ap_args);
        }
    }

    // evaluate {the function body} with {the arguments to the function}
    combo_tree body(pc->get_body());
    cout << body << endl;

    vertex_seq args;
    // copy(body.begin(), body.end(), back_inserter(args));

    // evaluate the arguments to the function (their variables are in
    // the current scope, i.e. bmap)
    for (combo_tree::sibling_iterator arg_it = it.begin(); arg_it != it.end(); ++arg_it) {
        combo_tree arg_result(eval_throws_tree(bmap, arg_it));

        OC_ASSERT(arg_it.number_of_children() == 0, "functions cannot have list arguments");
        args.push_back(*arg_result.begin());
        cout << (*arg_result.begin()) << endl;
    }

    combo_tree ret(eval_throws_tree(args, body.begin()));
    return ret;
}

/// eval_throws_binding -- evaluate a combo tree, using the argument
/// values supplied in the vertex_seq list.
///
/// This proceedure does not do any type-checking; the static type-checker
/// should be used for this purpose.
vertex eval_throws_binding(const vertex_seq& bmap,
                           combo_tree::iterator it)
    throw(OverflowException, EvalException, ComboException,
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

    combo_tree ret(eval_throws_tree(bmap, it));
    // Make sure it has no children.
    OC_ASSERT(ret.number_of_children(ret.begin()) == 0,
              "Invalid use of eval_throws_binding:"
              "expression evaluates to a whole combo_tree, not just one vertex");

    return *ret.begin();
}

vertex eval_throws_vertex(const vertex_seq& bmap,
                           combo_tree::iterator it)
    throw(OverflowException, EvalException, ComboException,
          AssertionException, std::bad_exception)
{
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
                if (eval_throws_binding(bmap, sib) == id::logical_false)
                    return id::logical_false;
            return id::logical_true;
        }

        case id::logical_or : {
            if (it.begin() == it.end())  // For correct foldr behaviour
                return id::logical_or;
            for (sib_it sib = it.begin(); sib != it.end(); ++sib)
                if (eval_throws_binding(bmap, sib) == id::logical_true)
                    return id::logical_true;
            return id::logical_false;
        }
        case id::logical_not :
            return negate_vertex(eval_throws_binding(bmap, it.begin()));

        // Mixed operators

        case id::greater_than_zero : {
            sib_it sib = it.begin();
            vertex x;
            try {
                // A divide-by-zero in the contin could throw. We want
                // to return a boolean in this case, anyway.  Values
                // might be +inf -inf or nan and we can still get a
                // sign bit off two of these cases...
                x = eval_throws_binding(bmap, sib);
            } catch (OverflowException& e) {
                x = e.get_vertex();
            }
            return bool_to_vertex(0 < get_contin(x));
        }

        case id::impulse : {
            vertex i;
            i = eval_throws_binding(bmap, it.begin());
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
                vertex vres = eval_throws_binding(bmap, sib);
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
                vertex vres = eval_throws_binding(bmap, sib);
                res *= get_contin(vres);
                if (0.0 == res) return res;  // avoid pointless evals
            }
            return res;
        }

        case id::div : {
            contin_t x, y;
            sib_it sib = it.begin();
            vertex vx = eval_throws_binding(bmap, sib);
            x = get_contin(vx);
            if (0.0 == x) return x;  // avoid pointless evals
            ++sib;
            vertex vy = eval_throws_binding(bmap, sib);
            y = get_contin(vy);
            contin_t res = x / y;
            if (isnan(res) || isinf(res))
                throw OverflowException(vertex(res));
            return res;
        }

        case id::log : {
            vertex vx = eval_throws_binding(bmap, it.begin());
#ifdef ABS_LOG
            contin_t res = log(std::abs(get_contin(vx)));
#else
            contin_t res = log(get_contin(vx));
#endif
            if (isnan(res) || isinf(res))
                throw OverflowException(vertex(res));
            return res;
        }

        case id::exp : {
            vertex vx = eval_throws_binding(bmap, it.begin());
            contin_t res = exp(get_contin(vx));
            // this may happen in case the argument is too high, then
            // exp will be infty
            if (isinf(res)) throw OverflowException(vertex(res));
            return res;
        }

        case id::sin : {
            vertex vx = eval_throws_binding(bmap, it.begin());
            return sin(get_contin(vx));
        }

        case id::rand :
            return randGen().randfloat();

        // Control operators

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
            throw OverflowException(vertex(*c));
        return v;
    }

    // enums are constants too
    else if (is_enum_type(v)) {
        return v;
    }
    else {
        // Don't know what to do with this.  It might just be some
        // un-interpreted string, such as those used in listUTest.
        // If so, then the list functions should catch this exception.
        // Everyone else should be surprised by this.
        throw EvalException(v, "unrecognized expression");
        return v;
    }
}

vertex eval_throws_binding(const vertex_seq& bmap, const combo_tree& tr)
    throw (OverflowException, EvalException, ComboException, AssertionException, std::bad_exception)
{
    return eval_throws_binding(bmap, tr.begin());
}

vertex eval_binding(const vertex_seq& bmap, combo_tree::iterator it)
    throw (ComboException, AssertionException, std::bad_exception)
{
    try {
        return eval_throws_binding(bmap, it);
    } catch (OverflowException& e) {
        return e.get_vertex();
    } catch (EvalException& e) {
        return e.get_vertex();
    }
}

vertex eval_binding(const vertex_seq& bmap, const combo_tree& tr)
    throw (StandardException, std::bad_exception)
{
    return eval_binding(bmap, tr.begin());
}

combo_tree eval_throws_tree(const vertex_seq& bmap,
                           combo_tree::iterator it)
    throw(OverflowException, EvalException, ComboException,
          AssertionException, std::bad_exception)
{
    typedef combo_tree::sibling_iterator sib_it;
    typedef combo_tree::iterator pre_it;
    const vertex& v = *it;

    // @todo FIXME there should be a general way to distinguish between
    // "f" (the function f, being passed to fold) vs "f" (the function f
    // being called with no arguments). If you don't handle that you get
    // weird errors (because fold or other higher-order functions will
    // attempt to evaluate the argument too soon). (??? bugt fold shouldn't
    // do this.. ??? Can we have examples of this?)

    // First handle the operators that allow/require returning a combo_tree
    // (which can represent a combo list or combo lambda expression).
    // Then handle the operators that can only return a single combo vertex.
    if (const builtin* b = boost::get<builtin>(&v)) {
        switch (*b) {

        // list constructor
        case id::list : {

            combo_tree tr(id::list);
            pre_it loc = tr.begin();

            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                // tr.append_child(loc, eval_throws_tree(bmap, sib).begin());
                combo_tree rr = eval_throws_tree(bmap, sib);
                tr.append_child(loc, rr.begin());
             }

            return tr;
        }

        // car takes a list and returns head of the list
        case id::car : {
            sib_it lp = it.begin();

            combo_tree evo;
            if (*lp != id::list) {
                evo = eval_throws_tree(bmap, lp);
                lp = evo.begin();
            }
            if (*lp != id::list)
                throw ComboException(TRACE_INFO, "not a list!");

            // If the list is empty; then return empty list!
            // That is, use an empty list to represent nil.
            if (lp.begin() == lp.end())
                return combo_tree(id::list);
            return eval_throws_tree(bmap, lp.begin());
        }

        // cdr takes a list and returns the tail of the list
        case id::cdr : {
            sib_it top = it.begin();

            combo_tree evo;
            if (*top != id::list) {
                evo = eval_throws_tree(bmap, top);
                top = evo.begin();
            }
            if (*top != id::list)
                throw ComboException(TRACE_INFO, "not a list!");

            sib_it sib = top.begin();

            // Skip over the first elt
            sib ++;

            combo_tree tr(id::list);
            pre_it loc = tr.begin();
            for (; sib != top.end(); ++sib) {
                combo_tree rest = eval_throws_tree(bmap, sib);
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
            combo_tree ht = eval_throws_tree(bmap, head);
            tr.append_child(loc, ht.begin());

            ++head;
            combo_tree rest = eval_throws_tree(bmap, head);

            sib_it lst = rest.begin();
            for (sib_it sib = lst.begin(); sib != lst.end(); ++sib)
                // tr.append_child(loc, eval_throws_tree(bmap, sib).begin());
                tr.append_child(loc, (pre_it) sib);

            return tr;
        }

        case id::foldr : {
            combo_tree tr(it);
 
            // base case: foldr(f v list) = v
            // i.e. list is empty list.
            sib_it itend = tr.begin().end();
            --itend;
            if (itend.begin() == itend.end()) {
                --itend;
                return eval_throws_tree(bmap, itend);
            }

            // main case: foldr(f v l) = f(car foldr(f v cdr))
            // new tree: f(car foldr(f v cdr))

            // cb_tr will be the call to f.
            sib_it f = it.begin();
            combo_tree cb_tr(f);
            sib_it loc = cb_tr.begin();

            // eval: car(<the list>)
            combo_tree car_lst(id::car); 
            sib_it car_lst_it = car_lst.begin();
            car_lst.append_child(car_lst_it, itend);
            car_lst = eval_throws_tree(bmap,car_lst_it);
            car_lst_it = car_lst.begin();

            // cb_tr == f(<x>)
            cb_tr.append_child(loc, car_lst_it);

            // copy {the combo subtree for this use of foldr} and change it to include the cdr of L instead of L
            // let cdr_call = a new tree containing a call to cdr
            combo_tree cdr_call(id::cdr);
            sib_it cdr_call_it = cdr_call.begin();
            cdr_call.append_child(cdr_call_it, itend);
            tr.erase(itend);
            sib_it tr_it = tr.begin();

            // let cdr_result = the result of the call to cdr
            combo_tree cdr_result(eval_throws_tree(bmap, cdr_call_it));
            sib_it cdr_result_it = cdr_result.begin();
            tr.append_child(tr_it, cdr_result_it);

            tr = eval_throws_tree(bmap, tr_it);
            tr_it = tr.begin();

            cb_tr.append_child(loc, tr_it);

            return eval_throws_tree(bmap, loc);
        }

        case id::foldl : {
            combo_tree tr(it);

            // base case: foldl(f v list) = v
            // i.e. list is empty list.
            sib_it itend = tr.begin().end();
            --itend;
            if (itend.begin() == itend.end()) {
                --itend;
                return eval_throws_tree(bmap, itend);
            }

            // new tree: foldl(f f(v car) cdr)
            sib_it f = it.begin();
            combo_tree lst = eval_throws_tree(bmap, itend);
            sib_it lst_it = lst.begin();
            sib_it tr_it = tr.begin();
            tr.erase(itend);
            --itend;
            tr.erase(itend);
	    
            combo_tree rec(f);
            sib_it rec_it = rec.begin();
            sib_it v = ++f;
            rec.append_child(rec_it, v);
            combo_tree car_lst(id::car);
            sib_it car_lst_it = car_lst.begin();
            car_lst.append_child(car_lst_it, lst_it);
            car_lst = eval_throws_tree(bmap, car_lst_it);
            car_lst_it = car_lst.begin();
            rec.append_child(rec_it, car_lst_it);  //f(v car)
            tr.append_child(tr_it, rec_it);

            combo_tree cdr_lst(id::cdr);
            sib_it cdr_lst_it = cdr_lst.begin();
            cdr_lst.append_child(cdr_lst_it, lst_it);
            cdr_lst = eval_throws_tree(bmap, cdr_lst_it);
            cdr_lst_it = cdr_lst.begin();
            tr.append_child(tr_it, cdr_lst_it);

            return eval_throws_tree(bmap, tr_it ); 
        }

        // lambda constructor
        case id::lambda : {
            combo_tree tr(it);
            return tr;
        }

        // The apply() operator is a sensible thing to have, but this code is bad so I disabled it.
        // It shouldn't use set_bindings; if we want lambda functions then we should use scopes properly -- Jade
        case id::apply : {
            OC_ASSERT(false, "apply() is not implemented");
            combo_tree tr(it);
            sib_it tr_it = tr.begin().begin();
            sib_it lambda_it = tr_it.end();
            lambda_it --;
            combo_tree exp_tr(lambda_it);

            vector<vertex> al; // list of arguments
            ++tr_it;
            for(; tr_it!=tr.begin().end(); ++tr_it){
                al.push_back(*tr_it);
            }
            //set_bindings(exp_tr, exp_tr.begin(), al, explicit_arity(exp_tr));
            return eval_throws_tree(bmap, exp_tr);
        }

        // XXX TODO: contin_if should go away.
        case id::contin_if :
        case id::cond : {
            sib_it sib = it.begin();
            while (1) {
                OC_ASSERT (sib != it.end(), "Error: mal-formed cond statement");

                /// @todo tree copy
                combo_tree trcond(eval_throws_tree(bmap, sib));
                ++sib;  // move past the condition

                // The very last value is the universal "else" clause,
                // taken when none of the previous predicates were true.
                if (sib == it.end())
                    return trcond;

                // If condition is true, then return the consequent
                // (i.e. the value immediately following.) Else, skip
                // the consequent, and loop around again.
                vertex vcond = *trcond.begin();
                if (vcond == id::logical_true)
                    return eval_throws_tree(bmap, sib);

                ++sib;  // move past the consequent
            }
            break;
        }

        default:
            break;
        }
    }
    // procedure
    else if (is_procedure_call(v)) {
        if (it.begin() == it.end()) // For correct foldr behaviour
            return combo_tree(it);

        //return pe->eval_procedure(it, combo::variable_unifier::DEFAULT_VU());
        return eval_procedure_tree(bmap, it);
    }

    // Operators which only return a single vertex
    try {
        vertex retv(eval_throws_vertex(bmap, it));
        /// @todo FIXME avoid copying !?
        combo_tree ret(retv);
        return ret;
    }
    catch (EvalException& e) {
        vertex retv = e.get_vertex();
        /// @todo FIXME avoid copying !?
        combo_tree ret(retv);
        return ret;
    }

    return combo_tree();
}

combo_tree eval_throws_tree(const vertex_seq& bmap, const combo_tree& tr)
    throw (OverflowException, EvalException, ComboException, AssertionException, std::bad_exception)
{
    return eval_throws_tree(bmap, tr.begin());
}


}} // ~namespaces combo opencog
