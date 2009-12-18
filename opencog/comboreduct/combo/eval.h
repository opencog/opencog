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
#include <opencog/util/RandGen.h>

#include <opencog/comboreduct/crutil/exception.h>
#include "vertex.h"
#include "type_tree.h"
#include "using.h"
#include "variable_unifier.h"

#include <exception>
#include <boost/unordered_map.hpp>

#define COEF_SAMPLE_COUNT 20.0 //involved in the formula that counts
//the number of trials needed to check a formula

namespace combo
{

using opencog::isApproxEq;

struct Evaluator {
    virtual ~Evaluator() { }
    virtual vertex eval_action(combo_tree::iterator, combo::variable_unifier&) = 0;
    virtual vertex eval_percept(combo_tree::iterator, combo::variable_unifier&) = 0;
    //TODO : this could well be generic and not in the Evaluator
    virtual vertex eval_procedure(combo_tree::iterator, combo::variable_unifier&) = 0;
    //eval_indefinite_object takes no arguments because it is assumed
    //that it has no child, this assumption may change over time
    virtual vertex eval_indefinite_object(indefinite_object,
                                          combo::variable_unifier&) = 0;
};

//there are 2 ways of binding input arguments to a combo_tree
//
//1)associate the variable arguments #1, #2, etc with there values
//using binding, and then eval will use that mapping to evaluate
//the variable arguments on the fly
//
//or
//
//2)substituting directly the values in the combo_tree statically
//(be careful because it modifies the combo_tree)
//using set_bindings

inline boost::variant<vertex, combo_tree::iterator>& binding(int idx)
{
    static boost::unordered_map<int, boost::variant<vertex, combo_tree::iterator> > map; //to support lazy evaluation, can also bind to a subtree
    return map[idx];
}

//binding arguments to function calls
//That is it replaces all variable arguments (#1, #2, etc)
//by the provided arguments and append the implicit arguments at the
//childfree operators.
//explicit_arity corresponding to the highest argument idx, it is given
//because it is necessary to know it in order to append
//the implicit arguments to the free (without children) operators
void set_bindings(combo_tree& tr, combo_tree::iterator it,
                  const std::vector<vertex>&,
                  arity_t explicit_arity);
//like above but can bind arguments that are subtrees
//of arg_parent rather than vertex
void set_bindings(combo_tree& tr, combo_tree::iterator it,
                  combo_tree::iterator arg_parent,
                  arity_t explicit_arity);

//like above but it applies on the entire tree
//and explicit_arity is calculated automatically
void set_bindings(combo_tree& tr, const std::vector<vertex>&);
void set_bindings(combo_tree& tr, combo_tree::iterator arg_parent);

template<typename It>
vertex eval_throws(opencog::RandGen& rng,
                   It it, Evaluator* pe = NULL,
                   combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU())
    throw(EvalException, opencog::ComboException,
          opencog::AssertionException, std::bad_exception)
{

    //std::cout << "EVAL: " << combo_tree(it) << std::endl;

    //std::cout << "VU: " << vu.toString() << std::endl;

    typedef typename It::sibling_iterator sib_it;
    vertex& v = *it;

    if (const argument* a = boost::get<argument>(&v)) {
        int idx = a->idx;
        //assumption : when idx is negative the argument is necessary boolean
        if (idx > 0) {
            if (const vertex* v = boost::get<const vertex>(&binding(idx)))
                return * v;
            else
                return eval_throws(rng,
                                   boost::get<combo_tree::iterator>(binding(idx)),
                                   pe, vu);
        } else {
            if (const vertex* v = boost::get<const vertex>(&binding(-idx)))
                return negate_vertex(*v);
            else
                return negate_vertex(eval_throws(rng,
                                                 boost::get<combo_tree::iterator>(binding(-idx)),
                                                 pe, vu));
        }
    }
    //builtin
    else if (const builtin* b = boost::get<builtin>(&v)) {
        switch (*b) {
            //boolean operators
        case id::logical_true :
            return v;
        case id::logical_false :
            return v;
        case id::logical_and :
            for (sib_it sib = it.begin();sib != it.end();++sib) {

                if (vu.empty()) { //no wild_card case
                    if (eval_throws(rng, sib, pe, vu) == id::logical_false) {
                        return id::logical_false;
                    }
                } else { //wild_card case
                    combo::variable_unifier work_vu(vu);
                    work_vu.setUpdated(false);
                    bool is_false = (eval_throws(rng, sib, pe, work_vu)
                                     == id::logical_false); 
                    vu.unify(combo::UNIFY_AND, work_vu);
                    if(is_false) {
                        return id::logical_false;
                    }
                }
            }

            OC_ASSERT(vu.empty()
                              || vu.isOneVariableActiveTMP(),
                              "Since it returns logical_true from that point"
                              " there should be at least one active variable");

            return id::logical_true;
        case id::logical_or :
            // default case, no wild card
            if (vu.empty()) {
                for (sib_it sib = it.begin();sib != it.end();++sib) {
                    if (eval_throws(rng, sib, pe, vu) == id::logical_true) {
                        return id::logical_true;
                    }
                }
                return id::logical_false;

                // wild card case
            } else {
                
                OC_ASSERT(vu.isOneVariableActiveTMP(),
                                  "the OR wild_card case relies on the fact"
                                  " that at least one variable is active");

                combo::variable_unifier work_vu(vu);
                work_vu.setUpdated(false);
                for (sib_it sib = it.begin();sib != it.end();++sib) {

                    combo::variable_unifier return_vu(work_vu);
                    return_vu.setUpdated(false);
                    bool res = vertex_to_bool(eval_throws(rng, sib, pe,
                                                          return_vu));

                    if(!return_vu.isUpdated()) {
                        if(res) { //then all activated entities are
                                  //necessarily valid
                            return id::logical_true;
                        }
                        else { //then all activated entities remains
                               //to be checked to which can directly
                               //go back to the next loop
                            continue;
                        }
                    }

                    work_vu.unify(combo::UNIFY_OR, return_vu);


                    //if no variable remains to be checked
                    //after OR unification it means that
                    //the unification has succeeded
                    //ASSUMING THAT unify has done actually something
                    //which is only the case if updated is true
                    if(!work_vu.isOneVariableActiveTMP()) {
                        //OC_ASSERT(res,
                        //                "res should be true because work_vu"
                        //                " should start the iteration with"
                        //                " at least one active variable");
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
            OC_ASSERT(
                              it.has_one_child(),
                              "combo_tree node should have exactly one child"
                              " (id::logical_not)");

            if (vu.empty()) {
                return negate_vertex(eval_throws(rng, it.begin(), pe, vu));
            }

            // variable unifier not empty, need to unify

            // eval variable unifier
            combo::variable_unifier work_vu(vu);
            work_vu.setUpdated(false);

            vertex vx = eval_throws(rng, it.begin(), pe, work_vu);
            vu.unify(combo::UNIFY_NOT, work_vu);

            if (work_vu.isUpdated()) {
                return bool_to_vertex(vu.isOneVariableActive());
            }
            return negate_vertex(vx);
        }
        case id::boolean_if : {
            OC_ASSERT(it.number_of_children() == 3,
                              "combo_tree node should have exactly three children"
                              " (id::boolean_if)");
            sib_it sib = it.begin();
            vertex vcond = eval_throws(rng, sib, pe, vu);
            OC_ASSERT(is_boolean(vcond),
                              "vertex should be a booelan.");
            ++sib;
            if (vcond == id::logical_true) {
                return eval_throws(rng, sib, pe, vu);
            } else {
                ++sib;
                return eval_throws(rng, sib, pe, vu);
            }
        }
        //mixed operators
        case id::contin_if : {
            OC_ASSERT(it.number_of_children() == 3,
                              "combo_tree node should have exactly three children"
                              " (id::contin_if)");
            sib_it sib = it.begin();
            vertex vcond = eval_throws(rng, sib, pe, vu);
            OC_ASSERT(is_boolean(vcond),
                              "vertex should be a boolean.");
            ++sib;
            if (vcond == id::logical_true) {
                //TODO : check the type of the result
                return eval_throws(rng, sib, pe, vu);
            } else {
                ++sib;
                //TODO : check the type of the result
                return eval_throws(rng, sib, pe, vu);
            }
        }
        case id::greater_than_zero : {
            OC_ASSERT(it.has_one_child(),
                              "combo_tree node should have exactly three children"
                              " (id::greater_than_zero).");
            sib_it sib = it.begin();
            vertex x = eval_throws(rng, sib, pe, vu);
            OC_ASSERT(is_contin(x),
                              "vertex should be a contin.");
            return bool_to_vertex(0 < get_contin(x));
        }
        case id::impulse : {
            vertex i;
            OC_ASSERT(it.has_one_child(),
                              "combo_tree node should have exactly one child"
                              " (id::impulse).");
            i = eval_throws(rng, it.begin(), pe, vu);
            OC_ASSERT(is_boolean(i),
                              "vetex should be a boolean).");
            return (i == id::logical_true ? 1.0 : 0.0);
        }
        //continuous operator
        case id::plus : {
            contin_t res = 0;
            //assumption : plus can have 1 or more arguments
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                vertex vres = eval_throws(rng, sib, pe, vu);
                OC_ASSERT(is_contin(vres),
                                  "vertex should be a contin.");
                res += get_contin(vres);
            }
            return res;
        }
        case id::rand :
            return rng.randfloat();
        case id::times : {
            contin_t res = 1;
            //assumption : times can have 1 or more arguments
            for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
                vertex vres = eval_throws(rng, sib, pe, vu);
                OC_ASSERT(is_contin(vres),
                                  "vertex should be a contin");
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
            vertex vx = eval_throws(rng, sib, pe, vu);
            OC_ASSERT(is_contin(vx),
                              "vertex should be a contin.");
            x = get_contin(vx);
            ++sib;
            vertex vy = eval_throws(rng, sib, pe, vu);
            OC_ASSERT(is_contin(vy),
                              "vertex should be a contin.");
            y = get_contin(vy);
            contin_t res = x / y;
            if (isnan(res) || isinf(res)) throw EvalException(vertex(res));
            return res;
        }
        case id::log : {
            OC_ASSERT(it.has_one_child(),
                              "combo_tree node should have exactly one child"
                              " (id::log).");
            vertex vx = eval_throws(rng, it.begin(), pe, vu);
            OC_ASSERT(is_contin(vx),
                              "vertex should be a contin");
            contin_t res = log(get_contin(vx));
            if (isnan(res) || isinf(res)) throw EvalException(vertex(res));
            return res;
        }
        case id::exp : {
            OC_ASSERT(it.has_one_child(),
                              "combo_tree node should have exactly one child"
                              " (id::exp)");
            vertex vx = eval_throws(rng, it.begin(), pe, vu);
            OC_ASSERT(is_contin(vx),
                              "vertex should be an contin");
            contin_t res = exp(get_contin(vx));
            //this may happen in case the argument is too high, then exp will be infty
            if (isinf(res)) throw EvalException(vertex(res));
            return res;
        }
        case id::sin : {
            OC_ASSERT(it.has_one_child(),
                              "combo_tree node should have exactly one child"
                              " (id::sin)");
            vertex vx = eval_throws(rng, it.begin(), pe, vu);
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
    //action
    else if (is_action(*it) && pe) {
        OC_ASSERT(pe,
                          "Non null Evaluator must be provided");
        return pe->eval_action(it, vu);
    }
    //perception
    else if (is_perception(*it) && pe) {
        return pe->eval_percept(it, vu);
    }
    //procedure
    else if (is_procedure_call(*it) && pe) {
        return pe->eval_procedure(it, vu);
    }
    //indefinite objects are evaluated by the pe
    else if (const indefinite_object* io = boost::get<indefinite_object>(&v)) {
        OC_ASSERT(pe,
                          "Non null Evaluator must be provided");
        return pe->eval_indefinite_object(*io, vu);
    }
    //definite objects evaluate to themselves
    else if (is_definite_object(*it)) {
        OC_ASSERT(it.is_childless(),
                          "combo_tree node should be childless (definite_object '%s').",
                          get_definite_object(*it).c_str());
        return v;
    }
    //contin
    else if (const contin_t* c = boost::get<contin_t>(&v)) {
        if (isnan(*c) || isinf(*c)) throw EvalException(vertex(*c));
        return v;
    }
    //action
    else if (is_action_symbol(v)) {
        return v;
    } else {
        std::cerr << "unrecognized expression " << combo_tree(it) << std::endl;
        throw EvalException(*it);
        return v;
    }
}

template<typename It>
vertex eval(opencog::RandGen& rng, It it)
     throw(opencog::ComboException,
           opencog::AssertionException, std::bad_exception)
{
    try {
        return eval_throws(rng, it);
    } catch (EvalException e) {
        return e.get_vertex();
    }
}

template<typename T>
vertex eval(opencog::RandGen& rng, const opencog::tree<T>& tr)
     throw(opencog::StandardException, std::bad_exception)
{
    return eval(rng, tr.begin());
}

template<typename T>
vertex eval_throws(opencog::RandGen& rng, const opencog::tree<T>& tr)
     throw(EvalException,
           opencog::ComboException,
           opencog::AssertionException,
           std::bad_exception)
{
    return eval_throws(rng, tr.begin());
}

//return the arity of a tree
template<typename T>
int arity(const opencog::tree<T>& tr)
{
    int a = 0;
    for (typename opencog::tree<T>::iterator it = tr.begin();
         it != tr.end(); ++it)
        if (is_argument(*it))
            a = std::max(a, std::abs(get_argument(*it).idx));
    return a;
}

/*
  this function take an arity in input and returns in output the number
  of samples that would be appropriate to check the semantics of its associated
  tree. (Note : could take the two trees to checking and according to their arity
  structure, whatever, find an appropriate number.)
*/
inline int sample_count(int arity)
{
    if (arity == 0)
        return 1;
    else return (int)(COEF_SAMPLE_COUNT*log((float)arity + EXPONENTIAL));
}


//------------------------------------------------------------------------
// Truth table
//------------------------------------------------------------------------

class truth_table : public std::vector<bool>
{
public:
    typedef std::vector<bool> super;

    truth_table() { }
    template<typename It>
    truth_table(It from, It to) : super(from, to) { }
    template<typename T>
    truth_table(const opencog::tree<T>& t, int arity, opencog::RandGen& rng)
            : super(opencog::power(2, arity)) {
        populate(t, arity, rng);
    }
    template<typename T>
    truth_table(const opencog::tree<T>& t, opencog::RandGen& rng) {
        int a = arity(t);
        this->resize(opencog::power(2, a));
        populate(t, a, rng);
    }

    template<typename Func>
    truth_table(const Func& f, int arity, opencog::RandGen& rng)
        : super(opencog::power(2, arity)) {
        iterator it = begin();
        for (int i = 0;it != end();++i, ++it) {
            std::vector<bool> v(arity);
            for (int j = 0;j < arity;++j)
                v[j] = (i >> j) % 2;
            (*it) = f(v.begin(), v.end());
        }
    }

    /*
      this operator allows to access quickly to the results of a
      truth_table. [from, to) points toward a chain of boolean describing
      the inputs of the function coded into the truth_table and
      the operator returns the results.
    */
    template<typename It>
    bool operator()(It from,It to) {
        const_iterator it = begin();
        for (int i = 1;from != to;++from, i = i << 1)
            if (*from)
                it += i;
        return *it;
    }

    size_type hamming_distance(const truth_table& other) const;
protected:
    template<typename T>
    void populate(const opencog::tree<T>& tr,
                  int arity, opencog::RandGen& rng) {
        iterator it = begin();
        for (int i = 0;it != end();++i, ++it) {
            for (int j = 0;j < arity;++j)
                binding(j + 1) = bool_to_vertex((i >> j) % 2);
            (*it) = (eval(rng, tr) == id::logical_true);
        }
    }
};

//-------------------------------------------------------------------------
// contin table
//-------------------------------------------------------------------------

//shorthands used by class RndNumTable and contin_table
typedef std::vector<contin_t> contin_vector;
typedef contin_vector::iterator cv_it;
typedef contin_vector::const_iterator const_cv_it;
typedef std::vector<contin_vector> contin_matrix;
typedef contin_matrix::iterator cm_it;
typedef contin_matrix::const_iterator const_cm_it;

/*
  class RndNumTable
    matrix of randomly generated contin_t of sample_count rows and arity columns
*/
class RndNumTable : public contin_matrix
{
public:
    //constructor
    RndNumTable() {}
    RndNumTable(int sample_count, int arity, opencog::RandGen& rng,
                double max_randvalue = 1.0, double min_randvalue = -1.0);
};

/*
  class contin_table
    contains sample_count evaluations obtained by evaluating t, a tree, over
    a RndNumTable rnt.
    assumption : t has only contin inputs and output
*/
class contin_table : public contin_vector   //a column of results
{
public:
    //typedef contin_vector super;

    //constructors
    contin_table() { }
    contin_table(const combo_tree& t, const RndNumTable& rnt, opencog::RandGen& rng);
    template<typename Func>
    contin_table(const Func& f, const RndNumTable& rnt) {
        foreach(const contin_vector& v, rnt)
        push_back(f(v.begin(), v.end()));
    }

    //equality operator
    bool operator==(const contin_table& ct) const;
    bool operator!=(const contin_table& ct) const {
        return !operator==(ct);
    }

    contin_t abs_distance(const contin_table& other) const;
};

//-------------------------------------------------------------------------
// Mixed table
//-------------------------------------------------------------------------

class mixed_table
{
    typedef type_tree::iterator type_pre_it;
    typedef type_tree::sibling_iterator type_sib_it;
    //Vector of bool or contin, depending on the output type of combo_tree
    //size = 2^|boolean inputs| * _rnt.size()
    //each slice of rnt.size() corresponds to a particular boolean input
    //all possible boolean inputs are enumerated in lexico order as for
    //truth_table
    //each element in a slice of rnt.size() corresponds to the result of
    //a particular contin input, all obtained from rnt
    std::vector<variant<bool, contin_t> > _vt;
    //NOTE : can be a bit optimized by using
    //variant<std::vector<bool>,std::vector<contin_t> > _vt;
    //instead

    int _contin_arg_count; //number of contin arguments
    int _bool_arg_count; //number of boolean arguments

    boost::unordered_map<int, int> _arg_map; //link the argument index with the
    //boolean or contin index, that is
    //for instance if the inputs are
    //(bool, contin, contin, bool, bool)
    //the the map is
    //(<0,0>,<1,0>,<2,1>,<3,1>,<4,2>)
    std::vector<int> _bool_arg; //link the ith bool arg with arg index
    std::vector<int> _contin_arg; //link the ith contin arg with arg index
    type_tree _prototype;
    type_node _output_type;

    RndNumTable _rnt;

    //take a prototype, set _prototype, _contin_arg_count, _bool_arg_count
    //_arg_map, _bool_arg, _contin_arg and _output_type
    void set_prototype(const type_tree& prototype) {
        _contin_arg_count = 0;
        _bool_arg_count = 0;
        _prototype = prototype;
        type_pre_it proto_it = prototype.begin();
        int arg_idx = 0;
        for (type_sib_it i = proto_it.begin();
             i != proto_it.end(); ++i, ++arg_idx) {
            if (type_pre_it(i) == _prototype.last_child(proto_it)) {
                _output_type = *i;
            } else {
                if (*i == id::boolean_type) {
                    std::pair<int, int> p(arg_idx, _bool_arg_count);
                    _arg_map.insert(p);
                    _bool_arg.push_back(arg_idx);
                    _bool_arg_count++;
                } else if (*i == id::contin_type) {
                    std::pair<int, int> p(arg_idx, _contin_arg_count);
                    _arg_map.insert(p);
                    _contin_arg.push_back(arg_idx);
                    _contin_arg_count++;
                }
            }
        }
    }

public:
    //constructors
    mixed_table() {}
    mixed_table(const combo_tree& tr, const RndNumTable& rnt,
                const type_tree& prototype, opencog::RandGen& rng) {
        _rnt = rnt;
        if (prototype.empty()) {
            type_tree inferred_proto = infer_type_tree(tr);
            set_prototype(inferred_proto);
        } else set_prototype(prototype);
        //populate _vt
        //works only for _bool_arg_count < 64
        unsigned long int bool_table_size = 1 << _bool_arg_count;
        for (unsigned long int i = 0; i < bool_table_size; ++i) {
            for (int bai = 0; bai < _bool_arg_count; ++bai) //bool arg index
                binding(_bool_arg[bai] + 1) = bool_to_vertex((i >> bai) % 2);
            for (const_cm_it si = rnt.begin(); si != rnt.end(); ++si) {
                int cai = 0; //contin arg index
                for (const_cv_it j = (*si).begin(); j != (*si).end(); ++j, ++cai)
                    binding(_contin_arg[cai] + 1) = *j;
                vertex e = eval_throws(rng, tr);
                _vt.push_back(is_boolean(e) ? vertex_to_bool(e) : get_contin(e));
            }
        }
    }
    //access mesthods
    const std::vector<variant<bool, contin_t> >& get_vt() const {
        return _vt;
    }

    boost::unordered_map<int, int> & get_arg_idx_map()  {
		return _arg_map;
    }

    //equality operator
    bool operator==(const mixed_table& mt) const {
        std::vector<variant<bool, contin_t> >::const_iterator il = _vt.begin();
        for (std::vector<variant<bool, contin_t> >::const_iterator
                ir = mt._vt.begin(); ir != mt._vt.end(); ++il, ++ir) {
            if (boost::get<bool>(&(*il))) {
                if (boost::get<bool>(*il) != boost::get<bool>(*ir))
                    return false;
            } else if (!isApproxEq(boost::get<contin_t>(*il),
                                   boost::get<contin_t>(*ir)))
                return false;
        }
        return true;
    }
    bool operator!=(const mixed_table& mt) const {
        return !operator==(mt);
    }
};



//WARNING : should be in eval_action.h
//but could not do that due to dependency issues
//-------------------------------------------------------------------------
// mixed_action table
//-------------------------------------------------------------------------

//this table permits to represent inputs and outputs of combo trees
//that have boolean, continuous, action_result arguments and/or output

class mixed_action_table
{
    typedef type_tree::iterator type_pre_it;
    typedef type_tree::sibling_iterator type_sib_it;
    //Vector of bool or contin, depending on the output type of combo_tree
    //size = 2^|boolean+action_result inputs| * _rnt.size()
    //each slice of rnt.size() corresponds to a particular
    //boolean+action_result input, all possible boolean+action_result inputs
    //are enumerated in lexico order as for truth_table
    //each element in a slice of rnt.size() corresponds to the result of
    //a particular contin input, all obtained from rnt
    std::vector<variant<bool, contin_t> > _vt;
    //NOTE : can be a bit optimized by using
    //variant<std::vector<bool>,std::vector<contin_t> > _vt;
    //instead

    int _bool_arg_count; //number of boolean arguments
    int _contin_arg_count; //number of contin arguments
    int _action_arg_count; //number of action_result arguments

    boost::unordered_map<int, int> _arg_map; //link the argument index with the
    //boolean, action_result or contin
    //index, that is
    //for instance if the inputs are
    //(bool, contin, contin, bool, action)
    //the the map is
    //(<0,0>,<1,0>,<2,1>,<3,1>,<4,0>)
    std::vector<int> _bool_arg; //link the ith bool arg with arg index
    std::vector<int> _contin_arg; //link the ith contin arg with arg index
    std::vector<int> _action_arg; //link the ith action arg with arg index
    type_tree _prototype;
    type_node _output_type;

    RndNumTable _rnt;

    //take a prototype, set _prototype, _contin_arg_count, _bool_arg_count,
    //_action_arg_count, _arg_map, _bool_arg, _contin_arg, action_arg
    //and _output_type
    void set_prototype(const type_tree& prototype) {
        _contin_arg_count = 0;
        _bool_arg_count = 0;
        _action_arg_count = 0;
        _prototype = prototype;
        type_pre_it proto_it = prototype.begin();
        int arg_idx = 0;
        for (type_sib_it i = proto_it.begin();i != proto_it.end();++i, ++arg_idx) {
            if (type_pre_it(i) == _prototype.last_child(proto_it)) {
                _output_type = *i;
            } else {
                if (*i == id::boolean_type) {
                    std::pair<int, int> p(arg_idx, _bool_arg_count);
                    _arg_map.insert(p);
                    _bool_arg.push_back(arg_idx);
                    _bool_arg_count++;
                } else if (*i == id::contin_type) {
                    std::pair<int, int> p(arg_idx, _contin_arg_count);
                    _arg_map.insert(p);
                    _contin_arg.push_back(arg_idx);
                    _contin_arg_count++;
                } else if (*i == id::action_result_type) {
                    std::pair<int, int> p(arg_idx, _action_arg_count);
                    _arg_map.insert(p);
                    _action_arg.push_back(arg_idx);
                    _action_arg_count++;
                }
            }
        }
    }

public:
    //constructors
    mixed_action_table() {}
    mixed_action_table(const combo_tree& tr, const RndNumTable& rnt,
                       const type_tree& prototype, opencog::RandGen& rng) {
        _rnt = rnt;
        if (prototype.empty()) {
            type_tree inferred_proto = infer_type_tree(tr);
            set_prototype(inferred_proto);
        } else set_prototype(prototype);
        //populate _vt
        //WARNING : works only for _bool_arg_count + _action_arg_count <= 64
        int bool_action_arg_count = (_bool_arg_count + _action_arg_count);
        unsigned long int bool_action_table_size = 1 << bool_action_arg_count;
        for (unsigned long int i = 0; i < bool_action_table_size; ++i) {
            //bai stands for bool arg index
            for (int bai = 0; bai < _bool_arg_count; ++bai)
                binding(_bool_arg[bai] + 1) = bool_to_vertex((i >> bai) % 2);
            //aai stands for action arg index
            for (int aai = 0; aai < _action_arg_count; ++aai) {
                int arg_idx = _action_arg[aai] + 1;
                bool arg_val = (i >> (aai + _bool_arg_count)) % 2;
                binding(arg_idx) = (arg_val ? id::action_success : id::action_failure);
            }
            //contin populate
            for (const_cm_it si = rnt.begin();si != rnt.end();++si) {
                int cai = 0; //contin arg index
                for (const_cv_it j = (*si).begin(); j != (*si).end(); ++j, ++cai)
                    binding(_contin_arg[cai] + 1) = *j;
                vertex e = eval_throws(rng, tr);
                if (is_boolean(e))
                    _vt.push_back(vertex_to_bool(e));
                else if (is_action_result(e))
                    _vt.push_back(e == id::action_success ? true : false);
                else if (is_contin(e))
                    _vt.push_back(get_contin(e));
                //should never get to this part
                else OC_ASSERT(false,
                                       "should never get to this part.");
            }
        }
    }
    //access mesthods
    const std::vector<variant<bool, contin_t> >& get_vt() const {
        return _vt;
    }

    //equality operator
    bool operator==(const mixed_action_table& mat) const {
        std::vector<variant<bool, contin_t> >::const_iterator il = _vt.begin();
        for (std::vector<variant<bool, contin_t> >::const_iterator
                ir = mat._vt.begin(); ir != mat._vt.end(); ++il, ++ir) {
            if (boost::get<bool>(&(*il))) {
                if (boost::get<bool>(*il) != boost::get<bool>(*ir))
                    return false;
            } else if (!isApproxEq(boost::get<contin_t>(*il),
                                   boost::get<contin_t>(*ir)))
                return false;
        }
        return true;
    }
    bool operator!=(const mixed_action_table& mat) const {
        return !operator==(mat);
    }
};


} //~namespace combo

inline std::ostream& operator<<(std::ostream& out,
                                const combo::truth_table& tt)
{
    for (std::vector<bool>::const_iterator it = tt.begin();it != tt.end();++it)
        out << *it << " ";
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::contin_matrix& cm)
{
    for (combo::const_cm_it i = cm.begin(); i != cm.end(); ++i) {
        for (combo::const_cv_it j = (*i).begin(); j != (*i).end(); ++j) {
            out << *j << '\t';
        }
        out << std::endl;
    }
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::RndNumTable& rnt)
{
    out << static_cast<combo::contin_matrix>(rnt);
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::contin_table& ct)
{
    for (combo::const_cv_it it = ct.begin(); it != ct.end(); ++it)
        out << *it << " ";
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::mixed_table& mt)
{
    const std::vector<boost::variant<bool, combo::contin_t> >& vt = mt.get_vt();
    for (unsigned i = 0; i != vt.size(); ++i)
        out << (boost::get<combo::contin_t>(&vt[i]) ?
                boost::get<combo::contin_t>(vt[i]) :
                boost::get<bool>(vt[i])) << " ";
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::mixed_action_table& mat)
{
    const std::vector<boost::variant<bool, combo::contin_t> >& vt = mat.get_vt();
    for (unsigned i = 0; i != vt.size(); ++i)
        out << (boost::get<combo::contin_t>(&vt[i]) ?
                boost::get<combo::contin_t>(vt[i]) :
                boost::get<bool>(vt[i])) << " ";
    return out;
}

namespace boost
{
inline size_t hash_value(const combo::truth_table& tt)
{
    return hash_range(tt.begin(), tt.end());
}
} //~namespace boost

#endif
