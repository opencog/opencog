/*
 * opencog/comboreduct/combo/vertex.h
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
#ifndef _COMBO_VERTEX_H
#define _COMBO_VERTEX_H

#include <boost/functional/hash.hpp>
#include <boost/variant.hpp>
#include <boost/range.hpp>

#include <iostream>

#include <opencog/util/tree.h>
#include <opencog/util/numeric.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>

#include "action.h"
#include "builtin_action.h"
#include "action_symbol.h"
#include "perception.h"
#include "definite_object.h"
#include "indefinite_object.h"
#include "message.h"
//#include "procedure_call.h"

namespace combo
{

class procedure_call_base;

typedef const procedure_call_base* procedure_call;
typedef std::set<procedure_call> procedure_call_set;
typedef procedure_call_set::iterator procedure_call_set_it;
typedef procedure_call_set::const_iterator procedure_call_const_it;


//this idiom allows builtin to live in namespace combo, but all
//ids to be hidden inside the namespace id
namespace id {
enum builtin {
    null_vertex = 0,
    logical_and, logical_or, logical_not, logical_true, logical_false,
    contin_if,
    boolean_if,
    plus, times, div, exp, log, sin,
    greater_than_zero,
    impulse,
    rand,         //random contin_t in [0,1) FIXME TODO : update reduct rules
    builtin_count //to get the number of builtin
};
}
typedef id::builtin builtin;

//this idiom allows wild_card to live in namespace combo, but all
//ids to be hidden inside the namespace id
namespace id {
enum wild_card {
    asterisk = 0,
    wild_card_count //to get the number of wild_card
};
}
typedef id::wild_card wild_card;

/*
  class argument
    represents the index, attribute idx, of an input variable
    of a function coded into a combo tree. In the case of a boolean
    argument a negative value corresponds to a negative literal.
    idx == 0 is invalide.
    For example idx == -3 represents the literal NOT(#3) where #3 is the
    third argument of the function.
*/
class argument
{
public:
    explicit argument(int i) : idx(i) {
        OC_ASSERT(idx != 0,
                         "idx should be different fro zero.");
    }
    int idx;

    void negate() {
        idx = -idx;
    }
    bool is_negated() const {
        return idx < 0;
    }
    bool operator<(argument rhs) const {
        static opencog::absolute_value_order<int> comp;
        return comp(idx, rhs.idx);
    }
    int abs_idx() const {
        return idx < 0 ? -idx : idx;
    }
    //returns 0 for argument of idx 1, 1 for idx 2, and so on
    int abs_idx_from_zero() const {
        return (idx < 0 ? -idx : idx) - 1;
    }
    //check if idx is in the possible range given arity a
    //formally idx is in the range of a:
    //if idx==0 then it is not valid anyway
    //else if a>0 (that is no arg_list) then idx<=a
    //else if a<0 then idx<-a
    bool is_idx_valid(arity_t a) const {
        return (idx == 0 ? false : (a > 0 ? abs_idx() <= a : abs_idx() < -a));
    }
    bool operator==(argument rhs) const {
        return idx == rhs.idx;
    }
    bool operator!=(argument rhs) const {
        return idx != rhs.idx;
    }
};

typedef double contin_t;

typedef boost::variant < builtin,
                         wild_card,
                         argument,
                         contin_t,
                         action,
                         builtin_action,
                         perception,
                         definite_object,
                         indefinite_object,
                         message,
                         procedure_call,
                         action_symbol > vertex;

typedef std::vector<vertex> argument_list;
typedef argument_list::iterator argument_list_it;
typedef argument_list::const_iterator argument_list_const_it;
typedef std::vector<argument_list> argument_list_list;
typedef argument_list_list::iterator argument_list_list_it;
typedef argument_list_list::const_iterator argument_list_list_const_it;

} //~namespace combo

std::ostream& operator<<(std::ostream&, const combo::builtin&);
std::ostream& operator<<(std::ostream&, const combo::wild_card&);
std::ostream& operator<<(std::ostream&, const combo::argument&);
std::ostream& operator<<(std::ostream&, const combo::vertex&);

//builtin == vertex
inline bool operator==(const combo::vertex& v, combo::builtin h)
{
    if (const combo::builtin* vh = boost::get<combo::builtin>(&v))
        return (*vh == h);
    return false;
}
inline bool operator==(combo::builtin h, const combo::vertex& v)
{
    return (v == h);
}
inline bool operator!=(const combo::vertex& v, combo::builtin h)
{
    return !(v == h);
}
inline bool operator!=(combo::builtin h, const combo::vertex& v)
{
    return !(v == h);
}

//wild_card == vertex
inline bool operator==(const combo::vertex& v, combo::wild_card w)
{
    if (const combo::wild_card* vw = boost::get<combo::wild_card>(&v))
        return (*vw == w);
    return false;
}
inline bool operator==(combo::wild_card w, const combo::vertex& v)
{
    return (w == v);
}
inline bool operator!=(const combo::vertex& v, combo::wild_card w)
{
    return !(v == w);
}
inline bool operator!=(combo::wild_card w, const combo::vertex& v)
{
    return !(v == w);
}

//action == vertex
inline bool operator==(const combo::vertex& v, combo::action a)
{
    if (const combo::action* va = boost::get<combo::action>(&v))
        return (*va == a);
    return false;
}
inline bool operator==(combo::action a, const combo::vertex& v)
{
    return (v == a);
}
inline bool operator!=(const combo::vertex& v, combo::action a)
{
    return !(v == a);
}
inline bool operator!=(combo::action a, const combo::vertex& v)
{
    return !(v == a);
}

//builtin_action == vertex
inline bool operator==(const combo::vertex& v, combo::builtin_action a)
{
    if (const combo::builtin_action* va = boost::get<combo::builtin_action>(&v))
        return (*va == a);
    return false;
}
inline bool operator==(combo::builtin_action a, const combo::vertex& v)
{
    return (v == a);
}
inline bool operator!=(const combo::vertex& v, combo::builtin_action a)
{
    return !(v == a);
}
inline bool operator!=(combo::builtin_action a, const combo::vertex& v)
{
    return !(v == a);
}

//perception == vertex
inline bool operator==(const combo::vertex& v, combo::perception p)
{
    if (const combo::perception* vp = boost::get<combo::perception>(&v))
        return (*vp == p);
    return false;
}
inline bool operator==(combo::perception p, const combo::vertex& v)
{
    return (v == p);
}
inline bool operator!=(const combo::vertex& v, combo::perception p)
{
    return !(v == p);
}
inline bool operator!=(combo::perception p, const combo::vertex& v)
{
    return !(v == p);
}

//definite_object == vertex
inline bool operator==(const combo::vertex& v, combo::definite_object d)
{
    if (const combo::definite_object*
            vd = boost::get<combo::definite_object>(&v))
        return (*vd == d);
    return false;
}
inline bool operator==(combo::definite_object d, const combo::vertex& v)
{
    return (v == d);
}
inline bool operator!=(const combo::vertex& v, combo::definite_object d)
{
    return !(v == d);
}
inline bool operator!=(combo::definite_object d, const combo::vertex& v)
{
    return !(v == d);
}

//indefinite_object == vertex
inline bool operator==(const combo::vertex& v, combo::indefinite_object i)
{
    if (const combo::indefinite_object*
            vi = boost::get<combo::indefinite_object>(&v))
        return (*vi == i);
    return false;
}
inline bool operator==(combo::indefinite_object i, const combo::vertex& v)
{
    return (v == i);
}
inline bool operator!=(const combo::vertex& v, combo::indefinite_object i)
{
    return !(v == i);
}
inline bool operator!=(combo::indefinite_object i, const combo::vertex& v)
{
    return !(v == i);
}

//message == vertex
inline bool operator==(const combo::vertex& v, combo::message m)
{
    if (const combo::message* vm = boost::get<combo::message>(&v))
        return (*vm == m);
    return false;
}
inline bool operator==(combo::message m, const combo::vertex& v)
{
    return (v == m);
}
inline bool operator!=(const combo::vertex& v, combo::message m)
{
    return !(v == m);
}
inline bool operator!=(combo::message m, const combo::vertex& v)
{
    return !(v == m);
}

//action_symbol == vertex
inline bool operator==(const combo::vertex& v, combo::action_symbol i)
{
    if (const combo::action_symbol*
            vi = boost::get<combo::action_symbol>(&v))
        return (*vi == i);
    return false;
}
inline bool operator==(combo::action_symbol i, const combo::vertex& v)
{
    return (v == i);
}
inline bool operator!=(const combo::vertex& v, combo::action_symbol i)
{
    return !(v == i);
}
inline bool operator!=(combo::action_symbol i, const combo::vertex& v)
{
    return !(v == i);
}

namespace boost
{
//don't know why this is needed *in namespace boost*, but it is,
//for e.g. calling a generic stl function that compares vertices for inequality
inline bool operator!=(const combo::vertex& v1, const combo::vertex& v2)
{
    return !(v1 == v2);
}

inline size_t hash_value(const combo::message& m)
{
    return hash_value(m.getContent());
}

inline size_t hash_value(const combo::vertex& v)
{
    static const size_t c1 = size_t(combo::id::builtin_count);
    //it is likely that a combo will rarely have over 15 arguments
    static const size_t c2 = c1 + 15;
    static const size_t c3 = c2 + size_t(combo::id::action_count);
    static const size_t c_last = c3;

    //there will be some overlap between contin_t, definite_object, message
    //and procedure_call but this overlap is unavoidable

    if (const combo::builtin* h = boost::get<combo::builtin>(&v))
        return size_t(*h);
    if (const combo::wild_card* w = boost::get<combo::wild_card>(&v))
        return size_t(*w);
    if (const combo::argument* a = boost::get<combo::argument>(&v))
        return size_t(a->idx * (a->is_negated() + 2)) + c1;
    if (const combo::contin_t* c = boost::get<combo::contin_t>(&v)) {
        size_t tmp = c_last;
        hash_combine(tmp, hash_value(*c));
        return tmp;
    }
    if (const combo::action* a = boost::get<combo::action>(&v))
        return size_t(*a) + c2;
    if (const combo::builtin_action* b = boost::get<combo::builtin_action>(&v)) {
        size_t tmp = c_last;
        hash_combine(tmp, hash_value(*b));
        return tmp;
    }
    if (const combo::perception* p = boost::get<combo::perception>(&v)) {
        size_t tmp = c_last;
        hash_combine(tmp, hash_value(*p));
        return tmp;
    }
    if (const combo::definite_object*
            d = boost::get<combo::definite_object>(&v)) {
        size_t tmp = c_last;
        hash_combine(tmp, hash_value(*d));
        return tmp;
    }
    if (const combo::indefinite_object*
            i = boost::get<combo::indefinite_object>(&v)) {
        size_t tmp = c_last;
        hash_combine(tmp, hash_value(*i));
        return tmp;
    }
    if (const combo::message* m = boost::get<combo::message>(&v)) {
        size_t tmp = c_last;
        hash_combine(tmp, hash_value(*m));
        return tmp;
    }
    if (const combo::procedure_call* pc = boost::get<combo::procedure_call>(&v)) {
        size_t tmp = c_last;
        hash_combine(tmp, hash_value(*pc));
        return tmp;
    }
    if (const combo::action_symbol* as = boost::get<combo::action_symbol>(&v)) {
        size_t tmp = c_last;
        hash_combine(tmp, hash_value(*as));
        return tmp;
    }

    OC_ASSERT(false, "A case is missing");
    return 0;
}
} //~namespace boost

namespace combo
{
//typedef util::hash_set<vertex,boost::hash<vertex> > vset;
//typedef std::set<vertex> vset;
typedef opencog::tree<vertex> combo_tree;

template<typename T>
inline bool is_associative(const T& v)
{
    return (v == id::logical_and || v == id::logical_or ||
            v == id::plus || v == id::times ||
            //actions
            v == id::sequential_and || v == id::sequential_or ||
            v == id::sequential_exec);
}
template<typename T>
inline bool is_commutative(const T& v)
{
    return (v == id::logical_and || v == id::logical_or ||
            v == id::plus || v == id::times
            || is_symmetric(v));
}

//properties of perceptions

template<typename T>
inline bool is_ultrametric(const T& v)
{
    if (is_perception(v))
        return get_perception(v)->is_ultrametric();
    else return false;
}

template<typename T>
inline bool is_transitive(const T& v)
{
    if (is_perception(v))
        return get_perception(v)->is_transitive();
    else return false;
}

template<typename T>
inline bool is_reflexive(const T& v)
{
    if (is_perception(v))
        return get_perception(v)->is_reflexive();
    else return false;
}

template<typename T>
inline bool is_irreflexive(const T& v)
{
    if (is_perception(v))
        return get_perception(v)->is_irreflexive();
    else return false;
}

template<typename T>
inline bool is_symmetric(const T& v)
{
    if (is_perception(v))
        return get_perception(v)->is_symmetric();
    else return false;
}

template<typename T>
inline bool is_identity_of_indiscernibles(const T& v)
{
    if (is_perception(v))
        return get_perception(v)->is_identity_of_indiscernibles();
    else return false;
}

inline bool is_procedure_call(const vertex& v)
{
    return (boost::get<procedure_call>(&v));
}

inline procedure_call get_procedure_call(const vertex& v)
{
    return (boost::get<procedure_call>(v));
}

inline bool is_action_symbol(const vertex& v)
{
    return (boost::get<action_symbol>(&v));
}

inline action_symbol get_action_symbol(const vertex& v)
{
    return (boost::get<action_symbol>(v));
}

inline bool is_indefinite_object(const vertex& v)
{
    return (boost::get<indefinite_object>(&v));
}

inline indefinite_object get_indefinite_object(const vertex& v)
{
    return (boost::get<indefinite_object>(v));
}

inline bool is_message(const vertex& v)
{
    return (boost::get<message>(&v));
}

inline message get_message(const vertex& v)
{
    return (boost::get<message>(v));
}

inline bool is_builtin(const vertex& v)
{
    return (boost::get<builtin>(&v));
}

inline builtin get_builtin(const vertex& v)
{
    return (boost::get<builtin>(v));
}

inline bool is_wild_card(const vertex& v)
{
    return (boost::get<wild_card>(&v));
}

inline wild_card get_wild_card(const vertex& v)
{
    return (boost::get<wild_card>(v));
}

inline bool is_contin(const vertex& v)
{
    return (boost::get<contin_t>(&v));
}
inline contin_t get_contin(const vertex& v)
{
    return (boost::get<contin_t>(v));
}
inline bool is_argument(const vertex& v)
{
    return (boost::get<argument>(&v));
}
inline argument& get_argument(vertex& v)
{
    return boost::get<argument>(v);
}
inline const argument& get_argument(const vertex& v)
{
    return boost::get<argument>(v);
}
inline bool is_negated(vertex& v)
{
    if (argument* a = boost::get<argument>(&v))
        return a->is_negated();
    return false;
}

inline bool is_action(const vertex& v)
{
    return (boost::get<action>(&v));
}

inline action get_action(const vertex& v)
{
    return (boost::get<action>(v));
}

inline bool is_builtin_action(const vertex& v)
{
    return (boost::get<builtin_action>(&v));
}
inline builtin_action get_builtin_action(const vertex& v)
{
    return (boost::get<builtin_action>(v));
}
inline bool is_action_result(const vertex& v)
{
    return (v == id::action_failure || v == id::action_success);
}

inline bool is_perception(const vertex& v)
{
    return (boost::get<perception>(&v));
}

inline perception get_perception(const vertex& v)
{
    return (boost::get<perception>(v));
}

inline bool is_definite_object(const vertex& v)
{
    return (boost::get<definite_object>(&v));
}

inline definite_object get_definite_object(const vertex& v)
{
    return (boost::get<definite_object>(v));
}

inline vertex bool_to_vertex(bool b)
{
    return (b ? id::logical_true : id::logical_false);
}
inline bool vertex_to_bool(const vertex& v)
{
    OC_ASSERT(v == id::logical_true || v == id::logical_false,
                     "vertex should be of logical types 'id::logical_true' or 'id::logical_false'.");
    return (v == id::logical_true);
}
//renamed negate_vertex to not enter in conflict with negate(string) of STL
inline vertex negate_vertex(const vertex& v)
{
    OC_ASSERT(v == id::logical_true || v == id::logical_false,
                     "vertex should be of logical types 'id::logical_true' or 'id::logical_false'.");
    return (v == id::logical_true ? id::logical_false : id::logical_true);
}

inline bool is_complement(const vertex& x, const vertex& y)
{
    if (const argument* ax = boost::get<argument>(&x)) {
        if (const argument* ay = boost::get<argument>(&y)) {
            return (ax->idx == -ay->idx);
        }
    }
    return false;
}

template<typename T>
inline bool is_boolean(const T& v)
{
    return (v == id::logical_true || v == id::logical_false);
}

template<typename T>
inline bool is_logical_operator(const T& v)
{
    return (v == id::logical_and || v == id::logical_or || v == id::logical_not);
}
template<typename T>
inline bool is_constant(const T& v)
{
    return (is_boolean(v) || is_contin(v) || is_action_result(v));
}

//copy a combo_tree, ignoring subtrees rooted in null vertices
void copy_without_null_vertices(combo_tree::iterator src,
                                combo_tree& dst_tr, combo_tree::iterator dst);

inline bool may_have_side_effects(combo_tree::iterator /*it*/)
{
    //TODO
    return false;
}

//input vertex and combo_tree functions

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
void str_to_vertex(const std::string& str, vertex& v)
{
    OC_ASSERT(!str.empty(), "input to string should not be empty.");
    //builtin
    if (str == "and" || str == "logical_and")
        v = id::logical_and;
    else if (str == "or" || str == "logical_or")
        v = id::logical_or;
    else if (str == "not" || str == "logical_not")
        v = id::logical_not;
    else if (str == "true" || str == "logical_true")
        v = id::logical_true;
    else if (str == "false" || str == "logical_false")
        v = id::logical_false;
    else if (str == "contin_if" || str == "contin_boolean_if")
        v = id::contin_if;
    else if (str == "boolean_if" || str == "boolean_boolean_if")
        v = id::boolean_if;
    else if (str == "+")
        v = id::plus;
    else if (str == "*")
        v = id::times;
    else if (str == "/")
        v = id::div;
    else if (str == "log")
        v = id::log;
    else if (str == "exp")
        v = id::exp;
    else if (str == "sin")
        v = id::sin;
    else if (str == "0<")
        v = id::greater_than_zero;
    else if (str == "impulse")
        v = id::impulse;
    else if (str == "rand")
        v = id::rand;
    else if (str == "null_vertex")
        v = id::null_vertex;
    //wild_card
    else if (str == "_*_")
        v = id::asterisk;
    //action
    else if (str == "and_seq")
        v = id::sequential_and;
    else if (str == "or_seq")
        v = id::sequential_or;
    else if (str == "exec_seq")
        v = id::sequential_exec;
    else if (str == "action_not")
        v = id::action_not;
    else if (str == "action_boolean_if" || str == "action_if") //second clause for backwards compatability
        v = id::action_if;
    else if (str == "boolean_action_if")
        v = id::boolean_action_if;
    else if (str == "contin_action_if")
        v = id::contin_action_if;
    else if (str == "action_action_if")
        v = id::action_action_if;
    else if (str == "action_failure" || str == "failure")
        v = id::action_failure;
    else if (str == "action_success" || str == "success")
        v = id::action_success;
    else if (str == "action_while")
        v = id::action_while;
    else if (str == "boolean_while")
        v = id::boolean_while;
    else if (str == "return_success")
        v = id::return_success;
    else if (str == "repeat_n")
        v = id::repeat_n;
    //argument
    else if (str[0] == '#') {
        int arg = boost::lexical_cast<int>(str.substr(1));
        OC_ASSERT(arg != 0, "arg value should be different from zero.");
        v = argument(arg);
    }
    //constant
    else {
        try {
            v = boost::lexical_cast<contin_t>(str);
        }

        //builtin_action, perception, definite_object, message
        //action_symbol, indefinite_object

        catch (boost::bad_lexical_cast&) {
            //check of message
            if (str.find(message::prefix()) == 0) { //it starts with message:
                std::string m_str = str.substr(message::prefix().size());
                //check that the first and the last character are \"
                //and take them off
                if (m_str.find('\"') == 0 && m_str.rfind('\"') == m_str.size() - 1) {
                    m_str.erase(m_str.begin());
                    m_str.erase(--m_str.end());
                    message m(m_str);
                    v = m;
                } else {
                    std::cout << "WARNING : " << "You probably forgot to place your message between doubles quotes in " << str << std::endl;
                }
            }
            //check for builtin_action
            else if (builtin_action ba = BUILTIN_ACTION::instance(str)) {
                v = ba;
            }
            //check for perception
            else if (perception p = PERCEPTION::instance(str)) {
                v = p;
            }
            //check for action symbol
            else if (action_symbol as = ACTION_SYMBOL::instance(str)) {
                v = as;
            }
            //check for indefinite_object
            else if (indefinite_object i = INDEFINITE_OBJECT::instance(str)) {
                v = i;
            }
            //then should be definite object
            else {
                v = str;
            }
        }
    }
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
vertex str_to_vertex(const std::string& str)
{
    vertex v;
    str_to_vertex<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(str, v);
    return v;
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
std::istream& stream_to_vertex(std::istream& in, vertex& v)
{
    std::string str;
    //use getline instead of in >> str to be sure to not
    //skip spaces in the message
    std::getline(in, str);
    str_to_vertex<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(str, v);
    return in;
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
void sub_strtree_to_combo_tree(const opencog::tree<std::string>& src,
                               opencog::tree<std::string>::iterator src_it,
                               combo_tree& dst, combo_tree::iterator dst_it)
{
    dst_it = dst.replace(dst_it, str_to_vertex<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(*src_it));
    dst.erase_children(dst_it);
    for (opencog::tree<std::string>::sibling_iterator sib = src_it.begin();
            sib != src_it.end(); ++sib)
        sub_strtree_to_combo_tree<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(src, opencog::tree<std::string>::iterator(sib), dst, dst.append_child(dst_it));
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
void strtree_to_combo_tree(const opencog::tree<std::string>& src, combo_tree& dst)
{
    dst = combo_tree(vertex());
    opencog::tree<std::string>::iterator src_it = src.begin();
    combo_tree::iterator dst_it = dst.begin();
    while (src_it != src.end()) {
        dst_it = dst.insert_after(dst_it, vertex());
        sub_strtree_to_combo_tree<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(src, src_it, dst, dst_it);
        src_it.skip_children();
        ++src_it;
    }
    dst.erase(dst.begin());
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
std::istream& stream_to_combo_tree(std::istream& in, combo_tree& tr)
{
    opencog::tree<std::string> tmp;
    in >> tmp;
    strtree_to_combo_tree<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(tmp, tr);
    return  in;
}

} //~namespace combo

namespace boost
{
template<>
struct range_iterator<combo::combo_tree::pre_order_iterator> {
    typedef boost::counting_iterator<combo::combo_tree::sibling_iterator> type;
};
template<>
struct range_const_iterator<combo::combo_tree::pre_order_iterator> {
    typedef boost::counting_iterator<combo::combo_tree::sibling_iterator> type;
};

template<>
struct range_iterator<combo::combo_tree> {
    typedef boost::counting_iterator<combo::combo_tree::iterator> type;
};
template<>
struct range_const_iterator<combo::combo_tree> {
    typedef boost::counting_iterator<combo::combo_tree::iterator> type;
};

/*  template<>
struct range_iterator<combo::combo_tree::p_iterator> {
  typedef boost::counting_iterator<combo::combo_tree::sibling_iterator> type;
};
template<>
struct range_const_iterator<combo::combo_tree::pre_order_iterator> {
  typedef boost::counting_iterator<combo::combo_tree::sibling_iterator> type;
  };*/
} //~namespace boost

#endif
