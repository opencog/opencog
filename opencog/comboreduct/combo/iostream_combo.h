/** iostream_combo.h --- 
 *
 * Copyright (C) 2012 OpenCog Foundation
 *
 * Authors: Nil Geisweiller <ngeiswei@gmail.com>
 *          Matt Chapman <Matt@NinjitsuWeb.com>
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


#ifndef _OPENCOG_IOSTREAM_PYTHON_H
#define _OPENCOG_IOSTREAM_PYTHON_H

#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include "vertex.h"

/**
 * Set of functions to read and write combo programs in various
 * format. Current formats supported are combo (default) and python.
 */

namespace opencog { namespace combo {

namespace fmt {
enum format {
    combo,
    python,
    format_count                // to get the number of formats
};
}
typedef fmt::format format;

// like operator<< but can choose the output format
std::ostream& ostream_builtin(std::ostream&, const builtin&, format f = fmt::combo);
std::ostream& ostream_argument(std::ostream&, const argument&, format f = fmt::combo);
std::ostream& ostream_vertex(std::ostream&, const vertex&, format f = fmt::combo);
std::ostream& ostream_combo_tree(std::ostream&, const combo_tree, format f = fmt::combo);
template<typename Iter>
std::ostream& ostream_combo_it(std::ostream& out, Iter it, format f = fmt::combo) {
    ostream_vertex(out, *it, f);
    if (it.number_of_children() > 0) {
        out << "(";
        auto sib = it.begin();
        ostream_combo_it(out, sib, f);
        for (; sib != it.end(); ++sib) {
            out << " ";
            ostream_combo_it(out, sib, f);
        }
        out << ")";
        if (f == fmt::python)
            out << ",";
    }
    return out;
}

// return false if the string has no match
inline bool builtin_str_to_vertex(const std::string& str, vertex& v)
{
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
    else if (str == "cond")
        v = id::cond;
    else if (str == "contin_if" || str == "contin_boolean_if")
        v = id::contin_if;
    else if (str == "+" || str == "plus")
        v = id::plus;
    else if (str == "*" || str == "times")
        v = id::times;
    else if (str == "/" || str == "div")
        v = id::div;
    else if (str == "ann")
        v = ann_type(0,id::ann);
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
    else return false;
    return true;
}

//* Concvert ANN string to vertex.
//* Return false if str is not of ANN type.
//*
//* ANN strings must begin with $N or $I
//* $Nxxx are ann_nodes and $Ixxx are ann_inputs
//
inline bool ann_str_to_vertex(const std::string& str, vertex& v)
{
    if (str[0] != '$')
        return false;

    if (str[1]=='N') {
        int arg = boost::lexical_cast<int>(str.substr(2));
        v = ann_type(arg,id::ann_node);
        return true;
    }
    if (str[1]=='I') {
        int arg = boost::lexical_cast<int>(str.substr(2));
        v = ann_type(arg,id::ann_input);
        return true;
    }
    return false;
}

//* Convert argument string to vertex.
//* Return false if str is not an argument.
//*
//* Arguments come in two forms: $n and !$n where n is an integer.
//
inline bool argument_str_to_vertex(const std::string& str, vertex& v)
{
    if (str[0] == '$') {
        arity_t arg = boost::lexical_cast<arity_t>(str.substr(1));
        OC_ASSERT(arg != 0, "arg value should be different than zero.");
        v = argument(arg);
        return true;
    }
    if (str[0] == '!' && str[1] == '$') {
        arity_t arg = boost::lexical_cast<arity_t>(str.substr(2));
        OC_ASSERT(arg != 0, "arg value should be different than zero.");
        v = argument(-arg);
        return true;
    }
    return false;
}

//* Convert string to a contin vertex.
//* Return false if the string is not a contin.
//*
//* A contin is any naked number, with or without a decimal point.
//
inline bool contin_str_to_vertex(const std::string& str, vertex& v)
{
    try {
        v = boost::lexical_cast<contin_t>(str);
    } catch (boost::bad_lexical_cast&) {
        return false;
    }
    return true;
}

//* Convert string to message vertex.
//* Return false if the string is not a message.
//*
//* message strings must begin with the prefix "message:"
//* and the message itself must be in quotes, so, for example:
//*     message:"this is a message"
//
inline bool message_str_to_vertex(const std::string& str, vertex& v)
{
    // It starts with message:
    if (str.find(message::prefix())) 
        return false;

    std::string m_str = str.substr(message::prefix().size());
    // Check that the first and the last character are \"
    // and take them off
    if (m_str.find('\"') == 0 && m_str.rfind('\"') == m_str.size() - 1) {
        m_str.erase(m_str.begin());
        m_str.erase(--m_str.end());
        message m(m_str);
        v = m;
    } else {
        std::cerr << "WARNING : You probably forgot to place your "
                     "message between doubles quotes, string was: "
                  << str << std::endl;
        return false;
    }
    return true;
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
void str_to_vertex(const std::string& str, vertex& v)
{
    OC_ASSERT(!str.empty(), "input to string should not be empty.");
    // builtin, ann, argument, constant and message
    // the order may matter
    if(builtin_str_to_vertex(str, v)
       || argument_str_to_vertex(str, v)
       || contin_str_to_vertex(str, v)
       || ann_str_to_vertex(str, v)
       || message_str_to_vertex(str, v)) {
        return;
    }
    // builtin_action
    else if (builtin_action ba = BUILTIN_ACTION::get_instance(str)) {
        v = ba;
    }
    // perception
    else if (perception p = PERCEPTION::get_instance(str)) {
        v = p;
    }
    // action symbol
    else if (action_symbol as = ACTION_SYMBOL::get_instance(str)) {
        v = as;
    }
    // indefinite_object
    else if (indefinite_object i = INDEFINITE_OBJECT::get_instance(str)) {
        v = i;
    }
    // should be definite object then
    else {
        // Any word character (alphanumeric characters plus the
        // underscore). If you find that too constraning feel free to
        // relax.
        static const boost::regex e("[\\w-]+");
        OC_ASSERT(boost::regex_match(str, e),
                  "Lexical error: '%s' cannot be a definite_object", str.c_str());
        v = str;
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
void sub_strtree_to_combo_tree(const tree<std::string>& src,
                               tree<std::string>::iterator src_it,
                               combo_tree& dst, combo_tree::iterator dst_it)
{
    dst_it = dst.replace(dst_it, str_to_vertex<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(*src_it));
    dst.erase_children(dst_it);
    for (tree<std::string>::sibling_iterator sib = src_it.begin();
            sib != src_it.end(); ++sib)
        sub_strtree_to_combo_tree<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(src, tree<std::string>::iterator(sib), dst, dst.append_child(dst_it));
}

template<class BUILTIN_ACTION, class PERCEPTION, class ACTION_SYMBOL, class INDEFINITE_OBJECT>
void strtree_to_combo_tree(const tree<std::string>& src, combo_tree& dst)
{
    dst = combo_tree(vertex());
    tree<std::string>::iterator src_it = src.begin();
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
    tree<std::string> tmp;
    in >> tmp;
    strtree_to_combo_tree<BUILTIN_ACTION, PERCEPTION, ACTION_SYMBOL, INDEFINITE_OBJECT>(tmp, tr);
    return  in;
}

/**
 * ph2l where ph == "place holder" and l == "label"
 *
 * This method replaces place holders by labels in a string
 * containing a combo expression.
 *
 * For instance, "and($1 $2)" would be replaced by "and($fat $pretty)"
 * given the vector of labels {"fat", "pretty"}
 */
std::string ph2l(const std::string& ce,
                 const std::vector<std::string>& labels);

/**
 * l2ph where ph == "place holder" and l == "label"
 *
 * This is the converse of ph2l. Given, for instance "and($fat $pretty)"
 * it returns "and($1 $2)".  If a variable is not in labels
 * (that is it doesn't correspond to a place holder) (???)
 */
std::string l2ph(const std::string& ce,
                 const std::vector<std::string>& labels);

std::ostream& operator<<(std::ostream&, const ann_type&);
std::ostream& operator<<(std::ostream&, const builtin&);
std::ostream& operator<<(std::ostream&, const wild_card&);
std::ostream& operator<<(std::ostream&, const argument&);
// output argument $n when positive, !$n when negative 
std::ostream& ostream_abbreviate_literal(std::ostream&, const argument&);
std::ostream& operator<<(std::ostream&, const vertex&);

}} // ~ namespace opencog::combo

#endif // _OPENCOG_IOSTREAM_PYTHON_H
