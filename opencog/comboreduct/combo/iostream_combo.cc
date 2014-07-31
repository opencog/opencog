/** iostream_combo.cc --- 
 *
 * Copyright (C) 2012 Nil Geisweiller
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

#include "iostream_combo.h"
#include "procedure_call.h"

namespace opencog { namespace combo {

using namespace std;
using namespace boost;

// uncomment this to output a negative literal !$n instead of not($n)
#define ABBREVIATE_NEGATIVE_LITERAL

// return false if the string has no match
bool builtin_str_to_vertex(const std::string& str, vertex& v)
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
    else if (str == "list")
        v = id::list;
    else if (str == "car")
        v = id::car;
    else if (str == "cdr")
        v = id::cdr;
    else if (str == "cons")
        v = id::cons;
    else if (str == "foldr")
        v = id::foldr;
    else if (str == "foldl")
        v = id::foldl;
    else if (str == "->" || str == "lambda")
        v = id::lambda;
    else if (str == "apply")
        v = id::apply;
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

//* Convert ANN string to vertex.
//* Return false if str is not of ANN type.
//*
//* ANN strings must begin with $N or $I
//* $Nxxx are ann_nodes and $Ixxx are ann_inputs
//
bool ann_str_to_vertex(const std::string& str, vertex& v)
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
bool argument_str_to_vertex(const std::string& str, vertex& v)
{
    if (str[0] == '$') {
        arity_t arg = 0;
        try {
            arg = boost::lexical_cast<arity_t>(str.substr(1));
        } catch (boost::bad_lexical_cast&) {
            // The ANN types start with a dollar sign too, but those
            // dollar signs are followed by not-a-number.
            return false;
        }

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
bool contin_str_to_vertex(const std::string& str, vertex& v)
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
bool message_str_to_vertex(const std::string& str, vertex& v)
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
        return true;
    } 

    std::cerr << "WARNING : You probably forgot to place your "
                 "message between doubles quotes, string was: "
              << str << std::endl;
    return false;
}

//* Convert string to enum vertex.
//* Return false if the string is not an enum.
//*
//* enum strings must begin with the prefix "enum:"
//* The enum itself may or may not be in quotes, so, for example:
//*     enum:"this is an enum with whitespace"
//*     enum:this_is_an_enum_without_whitespace
//
bool enum_str_to_vertex(const std::string& str, vertex& v)
{
    // It starts with "enum:"
    if (str.find(enum_t::prefix())) 
        return false;

    std::string m_str = str.substr(enum_t::prefix().size());
    // Check that the first and the last character are \"
    // and take them off
    if (m_str.find('\"') == 0 && m_str.rfind('\"') == m_str.size() - 1) {
        m_str.erase(m_str.begin());
        m_str.erase(--m_str.end());
    } 

    enum_t e(m_str);
    v = e;
    return true;
}

ostream& ostream_builtin(ostream& out, const builtin& h, format f)
{
    switch (f) {
    case fmt::python:
        switch (h) {
        case id::null_vertex:
            return out << "null_vertex";
        case id::logical_and:
            return out << "ands";
        case id::logical_or:
            return out << "ors";
        case id::logical_not:
            return out << "not";
        case id::logical_true:
            return out << "True";
        case id::logical_false:
            return out << "False";
        default:
            return out << "Builtin: " << (unsigned) h << " unknown";
        }
    case fmt::combo:
        switch (h) {
        case id::null_vertex:
            return out << "null_vertex";
        case id::logical_and:
            return out << "and";
        case id::logical_or:
            return out << "or";
        case id::logical_not:
            return out << "not";
        case id::logical_true:
            return out << "true";
        case id::logical_false:
            return out << "false";
        case id::plus:
            return out << "+";
        case id::times:
            return out << "*";
        case id::div:
            return out << "/";
        case id::log:
            return out << "log";
        case id::exp:
            return out << "exp";
        case id::sin:
            return out << "sin";
        case id::greater_than_zero:
            return out << "0<";
        case id::impulse:
            return out << "impulse";
        case id::rand:
            return out << "rand";
        case id::list:
            return out << "list";
        case id::car:
            return out << "car";
        case id::cdr:
            return out << "cdr";
        case id::cons:
            return out << "cons";
        case id::foldr:
            return out << "foldr";
        case id::foldl:
            return out << "foldl";
        case id::lambda:
            return out << "->";
        case id::apply:
            return out << "apply";
        case id::contin_if:
            return out << "contin_if";
        case id::cond:
            return out << "cond";
        case id::equ:
            return out << "equ";
        default:
            return out << "Builtin " << (unsigned) h << " unknown";
        }
    default:
        return out << "Format " << f << " unknown";
    }
}

ostream& ostream_argument(ostream& out, const argument& a, format f)
{
    switch(f) {
    case fmt::python:
        if (a.is_negated())        
            return out << "not(i[" << -a.idx - 1 << "]),";
        return out << "i[" << a.idx - 1 << "],";
    case fmt::combo:
#ifdef ABBREVIATE_NEGATIVE_LITERAL
        return ostream_abbreviate_literal(out, a);
#else
        if (a.is_negated())        
            return out << "not($" << -a.idx << ")";
        return out << "$" << a.idx << vm;
#endif
    default:
        return out << "Format " << f << "unknown";
    }    
}

ostream& ostream_vertex(ostream& out, const vertex& v, format f)
{
    // Handle the most likely types first.
    if (const argument* a = get<argument>(&v))
        return ostream_argument(out, *a, f);
    if (const builtin* h = get<builtin>(&v))
        return ostream_builtin(out, *h, f);
    if (const enum_t* m = get<enum_t>(&v))
        return out << m->getContent();

    // XXX ?? Ahem, won't calling out<<(*m) just lead to infinite
    // recursion ?? 
    if (const ann_type* z = get<ann_type>(&v))
        return out << (*z);
    if (const wild_card* w = get<wild_card>(&v))
        return out << (*w);
    if (const action* act = get<action>(&v))
        return out << (*act);
    if (const builtin_action* aact = get<builtin_action>(&v))
        return out << (*aact);
    if (const perception* per = get<perception>(&v))
        return out << (*per);
    if (const indefinite_object* iot = get<indefinite_object>(&v))
        return out << (*iot);
    if (const message* m = get<message>(&v))
        return out << m->getContent();
    if (const definite_object* dot = get<definite_object>(&v))
        return out << (*dot);
    if (const action_symbol* as = get<action_symbol>(&v))
        return out << (*as);
    if (const procedure_call* cp = get<procedure_call>(&v))
        return out << (*cp);

    try {
        return out << get<contin_t>(v);
    } catch (...) {
        OC_ASSERT(false, "Don't know how to print this type");
    }

    return out;
}

std::ostream& ostream_combo_tree(std::ostream& out, const combo_tree ct, format f) {
    for (combo_tree::iterator it=ct.begin(); it!=ct.end(); ++it) {
        ostream_combo_it(out, it, f);
        it.skip_children();
        out << " ";
    }
    return out;
}
        
string ph2l(const string& ce, const vector<string>& labels)
{
    /// @todo the implementation could be done in 2 lines with
    /// boost.regex with boost version 1.42 or above because then we
    /// can use Formatter as callback, but we're stuck with boost 1.38
    /// :-(
    /// @todo we're not stuck any more with boost 1.38!!!
    string res;
    string match;
    bool matching = false;
    for (char c : ce) {
        if (!matching) {
            res += c;
            if (c == '$') // matching starts
                matching = true;
        } else {
            if(c == ' ' || c == ')' || c == '\n') { //matching ends
                res += labels[lexical_cast<arity_t>(match) - 1] + c;
                match.clear();
                matching = false;
            } else // matching goes
                match += c;
        }
    }
    // if a matching is going on flush to the result
    if (matching)
        res += labels[lexical_cast<arity_t>(match) - 1];
    return res;
}

std::string l2ph(const std::string& ce,
                 const std::vector<std::string>& labels)
{
    /// @todo the implementation could be done in 2 lines with
    /// boost.regex with boost version 1.42 or above because then we
    /// can use Formatter as callback, but we're stuck with boost 1.38
    /// :-(
    /// @todo we're not stuck any more with boost 1.38!!!
    string res;
    string match;
    bool matching = false;
    for (char c : ce) {
        if (!matching) {
            res += c;
            if (c == '$') // matching starts
                matching = true;
        } else {
            if (c == ' ' || c == ')' || c == '\n') { //matching ends
                auto found_it = std::find(labels.begin(), labels.end(), match);
                OC_ASSERT(found_it != labels.end(), "No label %s matching",
                          match.c_str());
                arity_t idx = distance(labels.begin(), found_it) + 1;
                res += lexical_cast<string>(idx) + c;
                match.clear();
                matching = false;
            } else // matching goes
                match += c;
        }
    }
    // If there's a match, flush the result
    if (matching) {
        auto found_it = std::find(labels.begin(), labels.end(), match);
        OC_ASSERT(found_it != labels.end(), "No label %s matching",
                  match.c_str());
        arity_t idx = distance(labels.begin(), found_it) + 1;
        res += lexical_cast<string>(idx);
    }
    return res;
}

vector<string> parse_combo_variables(const string& ce)
{
    /// @todo the implementation could be done in 1 lines with
    /// boost.regex but I don't have Internet right now and I don't
    /// know how to access boost doc!!!
    /// :-(
    vector<string> res;
    string match;
    bool matching = false;
    for (char c : ce) {
        if(!matching) {
            if(c == '$') // matching starts
                matching = true;
        } else {
            if(c == ' ' || c == ')' || c == '\n') { //matching ends
                res.push_back(match);
                match.clear();
                matching = false;
            } else // matching goes
                match += c;
        }
    }
    // if a matching is going on, add it to the result
    if(matching)
        res.push_back(match);

    return res;    
}

ostream& operator<<(ostream& out, const ann_type& h)
{
    switch (h.id) {
    case id::ann:
        return out << "ann";
    case id::ann_input:
        return out << "$I" << h.idx;
    case id::ann_node:
        return out << "$N" << h.idx;
    default:
        return out << "ANN : UNKNOWN_HANDLE";
   }
}

ostream& operator<<(ostream& out, const builtin& h)
{
    return ostream_builtin(out, h, fmt::combo);
}

ostream& operator<<(ostream& out, const wild_card& w)
{
    switch (w) {
    case id::asterisk:
        return out << "_*_";
    default:
        return out << "WILD_CARD: UNKNOWN_HANDLE";
    }
}

ostream& ostream_abbreviate_literal(ostream& out, const argument& a) {
    if(a.is_negated()) {
        return out << "!$" << -a.idx;
    }
    return out << "$" << a.idx;
}

ostream& operator<<(ostream& out, const argument& a)
{
    return ostream_argument(out, a, fmt::combo);
}

ostream& operator<<(ostream& out, const vertex& v)
{
    return ostream_vertex(out, v, fmt::combo);
}

}} // ~namespaces combo opencog
