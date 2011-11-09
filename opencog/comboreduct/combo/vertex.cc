/*
 * opencog/comboreduct/combo/vertex.cc
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
#include "vertex.h"
#include <boost/lexical_cast.hpp>
#include "procedure_call.h"
#include <opencog/util/algorithm.h>

namespace opencog { namespace combo {

using namespace std;
using namespace boost;

// uncomment this to output a negative literal !#n instead of not(#n)
#define ABBREVIATE_NEGATIVE_LITERAL

void copy_without_null_vertices(combo_tree::iterator src,
                                combo_tree& dst_tr, combo_tree::iterator dst)
{
    *dst = *src;
    for (combo_tree::sibling_iterator sib = src.begin();sib != src.end();++sib)
        if (*sib != id::null_vertex)
            copy_without_null_vertices(sib, dst_tr, dst_tr.append_child(dst));
}

string ph2l(const string& ce, const vector<string>& labels)
{
    /// @todo the implementation could be done in 2 lines with
    /// boost.regex with boost version 1.42 or above because then we
    /// can use Formatter as callback, but we're stuck with boost 1.38
    /// :-(
    string res;
    string match;
    bool matching = false;
    foreach(char c, ce) {
        if(!matching) {
            res += c;
            if(c == '#') // matching starts
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
    if(matching)
        res += labels[lexical_cast<arity_t>(match) - 1];
    return res;
}

string l2ph(const string& ce, const vector<string>& labels)
{
    /// @todo the implementation could be done in 2 lines with
    /// boost.regex with boost version 1.42 or above because then we
    /// can use Formatter as callback, but we're stuck with boost 1.38
    /// :-(
    string res;
    string match;
    bool matching = false;
    foreach(char c, ce) {
        if(!matching) {
            res += c;
            if(c == '#') // matching starts
                matching = true;
        } else {
            if(c == ' ' || c == ')' || c == '\n') { //matching ends
                auto found_it = find(labels, match);
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
    // if a matching is going on flush to the result
    if(matching) {
        auto found_it = find(labels, match);
        OC_ASSERT(found_it != labels.end(), "No label %s matching",
                  match.c_str());
        arity_t idx = distance(labels.begin(), found_it) + 1;
        res += lexical_cast<string>(idx);
    }
    return res;
}

ostream& operator<<(ostream& out, const opencog::combo::ann_type& h)
{
    using namespace opencog::combo;
    switch (h.id) {
    case id::ann:
        return out << "ann";
    case id::ann_input:
        return out << "#I" << h.idx;
    case id::ann_node:
        return out << "#N" << h.idx;
    default:
        return out << "ANN : UNKNOWN_HANDLE";
   }
}

ostream& operator<<(ostream& out, const opencog::combo::builtin& h)
{
    using namespace opencog::combo;
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
    case id::contin_if:
        return out << "contin_if";
    case id::boolean_if:
        return out << "boolean_if";
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
    default:
        return out << "BUILTIN : UNKNOWN_HANDLE";
    }
}

ostream& operator<<(ostream& out, const opencog::combo::wild_card& w)
{
    using namespace opencog::combo;
    switch (w) {
    case id::asterisk:
        return out << "_*_";
    default:
        return out << "WILD_CARD: UNKNOWN_HANDLE";
    }
}

ostream& ostream_abbreviate_literal(ostream& out, const opencog::combo::argument& a) {
    using namespace opencog::combo;
    if(a.is_negated()) {
        return out << "!#" << -a.idx;
    }
    return out << "#" << a.idx;
}

ostream& operator<<(ostream& out, const opencog::combo::argument& a)
{
    using namespace opencog::combo;
#ifdef ABBREVIATE_NEGATIVE_LITERAL
    return ostream_abbreviate_literal(out, a);
#else
    if (a.is_negated())        
        return out << "not(#" << -a.idx << ")";
    return out << "#" << a.idx;
#endif
}

ostream& operator<<(ostream& out, const opencog::combo::vertex& v)
{
    using namespace opencog::combo;
    if (const ann_type* z = get<ann_type>(&v))
        return out << (*z);
    if (const argument* a = get<argument>(&v))
        return out << (*a);
    if (const builtin* h = get<builtin>(&v))
        return out << (*h);
    if (const wild_card* w = get<wild_card>(&v))
        return out << (*w);
    if (const action* act = get<action>(&v))
        return out << (*act);
    if (const builtin_action* aact = get<builtin_action>(&v))
        return out << (*aact);
    if (const perception* per = get<perception>(&v))
        return out << (*per);
    if (const indefinite_object*
            iot = get<indefinite_object>(&v))
        return out << (*iot);
    if (const message* m = get<message>(&v))
        return out << (*m);
    if (const definite_object* dot = get<definite_object>(&v))
        return out << (*dot);
    if (const action_symbol* as = get<action_symbol>(&v))
        return out << (*as);
    if (const procedure_call* cp = get<procedure_call>(&v)) {
        return out << (*cp);
    }
    return out << get<contin_t>(v);
}

}} // ~namespaces combo opencog
