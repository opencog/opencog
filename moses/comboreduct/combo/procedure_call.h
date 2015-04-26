/*
 * opencog/comboreduct/combo/procedure_call.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _COMBO_PROCEDURE_CALL_H
#define _COMBO_PROCEDURE_CALL_H

#include <iostream>
#include <vector>
#include <cassert>

#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

#include "common_def.h"
#include "operator_base.h"
#include "vertex.h"
#include "iostream_combo.h"
#include "../type_checker/type_tree_def.h"

namespace opencog { namespace combo {

class procedure_call_base : public operator_base
{
protected:
    // name and type
    std::string _name;
    type_tree _type_tree;
    arity_t _arity;
    type_tree _output_type;
    type_tree_seq _arg_types;

    // procedure body
    combo_tree _body;

public:
    procedure_call_base(const std::string& name,
                        arity_t arity,
                        const combo_tree& tr,
                        bool infer_type = false);
    virtual ~procedure_call_base();

    // get_name
    const std::string& get_name() const;

    // type_tree
    const type_tree& get_type_tree() const;
    void set_type_tree(const type_tree& tt);

    // Helper methods for fast access type properties
    // number of arguments that takes the operator
    arity_t arity() const;
    // Return the type node of the operator
    type_tree get_output_type_tree() const;
    // Return the type tree of the input argument of index i
    // it is assumed for now that there is no arg_list
    const type_tree& get_input_type_tree(arity_t i) const;

    // Get the procedure body
    const combo_tree& get_body() const;
    combo_tree& get_mutable_body();

    // Output the procedure_call
    // if complete is false then it returns only the name
    // otherwise name(arity) := body
    std::ostream& toStream(std::ostream& out, bool complete = false) const;
};


        
//typedef const procedure_call_base* procedure_call;
//typedef std::set<procedure_call> procedure_call_set;
//typedef procedure_call_set::iterator procedure_call_set_it;
//typedef procedure_call_set::const_iterator procedure_call_const_it;

bool operator==(const procedure_call_base& pc1,
                const procedure_call_base& pc2);
bool operator!=(const procedure_call_base& pc1,
                const procedure_call_base& pc2);

// Allocate a procedure_call given an input stream of its
// definition. If loading failes then return NULL pointer. If
// infer_type is true then when the procedure_call is create a type
// checking is perform, this is not suitable when loading mutually
// recursive procedures
template < class BUILTIN_ACTION,
           class PERCEPTION,
           class ACTION_SYMBOL,
           class INDEFINITE_OBJECT >
procedure_call load_procedure_call(std::istream& in,
                                   bool infer_type = false)
{
    using namespace std;
    string str, tmp;
    int nparen = 0;

    // place name(arity) in str
    do {
        in >> tmp;
        nparen += count(tmp.begin(), tmp.end(), '(')
            - count(tmp.begin(), tmp.end(), ')');
        str += tmp + ' ';
        tmp.assign("");
    } while (in.good() && nparen > 0);
    if (nparen != 0) {
        logger().error("procedure_call - Mismatched parenthesis in the arity definition procedure '%s'",
                       str.c_str());
        return NULL;
    }
    if (!in.good()) return NULL;

    // recognize :=
    in >> tmp;
    if (tmp != ":=" || !in.good()) {
        logger().error("procedure_call - Wrong procedure definition operator '%s' in procedure definition '%s' should be ':=' instead",
                       tmp.c_str(), str.c_str());
        return NULL;
    }

    // Check that there is no parenthese mismatch
    // WARNING : in that process multiple space are removed which means that
    // a message with "yo  man" will be tranformed in "yo man"
    string body;
    do {
        in >> tmp;
        nparen += count(tmp.begin(), tmp.end(), '(')
            - count(tmp.begin(), tmp.end(), ')');
        body += tmp + ' ';
        tmp.assign("");
    } while (in.good() && nparen > 0);
    if (nparen != 0) {
        logger().error("procedure_call - Mismatched parenthesis in the body of procedure '%s'. The total of parenthesis, with '(' counting for 1 and ')' counting for -1, sums up to %d",
                       str.c_str(), nparen);
        return NULL;
    }

    //affect arity and name
    string::size_type lparen = str.find('('), rparen = str.find(')');
    if (lparen == string::npos || rparen == string::npos || lparen > rparen) {
        return NULL;
    }

    string name = str.substr(0, lparen);
    int arity;
    string arity_str = str.substr(lparen + 1, rparen - lparen - 1);
    try {
        arity = boost::lexical_cast<int>(arity_str);
    } catch (...) {
        logger().error("procedure_call - Lexical error: '%s'"
                       " supposed to be an arity in procedure"
                       " definition '%s' does not correspond to"
                       " a number",
                       arity_str.c_str(), str.c_str());
        return NULL;
    }

    //place the body procedure in tr
    combo_tree tr;
    stringstream ss(body);
    stream_to_combo_tree< BUILTIN_ACTION, PERCEPTION,
                          ACTION_SYMBOL, INDEFINITE_OBJECT > (ss, tr);
    for(combo_tree::iterator it = tr.begin(); it != tr.end(); ++it) {
        if(is_argument(*it)) {
            const argument& arg = get_argument(*it);
            if(!arg.is_idx_valid(arity)) {
                stringstream arg_ss;
                arg_ss << arg;
                logger().error("procedure_call - Semantic error:"
                               " the procedure '%s' has arity '%d'"
                               " but contains variable argument '%s'"
                               " out of range",
                               str.c_str(), arity, arg_ss.str().c_str());
                return NULL;
            }
        }
    }
    return new procedure_call_base(name, arity, tr, infer_type);
}

std::ostream& operator<<(std::ostream&, const procedure_call_base&);
std::ostream& operator<<(std::ostream&, procedure_call);

} // ~namespace combo
} // ~namespace opencog

#endif
