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
#include "procedure_call.h"
#include <opencog/util/algorithm.h>
#include "iostream_combo.h"

namespace opencog { namespace combo {

bool operator<(const combo_tree& lt, const combo_tree& rt) {
    return size_tree_order<vertex>()(lt, rt);
}
        
bool is_procedure_call(const vertex& v)
{
    return (boost::get<procedure_call>(&v));
}

// Return cont&, avoid running the copy constructor!
const procedure_call& get_procedure_call(const vertex& v)
{
    return (boost::get<procedure_call>(v));
}

bool is_action_symbol(const vertex& v)
{
    return (boost::get<action_symbol>(&v));
}

// Return cont&, avoid running the copy constructor!
const action_symbol& get_action_symbol(const vertex& v)
{
    return (boost::get<action_symbol>(v));
}

bool is_indefinite_object(const vertex& v)
{
    return (boost::get<indefinite_object>(&v));
}

// Return cont&, avoid running the copy constructor!
const indefinite_object& get_indefinite_object(const vertex& v)
{
    return (boost::get<indefinite_object>(v));
}

bool is_message(const vertex& v)
{
    return (boost::get<message>(&v));
}

// Return cont&, avoid running the copy constructor!
const message& get_message(const vertex& v)
{
    return (boost::get<message>(v));
}

bool is_enum_type(const vertex& v)
{
    return (boost::get<enum_t>(&v));
}

// Return cont&, avoid running the copy constructor!
const enum_t& get_enum_type(const vertex& v)
{
    return (boost::get<enum_t>(v));
}

bool is_builtin(const vertex& v)
{
    return (boost::get<builtin>(&v));
}

builtin get_builtin(const vertex& v)
{
    return (boost::get<builtin>(v));
}

bool is_wild_card(const vertex& v)
{
    return (boost::get<wild_card>(&v));
}

wild_card get_wild_card(const vertex& v)
{
    return (boost::get<wild_card>(v));
}

bool is_contin(const vertex& v)
{
    return (boost::get<contin_t>(&v));
}
contin_t get_contin(const vertex& v)
{
    return (boost::get<contin_t>(v));
}

/**
 * return true if the vertex is an argument. Note, however, that
 * this does not take into account the type of the argument; thus,
 * any code that uses this blindly runs the risk of accepting arguments
 * of the wrong type (e.g. boolean instead of contin, or v.v.).
 *
 * This should be fixed, but we don't have the infrastructure for this.
 */
bool is_argument(const vertex& v)
{
    return (boost::get<argument>(&v));
}
argument& get_argument(vertex& v)
{
    return boost::get<argument>(v);
}
bool is_ann_type(const vertex& v)
{
    return (boost::get<ann_type>(&v));
}
ann_type& get_ann_type(vertex& v)
{
    return (boost::get<ann_type>(v));
}
const argument& get_argument(const vertex& v)
{
    return boost::get<argument>(v);
}
bool is_negated(vertex& v)
{
    if (argument* a = boost::get<argument>(&v))
        return a->is_negated();
    return false;
}

bool is_action(const vertex& v)
{
    return (boost::get<action>(&v));
}

action get_action(const vertex& v)
{
    return (boost::get<action>(v));
}

bool is_builtin_action(const vertex& v)
{
    return (boost::get<builtin_action>(&v));
}
builtin_action get_builtin_action(const vertex& v)
{
    return (boost::get<builtin_action>(v));
}
bool is_action_result(const vertex& v)
{
    return (v == id::action_failure || v == id::action_success);
}

bool is_perception(const vertex& v)
{
    return (boost::get<perception>(&v));
}

perception get_perception(const vertex& v)
{
    return (boost::get<perception>(v));
}

bool is_definite_object(const vertex& v)
{
    return (boost::get<definite_object>(&v));
}

definite_object get_definite_object(const vertex& v)
{
    return (boost::get<definite_object>(v));
}

contin_t cast_contin(const vertex& v)
{
    if (is_boolean(v))
        return (contin_t)vertex_to_bool(v);
    else if (is_contin(v))
        return get_contin(v);
    else {
        OC_ASSERT(false, "can't do much more");
        return 0;
    }
}
        
bool operator==(const vertex& v, procedure_call h)
{
    if (const procedure_call* vh = boost::get<procedure_call>(&v))
        return (*vh == h);
    return false;
}
// bool operator==(procedure_call h, const vertex& v)
// {
//     return (v == h);
// }
bool operator!=(const vertex& v, procedure_call h)
{
    return !(v == h);
}
bool operator!=(procedure_call h, const vertex& v)
{
    return !(v == h);
}

vertex negate_vertex(const vertex& v)
{
    return negate_builtin(get_builtin(v));
}
builtin negate_builtin(builtin b) {
    if (b == id::logical_true)
        return id::logical_false;
    else if (b == id::logical_false)
        return id::logical_true;
    else {
        std::stringstream ss;
        ss << b;
        OC_ASSERT(false,
                  "builtin %s should be id::logical_true or id::logical_false",
                  ss.str().c_str());
        return builtin();
    }
}

vertex swap_and_or(const vertex& v)
{
    OC_ASSERT(v == id::logical_or || v == id::logical_and);
    return v == id::logical_and ? id::logical_or : id::logical_and;
}
        
void copy_without_null_vertices(combo_tree::iterator src,
                                combo_tree& dst_tr, combo_tree::iterator dst)
{
    *dst = *src;
    for (combo_tree::sibling_iterator sib = src.begin();sib != src.end();++sib)
        if (*sib != id::null_vertex)
            copy_without_null_vertices(sib, dst_tr, dst_tr.append_child(dst));
}

}} // ~namespaces combo opencog
