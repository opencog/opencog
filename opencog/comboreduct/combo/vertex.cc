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
