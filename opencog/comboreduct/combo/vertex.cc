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

std::ostream& operator<<(std::ostream& out, const combo::builtin& h)
{
    using namespace combo;
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

std::ostream& operator<<(std::ostream& out, const combo::wild_card& w)
{
    using namespace combo;
    switch (w) {
    case id::asterisk:
        return out << "_*_";
    default:
        return out << "WILD_CARD: UNKNOWN_HANDLE";
    }
}

std::ostream& operator<<(std::ostream& out, const combo::argument& a)
{
    if (a.is_negated())
        return out << "not(#" << -a.idx << ")";
    return out << "#" << a.idx;
}

std::ostream& operator<<(std::ostream& out, const combo::vertex& v)
{
    if (const combo::argument* a = boost::get<combo::argument>(&v))
        return out << (*a);
    if (const combo::builtin* h = boost::get<combo::builtin>(&v))
        return out << (*h);
    if (const combo::wild_card* w = boost::get<combo::wild_card>(&v))
        return out << (*w);
    if (const combo::action* act = boost::get<combo::action>(&v))
        return out << (*act);
    if (const combo::builtin_action* aact = boost::get<combo::builtin_action>(&v))
        return out << (*aact);
    if (const combo::perception* per = boost::get<combo::perception>(&v))
        return out << (*per);
    if (const combo::indefinite_object*
            iot = boost::get<combo::indefinite_object>(&v))
        return out << (*iot);
    if (const combo::message* m = boost::get<combo::message>(&v))
        return out << (*m);
    if (const combo::definite_object* dot = boost::get<combo::definite_object>(&v))
        return out << (*dot);
    if (const combo::action_symbol* as = boost::get<combo::action_symbol>(&v))
        return out << (*as);
    if (const combo::procedure_call* cp = boost::get<combo::procedure_call>(&v)) {
        return out << (*cp);
    }
    return out << boost::get<combo::contin_t>(v);
}


namespace combo
{

void copy_without_null_vertices(combo_tree::iterator src,
                                combo_tree& dst_tr, combo_tree::iterator dst)
{
    *dst = *src;
    for (combo_tree::sibling_iterator sib = src.begin();sib != src.end();++sib)
        if (*sib != id::null_vertex)
            copy_without_null_vertices(sib, dst_tr, dst_tr.append_child(dst));
}

} //~namespace combo
