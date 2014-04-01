/*
 * opencog/embodiment/Control/Procedure/ComboProcedure.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#include "ComboProcedure.h"
#include <opencog/util/foreach.h>
#include <opencog/util/exceptions.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

#include <sstream>
#include <boost/lexical_cast.hpp>

namespace opencog { namespace Procedure {

using namespace AvatarCombo;

static combo_tree empty_procedure;

ComboProcedure::ComboProcedure() : procedure_call_base("EMPTY_PROCEDURE",
                0,
                empty_procedure) { }

ComboProcedure::ComboProcedure(const procedure_call_base& pc) : procedure_call_base(pc) { }

ComboProcedure::ComboProcedure(const std::string& name, unsigned int arity, const combo::combo_tree& tree, bool infer_type) : procedure_call_base(name, arity_t(arity), tree, infer_type)
{
    //below is commented because it seems that arity is not used anyway
    //OC_ASSERT((int)a==arity(),
    //"The arity suggested and the arity infered are different, there must be a problem somewhere, if not and that's really what you want ask Nil to fix it");
}

ComboProcedure::~ComboProcedure() { }

ProcedureType ComboProcedure::getType() const
{
    return COMBO;
}


const std::string& ComboProcedure::getName() const
{
    return get_name();
}

const combo::combo_tree& ComboProcedure::getComboTree() const
{
    return get_body();
}

std::istream& operator>>(std::istream& in, Procedure::ComboProcedure& proc)
    throw (ComboException, std::bad_exception)
{
    using namespace std;
    using namespace combo;
    string str, tmp;
    int nparen = 0;

    do {
        in >> tmp;
        nparen += count(tmp.begin(), tmp.end(), '(') - count(tmp.begin(), tmp.end(), ')');
        str += tmp + ' ';
        tmp.assign("");
    } while (in.good() && nparen > 0);
    if (nparen != 0) {
        std::stringstream stream (std::stringstream::out);
        stream << "Paren mismatch parsing proc dec: '" << str << "'" << endl;
        throw ComboException("ComboProcedure - %s.", stream.str().c_str());
    }
    if (!in.good()) {
        proc = ComboProcedure("", 0, combo_tree());
        return in;
    }
    in >> tmp;
    if (tmp != ":=") {
        std::stringstream stream (std::stringstream::out);
        stream << "Expecting ':=', got '" << tmp << "'" << endl;
        throw ComboException("ComboProcedure - %s.", stream.str().c_str());
    }
    if (!in.good()) {
        std::stringstream stream (std::stringstream::out);
        stream << "Missing proc body in parsing '" << str << "'" << endl;
        throw ComboException("ComboProcedure - %s.", stream.str().c_str());
    }

    string body;
    do {
        in >> tmp;
        nparen += count(tmp.begin(), tmp.end(), '(') - count(tmp.begin(), tmp.end(), ')');
        body += tmp + ' ';
        tmp.assign("");
    } while (in.good() && nparen > 0);
    if (nparen != 0) {
        std::stringstream stream (std::stringstream::out);
        stream << "Paren mismatch parsing proc body: '" << body << "'" << endl;
        throw ComboException("ComboProcedure - %s.", stream.str().c_str());
    }

    string::size_type lparen = str.find('('), rparen = str.find(')');
    if (lparen == string::npos || rparen == string::npos || lparen > rparen) {
        std::stringstream stream (std::stringstream::out);
        stream << "Malformed combo function declaration '"
        << str << "'" << endl;
        throw ComboException("ComboProcedure - %s.", stream.str().c_str());
    }

    string name = str.substr(0, lparen);
    unsigned int arity;
    try {
        arity = boost::lexical_cast<unsigned int>(str.substr(lparen + 1, rparen - lparen - 1));
    } catch (...) {
        std::stringstream stream (std::stringstream::out);
        stream << "Couldn't parse function arity" << endl;
        throw ComboException("ComboProcedure - %s.", stream.str().c_str());
    }

    combo_tree tr;
    stringstream ss(body);
    // ss >> tr;
    AvatarCombo::operator>>(ss, tr);

    for (combo_tree::iterator it = tr.begin();it != tr.end();++it) {
        if (is_argument(*it)) {
            int arg_index =  get_argument(*it).idx;
            if (arg_index < 1 || arg_index > (int)arity) {
                throw ComboException(TRACE_INFO,
                                     "Argument index '%d' should be >= 1 and <= arity '%d'.", arg_index, arity);
            }
        }
    }

    proc = ComboProcedure(name, arity, tr);

    return in;
}

std::ostream& operator<<(std::ostream& out, const Procedure::ComboProcedure& proc)
{
    return (out << proc.getName() << "(" << proc.getArity() << ") := " << proc.getComboTree());
}

}} // ~namespace opencog::Procedure
