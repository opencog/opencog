/*
 * opencog/comboreduct/combo/procedure_call.cc
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
#include "procedure_call.h"
#include "iostream_combo.h"
#include "../type_checker/type_tree.h"

namespace opencog { namespace combo {

procedure_call_base::procedure_call_base(const std::string& name,
                                         arity_t arity,
                                         const combo_tree& tr,
                                         bool infer_type)
    : _name(name), _type_tree(id::unknown_type), _arity(arity),
      _output_type(id::unknown_type), _body(tr)
{
    
    if (tr.empty())
        _output_type = type_tree(id::ill_formed_type);
    else if (infer_type) {
        //infer type tree
        set_type_tree(infer_type_tree(_body));
        OC_ASSERT(is_well_formed(_type_tree),
                          "Cannot instantiate a procedure with a ill formed combo tree");
    }
}

procedure_call_base::~procedure_call_base() {}

//get_name
const std::string& procedure_call_base::get_name() const
{
    return _name;
}

//type_tree
const type_tree& procedure_call_base::get_type_tree() const
{
    return _type_tree;
}
void procedure_call_base::set_type_tree(const type_tree& tt)
{

    _type_tree = tt;
    //setting arity
    _arity = type_tree_arity(_type_tree);
    //setting output type
    OC_ASSERT(!_type_tree.empty());
    type_tree_pre_it ty_it = _type_tree.begin();
    type_tree_sib_it sib = ty_it.begin();
    if (*ty_it == id::lambda_type)
        _output_type = type_tree(ty_it.last_child());
    else
        _output_type = type_tree(ty_it);
    //setting input argument type trees
    if (_arity == -1)
        _arg_types.push_back(type_tree(sib.begin()));
    else
        for (type_tree_sib_it sib = ty_it.begin();
                sib != type_tree_sib_it(_type_tree.last_child(ty_it)); ++sib)
            _arg_types.push_back(type_tree(sib));

}

//helper methods for fast access type properties
//number of arguments that takes the operator
arity_t procedure_call_base::arity() const
{
    return _arity;
}
//return the type node of the operator
type_tree procedure_call_base::get_output_type_tree() const
{
    return _output_type;
}
//return the type tree of the input argument of index i
const type_tree& procedure_call_base::get_input_type_tree(arity_t i) const
{
    return _arg_types[i];
}

const combo_tree& procedure_call_base::get_body() const
{
    return _body;
}
combo_tree& procedure_call_base::get_mutable_body()
{
    return _body;
}

std::ostream& procedure_call_base::toStream(std::ostream& out,
                                            bool complete) const
{
    out << get_name();
    if (complete)
        out << "(" << ((int)arity()) << ") := " << get_body();
    return out;
}

bool operator==(const procedure_call_base& pc1, const procedure_call_base& pc2)
{
    return (pc1.get_name() == pc2.get_name())
        && (pc1.get_body() == pc2.get_body());
}
bool operator!=(const procedure_call_base& pc1, const procedure_call_base& pc2)
{
    return !(pc1 == pc2);
}

std::ostream& operator<<(std::ostream& out, const procedure_call_base& pc)
{
    return pc.toStream(out);
}

std::ostream& operator<<(std::ostream& out, procedure_call pc)
{
    OC_ASSERT(pc);
    return pc->toStream(out);
}

} // ~namespace combo
} // ~namespace opencog

