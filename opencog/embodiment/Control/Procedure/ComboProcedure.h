/*
 * opencog/embodiment/Control/Procedure/ComboProcedure.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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

#ifndef COMBOPROCEDURE_H_
#define COMBOPROCEDURE_H_
/**
 * Base class for Combo Procedures
 * @author Welter Luigi
 *
 */
#include "GeneralProcedure.h"
#include <opencog/comboreduct/combo/procedure_call.h>

#include <iostream>
#include <exception>

namespace opencog { namespace Procedure {

class ComboProcedure : public GeneralProcedure, public combo::procedure_call_base
{

public:
    ComboProcedure(); // just to be used in internal maps of ComboProcedureRepository
    ComboProcedure(const procedure_call_base& pc); //copy a procedure_call into a ComboProcedure
    ComboProcedure(const std::string& name, unsigned int arity, const combo::combo_tree&, bool infer_type = false);

    virtual ~ComboProcedure();

    ProcedureType getType() const;
    const std::string& getName() const;
    const combo::combo_tree& getComboTree() const;
    unsigned int getArity() const {
        return arity();
    }

//  combo::combo_tree& getComboTree() { return _body; }
};

std::istream& operator>>(std::istream&, Procedure::ComboProcedure&)
     throw (ComboException, std::bad_exception);
std::ostream& operator<<(std::ostream&, const Procedure::ComboProcedure&);

}} // ~namespace opencog::Procedure

#endif /* COMBOPROCEDURE_H_ */
