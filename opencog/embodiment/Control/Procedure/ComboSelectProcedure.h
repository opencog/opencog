/*
 * opencog/embodiment/Control/Procedure/ComboSelectProcedure.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#ifndef COMBOSELECTPROCEDURE_H_
#define COMBOSELECTPROCEDURE_H_

/**
 * Base class for ComboSelect Procedures
 * @author Carlos Lopes
 *
 */
#include "GeneralProcedure.h"
#include "ComboProcedure.h"

#include <iostream>
#include <exception>

namespace opencog { namespace Procedure {

/**
 * ComboSelectProcedure has two combo scripts within it. These scripts are
 * stored into ComboProcedureRepository and ComboSelectProcedure holds only
 * their names for repository quering.
 */
class ComboSelectProcedure : public GeneralProcedure
{

public:

    ComboSelectProcedure();
    ComboSelectProcedure(const std::string& name,
                         const ComboProcedure& firstScript,
                         const ComboProcedure& secondScript);
//            ComboSelectProcedure(const std::string& name, const std::string& firstScriptName, const std::string& secondScriptName);

    virtual ~ComboSelectProcedure();

    // from GeneralProcedure
    ProcedureType getType() const;
    unsigned int getArity() const;
    const std::string& getName() const;

    // get the name of the first script
    const std::string& getFirstScriptName() const;

    // get the name of the second script
    const std::string& getSecondScriptName() const;

    // get the first combo script
    const ComboProcedure& getFirstScript() const;

    // get the second combo script
    const ComboProcedure& getSecondScript() const;

private:

    // procedure name
    std::string name;

    // names of the first and second combo scripts. These names are set
    // based on the combo scripts only
    std::string firstScriptName;
    std::string secondScriptName;

    // the ComboScripts themselves
    // The first one can contain wild-cards and when evaluated
    // they will be unified.
    // The second one only support parameters (0-parameter is also
    // allowed) and these are the variables unified by the first script.
    ComboProcedure firstScript;
    ComboProcedure secondScript;
};

}} // ~namespace opencog::Procedure

#endif /* COMBOSELECTPROCEDURE_H_ */
