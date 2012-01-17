/*
 * opencog/embodiment/Control/Procedure/ComboSelectProcedure.cc
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

#include "ComboSelectProcedure.h"

namespace opencog { namespace Procedure {

ComboSelectProcedure::ComboSelectProcedure()
{
    this->name = "";
    this->firstScriptName = "";
    this->secondScriptName = "";
}

//ComboSelectProcedure::ComboSelectProcedure(const std::string& _n, const std::string& _f, const std::string& _s) :
//                                           name(_n), firstScriptName(_f), secondScriptName(_s){
//}

ComboSelectProcedure::ComboSelectProcedure(const std::string& _n,
        const ComboProcedure& _f,
        const ComboProcedure& _s)
        : name(_n), firstScript(_f), secondScript(_s)
{

    this->firstScriptName = firstScript.getName();
    this->secondScriptName = secondScript.getName();
}

ComboSelectProcedure::~ComboSelectProcedure()
{
}


ProcedureType ComboSelectProcedure::getType() const
{
    return Procedure::COMBO_SELECT;
}

unsigned int ComboSelectProcedure::getArity() const
{
    // for now only ComboSelectProcedure without parameters
    return 0;
}

const std::string& ComboSelectProcedure::getName() const
{
    return this->name;
}

const ComboProcedure& ComboSelectProcedure::getFirstScript() const
{
    return this->firstScript;
}

const ComboProcedure& ComboSelectProcedure::getSecondScript() const
{
    return this->secondScript;
}

const std::string& ComboSelectProcedure::getFirstScriptName() const
{
    return this->firstScriptName;
}

const std::string& ComboSelectProcedure::getSecondScriptName() const
{
    return this->secondScriptName;
}

}} // ~namespace opencog::Procedure
