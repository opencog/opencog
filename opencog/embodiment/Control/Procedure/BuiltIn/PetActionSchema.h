/*
 * opencog/embodiment/Control/Procedure/BuiltIn/PetActionSchema.h
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

#ifndef PET_ATION_SCHEMA_H_
#define PET_ATION_SCHEMA_H_
/**
 * PetActionSchema.h:
 *
 * This is a class for all builtin schema that executes a single Pet action
 * The execute() method always returns the ID of the ActionPlan sent to SL/Proxy so
 * that Procedure Interpreter can check if the action plan has really finished or failed.
 *
 * @author Welter Luigi
 */

#include <opencog/embodiment/Control/Procedure/BuiltInProcedure.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>

#include <exception>

namespace Procedure
{

class PetActionSchema : public BuiltInProcedure
{

    std::string name;

protected:

    opencog::pai::PAI& pai;
    const opencog::pai::ActionType& actionType;

public:

    PetActionSchema(opencog::pai::PAI& pai, const opencog::pai::ActionType& actionType);
    virtual ~PetActionSchema();

    const std::string& getName() const;
    bool isPetAction() const;
    combo::vertex execute(const std::vector<combo::vertex>& arguments) const throw (opencog::RuntimeException, opencog::InvalidParamException, std::bad_exception);
};

}

#endif /*PET_ATION_SCHEMA_H_*/
