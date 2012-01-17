/*
 * opencog/embodiment/Control/Procedure/GeneralProcedure.h
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

#ifndef PROCEDURE_H_
#define PROCEDURE_H_
/**
 * Base class for Procedures
 * @author Welter Luigi
 *
 */
#include <string>

namespace opencog { namespace Procedure {

typedef enum {BUILT_IN, COMBO, COMBO_SELECT} ProcedureType;

class GeneralProcedure
{

public:
    virtual ~GeneralProcedure() {}

    virtual const std::string& getName() const = 0;
    virtual ProcedureType getType() const = 0;
    virtual unsigned int getArity() const = 0;
};

}} // ~namespace opencog::Procedure

#endif /*PROCEDURE_H_*/
