/*
 * opencog/embodiment/Control/Procedure/RunningProcedureId.cc
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

#include "RunningProcedureId.h"

namespace opencog { namespace Procedure {

RunningProcedureId::RunningProcedureId()
{
    this->type = COMBO;
}

RunningProcedureId::RunningProcedureId(unsigned long id, ProcedureType type)
{
    this->id = id;
    this->type = type;
}

void RunningProcedureId::setType(ProcedureType type)
{
    this->type = type;
}

const ProcedureType RunningProcedureId::getType() const
{
    return this->type;
}

void RunningProcedureId::setId(unsigned long id)
{
    this->id = id;
}

const unsigned long RunningProcedureId::getId() const
{
    return this->id;
}

bool RunningProcedureId::operator== (const RunningProcedureId& rpId) const
{
    return (this->id == rpId.getId() &&
            this->type == rpId.getType());
}

bool RunningProcedureId::operator< (const RunningProcedureId& rpId) const
{
    return (this->id < rpId.getId());
}

}} // ~namespace opencog::Procedure
