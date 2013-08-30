/*
 * opencog/embodiment/Control/OperationalAvatarController/ProcedureInterpreterAgent.cc
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


#include "OAC.h"
#include "ProcedureInterpreterAgent.h"

using namespace opencog::oac;

ProcedureInterpreterAgent::~ProcedureInterpreterAgent()
{
}

ProcedureInterpreterAgent::ProcedureInterpreterAgent(CogServer& cs) : Agent(cs)
{
}

void ProcedureInterpreterAgent::setInterpreter(Procedure::ProcedureInterpreter* _interpreter)
{
    interpreter = _interpreter;
}

void ProcedureInterpreterAgent::run()
{
    logger().fine("ProcedureInterpreterAgent::run()");
    interpreter->run(&(dynamic_cast<OAC*>(&_cogserver)->getNetworkElement()));
}
