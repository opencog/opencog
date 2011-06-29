/*
 * opencog/embodiment/Control/Procedure/RunningBuiltInProcedure.h
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

#ifndef _RUNNING_BUILTIN_PROCEDURE_H
#define _RUNNING_BUILTIN_PROCEDURE_H

#include "BuiltInProcedure.h"
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>

namespace Procedure
{

class RunningBuiltInProcedure
{

public:
    RunningBuiltInProcedure(const opencog::pai::PAI& _pai, const BuiltInProcedure& _p, const std::vector<combo::vertex>& _arguments);
    ~RunningBuiltInProcedure();

    void run();

    bool isFinished() const;
    bool isFailed() const;
    combo::vertex getResult() const;

protected:

    const opencog::pai::PAI& pai;
    const BuiltInProcedure& p;
    bool finished;
    bool failed;
    combo::vertex result;
    std::vector<combo::vertex> arguments;
};

} //~namespace Procedure

#endif
