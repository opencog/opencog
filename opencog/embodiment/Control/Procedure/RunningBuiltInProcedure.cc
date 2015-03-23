/*
 * opencog/embodiment/Control/Procedure/RunningBuiltInProcedure.cc
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

#include "RunningBuiltInProcedure.h"

namespace opencog { namespace Procedure {

using namespace pai;

RunningBuiltInProcedure::RunningBuiltInProcedure(const PAI& _pai, const BuiltInProcedure& _p, const std::vector<combo::vertex>& _arguments) : pai(_pai), p(_p), arguments(_arguments)
{
    finished = false;
    failed = false;
    result = combo::id::null_vertex; // TODO: perhaps there is a "undefined result" constant or something like that...
}
RunningBuiltInProcedure::~RunningBuiltInProcedure() {}

void RunningBuiltInProcedure::run()
{
    if (finished) return; // must run only once.
    try {
        result = p.execute(arguments);
    } catch (...) {
        failed = true;
    }
    finished = true;
}

bool RunningBuiltInProcedure::isFinished() const
{
    if (!finished) return false;
    if (!p.isAvatarAction()) return true;
    const ActionPlanID* planId = boost::get<ActionPlanID>(&result);
    if (!planId) return true;
    return pai.isPlanFinished(*planId);
}

bool RunningBuiltInProcedure::isFailed() const
{
    if (failed) return true;
    if (p.isAvatarAction() && finished) {
        const ActionPlanID* planId = boost::get<ActionPlanID>(&result);
        if (!planId) return true;
        return pai.hasPlanFailed(*planId);
    } else {
        return false;
    }
}

combo::vertex RunningBuiltInProcedure::getResult() const
{
    return result;
}

}} // ~namespace opencog::Procedure
