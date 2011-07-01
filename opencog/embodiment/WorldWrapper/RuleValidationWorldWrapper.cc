/*
 * opencog/embodiment/WorldWrapper/RuleValidationWorldWrapper.cc
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

#include "RuleValidationWorldWrapper.h"
#include "WorldWrapperUtilMock.h"

using namespace opencog::world;
using namespace opencog::combo;

RuleValidationWorldWrapper::RuleValidationWorldWrapper(VirtualWorldData::VirtualWorldState& _vw) :
        virtualWorld(_vw) {}

RuleValidationWorldWrapper::~RuleValidationWorldWrapper()
{
}

bool RuleValidationWorldWrapper::isPlanFinished() const
{
    // always finished - remember, this is a MOCK
    return true;
}

bool RuleValidationWorldWrapper::isPlanFailed() const
{
    // never failed - remember, this is a MOCK
    return false;
}

bool RuleValidationWorldWrapper::sendSequential_and(sib_it from, sib_it to)
{
    // no sequential_and is sent, but the answer is always true - remember, this is a MOCK
    return true;
}

/**
 * @override
 */
vertex RuleValidationWorldWrapper::evalPerception(pre_it it, variable_unifier& vu)
{
    return WorldWrapperUtilMock::evalPerception(it, virtualWorld, vu);
}

/**
 * @override
 */
vertex RuleValidationWorldWrapper::evalIndefiniteObject(indefinite_object io, variable_unifier& vu)
{
    return WorldWrapperUtilMock::evalIndefiniteObject(io, virtualWorld, vu);
}


