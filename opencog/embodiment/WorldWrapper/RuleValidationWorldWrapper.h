/*
 * opencog/embodiment/WorldWrapper/RuleValidationWorldWrapper.h
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

#ifndef _VIRUTAL_WORLD_WRAPPER_H
#define _VIRTUAL_WORLD_WRAPPER_H

#include "WorldWrapper.h"
#include <opencog/embodiment/RuleValidation/VirtualWorldData/VirtualWorldState.h>
#include <opencog/util/exceptions.h>

namespace opencog { namespace world {

class RuleValidationWorldWrapper : public WorldWrapperBase
{

public:

    RuleValidationWorldWrapper(VirtualWorldData::VirtualWorldState & virtualWorld);
    ~RuleValidationWorldWrapper();

    // override
    bool isPlanFinished() const;

    // override
    bool isPlanFailed() const;

    // override
    bool sendSequential_and(sib_it from, sib_it to);


    // override
    combo::vertex evalPerception(pre_it per,
                                 combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

    // override
    combo::vertex evalIndefiniteObject(combo::indefinite_object io,
                                       combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());

private:

    VirtualWorldData::VirtualWorldState& virtualWorld;
};
} } // namespace opencog::world

#endif
