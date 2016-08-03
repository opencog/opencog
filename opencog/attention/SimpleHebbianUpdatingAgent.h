/*
 * SimpleHebbianUpdatingAgent.h
 *
 * Copyright (C) 2015 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Misgana Bayeta <misgana.bayetta@gmail.com>
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

#ifndef SIMPLEHEBBIANPDATINGAGENT_H_
#define SIMPLEHEBBIANPDATINGAGENT_H_

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/cogserver/server/CogServer.h>
#include "HebbianUpdatingAgent.h"

namespace opencog
{

/**
 * A less complex implementation of HebbianUpdatingAgent which basically obsoletes
 * the need of creating inverse hebbian links.The major change being the way sti is
 * normalized.
 */
class SimpleHebbianUpdatingAgent: public virtual opencog::HebbianUpdatingAgent {
public:
    virtual const ClassInfo& classinfo() const
    {
        return info();
    }
    static const ClassInfo& info()
    {
        static const ClassInfo _ci("opencog::SimpleHebbianUpdatingAgent");
        return _ci;
    }
    SimpleHebbianUpdatingAgent(CogServer&);
    virtual ~SimpleHebbianUpdatingAgent();
    void hebbianUpdatingUpdate();
    double targetConjunction(HandleSeq);
};

typedef std::shared_ptr<SimpleHebbianUpdatingAgent> SimpleHebbianUpdatingAgentPtr;
}
#endif /* SIMPLEHEBBIANPDATINGAGENT_H_*/
