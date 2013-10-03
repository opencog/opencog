/*
 * opencog/dynamics/attention/STIDecayingAgent.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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



#ifndef _OPENCOG_STI_DECAYING_AGENT_H
#define _OPENCOG_STI_DECAYING_AGENT_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/Agent.h>
#include <opencog/util/Logger.h>

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

class CogServer;

/**
 * This agent implements a simple forgetting mechanism that doesn't rely on the
 * complexities of the full Economic attention allocation system.
 *
 * It's primary use is as part of the Embodiment system when we're not
 * concerned with having to tune attention allocation.
 */
class STIDecayingAgent : public Agent
{

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::STIDecayingAgent");
        return _ci;
    }

    STIDecayingAgent(CogServer&);
    virtual ~STIDecayingAgent();
    virtual void run();

}; // class

typedef std::shared_ptr<STIDecayingAgent> STIDecayingAgentPtr;

/** @}*/
} // namespace

#endif // _OPENCOG_STI_DECAYING_AGENT_H
