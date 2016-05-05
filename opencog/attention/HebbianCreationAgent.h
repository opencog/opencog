/*
 * opencog/attention/HebbianCreationAgent.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * Written by Joel Pitt <joel@fruitionnz.com>
 * All Rights Reserved
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

#ifndef _OPENCOG_HEBBIAN_CREATION_AGENT_H
#define _OPENCOG_HEBBIAN_CREATION_AGENT_H

#include <string>
#include <opencog/util/Logger.h>

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/AttentionValue.h>
#include <opencog/cogserver/server/Agent.h>

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

extern concurrent_queue<Handle> newAtomsInAV;

class CogServer;

class HebbianCreationAgent : public Agent
{

protected:
    AtomSpace* a;

    void setLogger(Logger* l);
    Logger *log; //!< Logger object for Agent

    void addHebbian(Handle atom,Handle source);
    float targetConjunction(Handle handle1,Handle handle2);

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::HebbianCreationAgent");
        return _ci;
    }

    HebbianCreationAgent(CogServer&);
    virtual ~HebbianCreationAgent();
    virtual void run();

    /** Return the agent's logger object
     *
     * @return A logger object.
     */
    Logger* getLogger();
}; // class

typedef std::shared_ptr<HebbianCreationAgent> HebbianCreationAgentPtr;

/** @}*/
} // namespace

#endif // _OPENCOG_HEBBIAN_LEARNING_AGENT_H
