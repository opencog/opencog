/*
 * opencog/attention/HebbianUpdatingAgent.h
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

#ifndef _OPENCOG_HEBBIAN_UPDATING_AGENT_H
#define _OPENCOG_HEBBIAN_UPDATING_AGENT_H

#include <string>
#include <iostream>
#include <sstream>

#include <opencog/util/Logger.h>
#include <opencog/util/RandGen.h>
#include <opencog/util/recent_val.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/AttentionValue.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/Agent.h>

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

class CogServer;

/**
 * This Agent randomly picks an Atom and updates all the outgoing HebbianLinks
 *
 * This Agents is supposed to run in it's own Thread.
 *
 * XXX: If there are to few Links they get updated to oft -> fast
 * TODO: The exact way to calculate the new/target TV might be improved
 */
class HebbianUpdatingAgent : public Agent
{

private:

    AtomSpace* a;

    /** Set the agent's logger object
     *
     * Note, this will be deleted when this agent is.
     *
     * @param l The logger to associate with the agent.
     */
    void setLogger(Logger* l);

    Logger *log; //!< Logger object for Agent

    float targetConjunction(HandleSeq handles);
    void updateHebbianLinks(Handle source);

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::HebbianUpdatingAgent");
        return _ci;
    }

    HebbianUpdatingAgent(CogServer&);
    virtual ~HebbianUpdatingAgent();
    virtual void run();

    /** Return the agent's logger object
     *
     * @return A logger object.
     */
    Logger* getLogger();

}; // class

typedef std::shared_ptr<HebbianUpdatingAgent> HebbianUpdatingAgentPtr;

/** @}*/
}  // namespace

#endif // _OPENCOG_IMPORTANCE_UPDATING_AGENT_H
