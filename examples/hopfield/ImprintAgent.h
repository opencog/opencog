/*
 * examples/hopfield/ImprintAgent.h
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#ifndef _OPENCOG_IMPRINT_AGENT_H
#define _OPENCOG_IMPRINT_AGENT_H

#include <string>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/truthvalue/AttentionValue.h>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/util/Logger.h>

#include "Pattern.h"

namespace opencog
{

class CogServer;

/** Agent that imprints Patterns on to the perceptual nodes of the Hopfield
 * network.
 */
class ImprintAgent : public Agent
{
public:
    typedef std::vector< std::vector<float> > w_t;
private:
    bool verbose;

    /** Set the agent's logger object
     *
     * Note, this will be deleted when this agent is.
     *
     * @param l The logger to associate with the agent.
     */
    void setLogger(Logger* l);

    Logger *log; //!< Logger object for Agent

    Pattern epsilon; //!< Pattern for imprinting on the network

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::ImprintAgent");
        return _ci;
    }

    ImprintAgent(CogServer&);
    virtual ~ImprintAgent();
    virtual void run();

    /** Return the agent's logger object
     *
     * @return A logger object.
     */
    Logger* getLogger();

    //! Set pattern to imprint when Agent runs
    void setPattern(Pattern _epsilon);

}; // class

typedef std::shared_ptr<ImprintAgent> ImprintAgentPtr;

} // namespace

#endif // _OPENCOG_IMPRINT_AGENT_H
