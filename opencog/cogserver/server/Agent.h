/*
 * opencog/cogserver/server/Agent.h
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

#ifndef _OPENCOG_AGENT_H
#define _OPENCOG_AGENT_H

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/cogserver/server/Factory.h>

#if __GNUC__ >= 5
#define SHARED_PTR_ATOMIC_OPS
#endif

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

typedef short stim_t;
typedef std::unordered_map<Handle, stim_t, handle_hash> AtomStimHashMap;

class CogServer;

/** The MindAgent Class
 * This class defines the base abstract class that should be extended by all
 * opencog agents.
 *
 * Typically, an opencog agent is coded similarly to a Java thread (except that
 * there's currently no concurrency between multiple agents): it has a "run"
 * method which is called by the server everytime the agent is selected for
 * execution. So, to write a custom agent, all one has to do is to derive from
 * the Agent class and implement the desired behavior inside the 'run' method.
 *
 * The Agent base class also provides the 'frequency' attribute which
 * defines how often the agent will be executed. A value of 1 (the default)
 * means that the agent will be executed every server cycle. A value of 2 means
 * that the agent will be executed every 2 cycles. And so on.
 *
 * Since agents are registered with the cogserver using the Registry+Factory
 * pattern, agent classes must override the 'classinfo' which uniquely
 * identifies its class. Typically the 'classinfo' method will simply forward to
 * a call to an 'info' class method that provides the actual class info -- the
 * 'info' class method is required by the Registry+Factory anyway.
 *
 * A typical derived Agent declaration & initialization would thus look as
 * follows:
 *
 * \code
 * // MyAgent.h
 * #include <opencog/cogserver/server/Agent.h>
 * #include <opencog/cogserver/server/Factory.h>
 * class CogServer;
 * class MyAgent : public opencog::Agent {
 *     virtual const ClassInfo& classinfo() const { return info(); }
 *     static const ClassInfo& info() {
 *         static const ClassInfo _ci("MyAgent");
 *         return _ci;
 *     }
 *
 *     void run(CogServer* server) {
 *         // implement the agent's behavior
 *         ...
 *     }
 * }
 *
 * // application/module code
 * #include "MyAgent.h"
 * #include <opencog/cogserver/server/Agent.h>
 * #include <opencog/cogserver/server/CogServer.h>
 * ...
 * Factory<MyAgent, Agent> factory;
 * CogServer& cogserver = static_cast<CogServer&>(server());
 * cogserver.registerAgent(MyAgent::info().id, &factory);
 * cogserver.createAgent(MyAgent::info().id, true);
 * \endcode
 *
 * @todo create a run wrapper around the actual implementation of the run method.
 * This would flip an "active" or "running" boolean property on or off. And
 * carry out other admin tasks before and after each cycle, without subclasses
 * needing to carry these out themselves unless they had specific reason to.
 */
class Agent
{

protected:
    /**
     * Set the agent's logger object.
     * NOTE: this will be deleted when this agent is.
     * XXX FIXME This is a terrible API design! One must never use
     * setters like this!  Let the agent own the logger, and if need be,
     * it could be queried (e.g. to alter the debug level).
     *
     * @param l The logger to associate with the agent.
     */
    void setLogger(Logger*);
    Logger* _log;

    CogServer& _cogserver;
    AtomSpace* _as;

    /**
     * The agent's frequency. Determines how often the opencog server should
     * schedule this agent. A value of 1 (the default) means that the agent
     * will be executed every server cycle; a value of 2 means that the agent
     * will be executed every 2 cycles; and so on.
     */
    int _frequency;

    /**
     * Sets the list of parameters for this agent and their default values.
     * If any parameter values are unspecified in the Config singleton, sets
     * them to the default values.
     * Note: it is assumed to be called only during initialization, so it is not
     * thread safe and shouldn't be called from multiple threads
     */
    void setParameters(const std::vector<std::string>&);
    std::vector<std::string> _parameters;

    /**
     * The atoms utilized by the Agent in a single cycle, to be used by the
     * System Activity Table to assign credit to this agent.
     */
    std::vector<UnorderedHandleSet> _utilizedHandleSets;
    mutable std::mutex _handleSetMutex;

    /** called by AtomTable via a boost::signals2::signal when an atom is removed. */
    void atomRemoved(const AtomPtr&);

public:

    /**
     * Return the agent's logger object
     *
     * @return A logger object.
     */
    Logger* getLogger();

    /** Agent's constructor. By default, initializes the frequency to 1. */
    Agent(CogServer&, const unsigned int f = 1);

    /** Agent's destructor */
    virtual ~Agent();

    /**
     * Abstract run method. Should be overridden by a derived agent with the
     * actual agent's behavior.
     */
    virtual void run() = 0;

    /** Agent stop() method, called when the agent is stopped */
    virtual void stop() {}

    /** Returns the agent's frequency. */
    virtual int frequency(void) const { return _frequency; }

    /** Sets the agent's frequency. */
    virtual void setFrequency(int freq) { _frequency = freq; }

    /** Returns the agent's class info. */
    virtual const ClassInfo& classinfo() const = 0;

    /**
     * Dumps the agent's name and all its configuration parameters
     * to a string.
     */
    std::string to_string() const;

    /**
     * Returns the sequence of handle sets for this cycle that the agent
     * would like to claim credit for in the System Activity Table.
     */
    virtual std::vector<UnorderedHandleSet> getUtilizedHandleSets() const
    {
        std::lock_guard<std::mutex> lock(_handleSetMutex);
        return _utilizedHandleSets;
    }

    /** Resets the utilized handle sets */
    void resetUtilizedHandleSets();

}; // class

typedef std::shared_ptr<Agent> AgentPtr;

/** @}*/
}  // namespace

#endif // _OPENCOG_AGENT_H
