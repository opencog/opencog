/*
 * opencog/server/Agent.h
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

#include <string>
#include <unordered_map>

#include <opencog/server/Factory.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>

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
 * pattern, agent classes must override the 'classinfo' which uniquelly
 * identifies its class. Typicall the 'classinfo' method will simply forward to
 * a call to an 'info' class method that provides the actual class info -- the
 * 'info' class method is required by the Registry+Factory anyway.
 *
 * A typical derived Agent declaration & initialization would thus look as
 * follows:
 *
 * \code
 * // MyAgent.h
 * #include <opencog/server/Agent.h>
 * #include <opencog/server/Factory.h>
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
 * #include <opencog/server/Agent.h>
 * #include <opencog/server/CogServer.h>
 * ...
 * Factory<MyAgent, Agent> factory;
 * CogServer& cogserver = static_cast<CogServer&>(server());
 * cogserver.registerAgent(MyAgent::info().id, &factory);
 * cogserver.createAgent(MyAgent::info().id, true);
 * \endcode
 *
 * @todo create a run wrapper around the actual implementation of the run method.
 * This would flip an "active" or "running" boolean property on or off. Any
 * carry out other admin tasks before and after each cycle, without subclasses
 * needing to carry these out themselves unless they had specific reason to.
 */
class Agent
{

protected:
    CogServer& _cogserver;

    AttentionValuePtr _attentionValue;

    /** The agent's frequency. Determines how often the opencog server should
     *  schedule this agent. A value of 1 (the default) means that the agent
     *  will be executed every server cycle; a value of 2 means that the agent
     *  will be executed every 2 cycles; and so on. */
    int _frequency;

    /** Sets the list of parameters for this agent and their default values.
     * If any parameter values are unspecified in the Config singleton, sets
     * them to the default values. */
    void setParameters(const std::string* params);

    const std::string* PARAMETERS;

    /** The atoms utilized by the Agent in a single cycle, to be used by the
     *  System Activity Table to assign credit to this agent. */
    std::vector<UnorderedHandleSet> _utilizedHandleSets;

    /** Total stimulus given out to atoms */
    stim_t totalStimulus;

    /** Hash table of atoms given stimulus since reset */
    AtomStimHashMap* stimulatedAtoms;

    boost::signals2::connection conn;

    /** called by AtomTable via a boost::signals2::signal when an atom is removed. */
    void atomRemoved(AtomPtr);

public:

    /** Agent's constructor. By default, initializes the frequency to 1. */
    Agent(CogServer&, const unsigned int f = 1);    

    /** Agent's destructor */
    virtual ~Agent();

    /** Abstract run method. Should be overriden by a derived agent with the
     *  actual agent's behavior. */
    virtual void run() = 0;

    /** Returns the agent's frequency. */
    virtual int frequency(void) const { return _frequency; }

    /** Sets the agent's frequency. */
    virtual void setFrequency(int frequency) { _frequency=frequency; }

    /** Returns the agent's class info. */
    virtual const ClassInfo& classinfo() const = 0;
    
    /** Dumps the agent's name and all its configuration parameters
     * to a string. */    
    std::string to_string() const;

    /** Returns the sequence of handle sets for this cycle that the agent
     *  would like to claim credit for in the System Activity Table. */
    virtual const std::vector<UnorderedHandleSet>& getUtilizedHandleSets() const
    {
        return _utilizedHandleSets;
    }

    /** Resets the utilized handle sets */
    void resetUtilizedHandleSets();

    /**
     * Stimulate a Handle's atom.
     *
     * @param h atom handle
     * @param amount of stimulus to give.
     * @return total stimulus given since last reset.
     */
    stim_t stimulateAtom(Handle h, stim_t amount);

    /**
     * Stimulate all atoms in HandleSeq evenly with a given amount of stimulus.
     *
     * @param hs set of atoms to spread stimulus across.
     * @param amount amount of stimulus to share.
     * @return remainder stimulus after equal spread between atoms.
     */
    stim_t stimulateAtom(HandleSeq hs, stim_t amount);

    /**
     * Remove stimulus from a Handle's atom.
     *
     * @param atom handle
     */
    void removeAtomStimulus(Handle h);

    /**
     * Reset stimulus.
     *
     * @return new stimulus since reset, usually zero unless another
     * thread adds more.
     */
    stim_t resetStimulus();

    /**
     * Get total stimulus.
     *
     * @return total stimulus since last reset.
     */
    stim_t getTotalStimulus() const;

    /**
     * Get stimulus for Atom.
     *
     * @param h handle of atom to get stimulus for.
     * @return total stimulus since last reset.
     */
    stim_t getAtomStimulus(Handle h) const;

    /** The following two are NOT thread-safe! Neither can be called
     * safely from multiple threads!
     */
    AttentionValuePtr getAV() { return _attentionValue; }
    void setAV(AttentionValuePtr new_av) { _attentionValue = new_av; }
}; // class

typedef std::shared_ptr<Agent> AgentPtr;

/** @}*/
}  // namespace

#endif // _OPENCOG_AGENT_H
