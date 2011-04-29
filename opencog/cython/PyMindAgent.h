/*
 * opencog/cython/PyMindAgent.h
 *
 * Copyright (C) 2011 by The OpenCog Foundation
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

#ifndef _OPENCOG_PYAGENT_H
#define _OPENCOG_PYAGENT_H

#include <string>

#include <Python.h>

#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>

namespace opencog
{

class CogServer;
class AtomSpaceImpl;

/** The PyMindAgent Class
 * This class wraps Python MindAgents and allows the CogServer to interact
 * and manage them.
 */
class PyMindAgent : public Agent
{

protected:

public:

    /** Agent's constructor. Pass a PyObject that is a Python MindAgent object. */
    PyMindAgent(PyObject* py_agent, std::string& moduleFileName);

    /** Agent's destructor */
    virtual ~PyMindAgent();

    /** Abstract run method. Should be overriden by a derived agent with the
     *  actual agent's behavior. */
    virtual void run(CogServer* server) = 0;

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
    virtual const HandleSetSeq& getUtilizedHandleSets() const
    {
        return _utilizedHandleSets;
    }

    /** Resets the utilized handle sets */
    void resetUtilizedHandleSets();

    /**
     * Stimulate a Handle's atom.
     *
     * @param atom handle
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
     * @param handle of atom to get stimulus for.
     * @return total stimulus since last reset.
     */
    stim_t getAtomStimulus(Handle h) const;




}; // class

}  // namespace

#endif // _OPENCOG_AGENT_H

