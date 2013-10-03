/*
 * opencog/server/Agent.cc
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

#include "Agent.h"

#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>

using namespace opencog;

Agent::Agent(CogServer& cs, const unsigned int f) : _cogserver(cs), _frequency(f)
{
    // an empty set of parameters and defaults (so that various
    // methods will still work even if none are set in this or a derived
    // class)
    static const std::string defaultConfig[] = {
        "", ""
    };
    setParameters(defaultConfig);

    stimulatedAtoms = new AtomStimHashMap();
    totalStimulus = 0;

    conn = _cogserver.getAtomSpace().atomSpaceAsync->removeAtomSignal(
            boost::bind(&Agent::atomRemoved, this, _1, _2));
}

Agent::~Agent()
{
    // give back funds
    _cogserver.getAtomSpace().getAttentionBank().updateSTIFunds(getAttentionValue().getSTI());
    _cogserver.getAtomSpace().getAttentionBank().updateLTIFunds(getAttentionValue().getLTI());

    resetUtilizedHandleSets();
    conn.disconnect();
    delete stimulatedAtoms;
}

void Agent::setParameters(const std::string* params)
{
    PARAMETERS = params;

    for (unsigned int i = 0; params[i] != ""; i += 2) {
        if (!config().has(params[i])) {
           config().set(params[i], params[i + 1]);
        }
    }
}

std::string Agent::to_string() const
{
    std::ostringstream oss;
    oss << classinfo().id;
    oss << " {\"";
    for (unsigned int i = 0; PARAMETERS[i] != ""; i += 2) {
        if (i != 0) oss << "\", \"";
        oss << PARAMETERS[i] << "\" => \"" << config()[PARAMETERS[i]];
    }
    oss << "\"}";
    return oss.str();
}

void Agent::atomRemoved(AtomSpaceImpl* a, Handle h)
{
    for (size_t i = 0; i < _utilizedHandleSets.size(); i++)
        _utilizedHandleSets[i].erase(h);
    removeAtomStimulus(h);
}

void Agent::resetUtilizedHandleSets()
{
    for (size_t i = 0; i < _utilizedHandleSets.size(); i++)
        _utilizedHandleSets[i].clear();
    _utilizedHandleSets.clear();
}


stim_t Agent::stimulateAtom(Handle h, stim_t amount)
{
    // Add atom to the map of atoms with stimulus
    // and add stimulus to it
    (*stimulatedAtoms)[h] += amount;

    // update record of total stimulus given out
    totalStimulus += amount;
    //logger().fine("%d added to totalStimulus, now %d", amount, totalStimulus);
    return totalStimulus;
}

void Agent::removeAtomStimulus(Handle h)
{
    stim_t amount;
    // if handle not in map then return
    if (stimulatedAtoms->find(h) == stimulatedAtoms->end())
        return;

    amount = (*stimulatedAtoms)[h];
    stimulatedAtoms->erase(h);

    // update record of total stimulus given out
    totalStimulus -= amount;
}

stim_t Agent::stimulateAtom(HandleSeq hs, stim_t amount)
{
    stim_t split;

    // how much to give each atom
    split = amount / hs.size();

    foreach(Handle handle, hs) {
        stimulateAtom(handle, split);
    }
    // return unused stimulus
    return amount - (split * hs.size());
}

stim_t Agent::resetStimulus()
{
    stimulatedAtoms->clear();
    // reset stimulus counter
    totalStimulus = 0;
    return totalStimulus;
}

stim_t Agent::getTotalStimulus() const
{
    return totalStimulus;
}

stim_t Agent::getAtomStimulus(Handle h) const
{
    if (stimulatedAtoms->find(h) == stimulatedAtoms->end()) {
        return 0;
    } else {
        return (*stimulatedAtoms)[h];
    }
}

