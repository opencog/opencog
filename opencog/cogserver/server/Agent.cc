/*
 * opencog/cogserver/server/Agent.cc
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

#include <opencog/cogserver/server/CogServer.h>
#include <opencog/util/Config.h>

#define DEBUG

using namespace opencog;

Agent::Agent(CogServer& cs, const unsigned int f) :
    _log(nullptr), _cogserver(cs), _frequency(f)
{
#if 0
This causes the cogserver to crash!
See https://github.com/opencog/opencog/issues/2329

    STIAtomWage = config().get_int("ECAN_STARTING_ATOM_STI_WAGE");
    LTIAtomWage = config().get_int("ECAN_STARTING_ATOM_LTI_WAGE");

    targetSTI = config().get_int("TARGET_STI_FUNDS");
    stiFundsBuffer = config().get_int("STI_FUNDS_BUFFER");
    targetLTI = config().get_int("TARGET_LTI_FUNDS");
    ltiFundsBuffer = config().get_int("LTI_FUNDS_BUFFER");
#endif

    _attentionValue = AttentionValue::DEFAULT_AV();

    setParameters({""});

    totalStimulus = 0;

    conn = _cogserver.getAtomSpace().removeAtomSignal(
            boost::bind(&Agent::atomRemoved, this, _1));

    _as = &cs.getAtomSpace();
}

Agent::~Agent()
{
    // give back funds
    _as->update_STI_funds(_attentionValue->getSTI());
    _as->update_LTI_funds(_attentionValue->getLTI());

    resetUtilizedHandleSets();
    conn.disconnect();

    if (_log) delete _log;
}

void Agent::setLogger(Logger* l)
{
    if (_log) delete _log;
    _log = l;
}

Logger* Agent::getLogger()
{
    return _log;
}


void Agent::setParameters(const std::vector<std::string>& params)
{
    _parameters = params;
    for (unsigned int i = 0; params[i] != ""; i += 2)
    {
        if (!config().has(params[i]))
           config().set(params[i], params[i + 1]);
    }
}

std::string Agent::to_string() const
{
    std::ostringstream oss;
    oss << classinfo().id;
    oss << " {\"";
    for (unsigned int i = 0; _parameters[i] != ""; i += 2) {
        if (i != 0) oss << "\", \"";
        oss << _parameters[i] << "\" => \"" << config()[_parameters[i]];
    }
    oss << "\"}";
    return oss.str();
}

void Agent::atomRemoved(AtomPtr atom)
{
    Handle h(atom->getHandle());
    {
        std::lock_guard<std::mutex> lock(_handleSetMutex);
        for (size_t i = 0; i < _utilizedHandleSets.size(); i++)
            _utilizedHandleSets[i].erase(h);
    }
    removeAtomStimulus(h);
}

void Agent::resetUtilizedHandleSets(void)
{
    std::lock_guard<std::mutex> lock(_handleSetMutex);
    for (size_t i = 0; i < _utilizedHandleSets.size(); i++)
        _utilizedHandleSets[i].clear();
    _utilizedHandleSets.clear();
}


stim_t Agent::stimulateAtom(const Handle& h, stim_t amount)
{
    {
        std::lock_guard<std::mutex> lock(stimulatedAtomsMutex);

        // Add atom to the map of atoms with stimulus
        // and add stimulus to it
        stimulatedAtoms[h] += amount;
    }

    // update record of total stimulus given out
    totalStimulus += amount;

    logger().fine("Atom %d received stimulus of %d, total now %d",
                  h.value(), amount, totalStimulus);

    return totalStimulus;
}

void Agent::removeAtomStimulus(const Handle& h)
{
    std::lock_guard<std::mutex> lock(stimulatedAtomsMutex);

    // if handle not in map then return
    auto pr = stimulatedAtoms.find(h);
    if (pr == stimulatedAtoms.end()) return;

    stim_t amount = pr->second;
    stimulatedAtoms.erase(h);

    // update record of total stimulus given out
    totalStimulus -= amount;
}

stim_t Agent::stimulateAtom(const HandleSeq& hs, stim_t amount)
{
    stim_t split;

    // how much to give each atom
    split = amount / hs.size();

    for (const Handle& handle : hs) {
        stimulateAtom(handle, split);
    }
    // return unused stimulus
    return amount - (split * hs.size());
}

stim_t Agent::resetStimulus(void)
{
    std::lock_guard<std::mutex> lock(stimulatedAtomsMutex);
    stimulatedAtoms.clear();

    // reset stimulus counter
    totalStimulus = 0;
    return 0;
}

stim_t Agent::getTotalStimulus(void) const
{
    return totalStimulus;
}

stim_t Agent::getAtomStimulus(const Handle& h) const
{
    std::lock_guard<std::mutex> lock(stimulatedAtomsMutex);
    auto pr = stimulatedAtoms.find(h);
    if (pr == stimulatedAtoms.end())
        return 0;
    else
        return pr->second;
}

void Agent::experimentalStimulateAtom(const Handle& h, double stimulus)
{
    long sti = h->getAttentionValue()->getSTI();
    long lti = h->getAttentionValue()->getLTI();
    long stiWage = calculate_STI_Wage() * stimulus;
    long ltiWage = calculate_LTI_Wage() * stimulus;

    h->setSTI(sti + stiWage);
    h->setLTI(lti + ltiWage);
}

AttentionValue::sti_t Agent::calculate_STI_Wage(void)
{
    long funds = _as->get_STI_funds();
    double diff  = funds - targetSTI;
    double ndiff = diff / stiFundsBuffer;
    ndiff = std::min(ndiff, 1.0);
    ndiff = std::max(ndiff, -1.0);

    return STIAtomWage + (STIAtomWage * ndiff);
}

AttentionValue::lti_t Agent::calculate_LTI_Wage(void)
{
    long funds = _as->get_LTI_funds();
    double diff  = funds - targetLTI;
    double ndiff = diff / ltiFundsBuffer;
    ndiff = std::min(ndiff, 1.0);
    ndiff = std::max(ndiff, -1.0);

    return LTIAtomWage + (LTIAtomWage * ndiff);
}
