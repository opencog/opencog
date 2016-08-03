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
    STIAtomWage = config().get_int("ECAN_STARTING_ATOM_STI_WAGE", 10);
    LTIAtomWage = config().get_int("ECAN_STARTING_ATOM_LTI_WAGE", 10);

    targetSTI = config().get_int("TARGET_STI_FUNDS", 10000);
    stiFundsBuffer = config().get_int("STI_FUNDS_BUFFER", 10000);
    targetLTI = config().get_int("TARGET_LTI_FUNDS", 10000);
    ltiFundsBuffer = config().get_int("LTI_FUNDS_BUFFER", 10000);

    _attentionValue = AttentionValue::DEFAULT_AV();

    setParameters({""});

    _totalStimulus = 0;

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

void Agent::atomRemoved(const AtomPtr& atom)
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

long Agent::stimulateAtom(const Handle& h, stim_t amount)
{
    {
        std::lock_guard<std::mutex> lock(_stimulatedAtomsMutex);

        // Add atom to the map of atoms with stimulus
        // and add stimulus to it
        _stimulatedAtoms[h] += amount;
    }

    // Update record of total stimulus given out.
    // XXX FIXME: This could overflow and you'd never know it!
    _totalStimulus += amount;

    logger().fine("Atom %s received stimulus of %d, total now %ld",
                  h->toString().c_str(), amount, _totalStimulus);

    return _totalStimulus;
}

void Agent::removeAtomStimulus(const Handle& h)
{
    std::lock_guard<std::mutex> lock(_stimulatedAtomsMutex);

    // if handle not in map then return
    auto pr = _stimulatedAtoms.find(h);
    if (pr == _stimulatedAtoms.end()) return;

    stim_t amount = pr->second;
    _stimulatedAtoms.erase(h);

    // Update record of total stimulus given out.
    // XXX FIXME: This could underflow and you'd never know it!
    _totalStimulus -= amount;
}

stim_t Agent::stimulateAtom(const HandleSeq& hs, stim_t amount)
{
    // How much to give each atom.
    stim_t split = amount / hs.size();

    for (const Handle& handle : hs)
        stimulateAtom(handle, split);

    // Return unused stimulus.
    return amount - (split * hs.size());
}

void Agent::resetStimulus(void)
{
    std::lock_guard<std::mutex> lock(_stimulatedAtomsMutex);
    _stimulatedAtoms.clear();

    // reset stimulus counter
    _totalStimulus = 0;
}

long Agent::getTotalStimulus(void) const
{
    return _totalStimulus;
}

stim_t Agent::getAtomStimulus(const Handle& h) const
{
    std::lock_guard<std::mutex> lock(_stimulatedAtomsMutex);
    auto pr = _stimulatedAtoms.find(h);
    if (pr == _stimulatedAtoms.end())
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
    double diff  = funds - _targetSTI;
    double ndiff = diff / _stiFundsBuffer;
    ndiff = std::min(ndiff, 1.0);
    ndiff = std::max(ndiff, -1.0);

    return _STIAtomWage + _STIAtomWage * ndiff;
}

AttentionValue::lti_t Agent::calculate_LTI_Wage(void)
{
    long funds = _as->get_LTI_funds();
    double diff  = funds - _targetLTI;
    double ndiff = diff / _ltiFundsBuffer;
    ndiff = std::min(ndiff, 1.0);
    ndiff = std::max(ndiff, -1.0);

    return _LTIAtomWage + _LTIAtomWage * ndiff;
}
