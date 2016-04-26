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

//#define DEBUG

using namespace opencog;

Agent::Agent(CogServer& cs, const unsigned int f) : _cogserver(cs), _frequency(f)
{
    _attentionValue = AttentionValue::DEFAULT_AV();

    // an empty set of parameters and defaults (so that various
    // methods will still work even if none are set in this or a derived
    // class)
    static const std::string defaultConfig[] = {
        "", ""
    };
    setParameters(defaultConfig);

    stimulatedAtoms = new AtomStimHashMap();
    totalStimulus = 0;

    conn = _cogserver.getAtomSpace().removeAtomSignal(
            boost::bind(&Agent::atomRemoved, this, _1));
}

Agent::~Agent()
{
    // give back funds
    _cogserver.getAtomSpace().update_STI_funds(_attentionValue->getSTI());
    _cogserver.getAtomSpace().update_LTI_funds(_attentionValue->getLTI());

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

void Agent::atomRemoved(AtomPtr atom)
{
    Handle h = atom->getHandle();
    {
        std::lock_guard<std::mutex> lock(_handleSetMutex);
        for (size_t i = 0; i < _utilizedHandleSets.size(); i++)
            _utilizedHandleSets[i].erase(h);
    }
    removeAtomStimulus(h);
}

void Agent::resetUtilizedHandleSets()
{
    std::lock_guard<std::mutex> lock(_handleSetMutex);
    for (size_t i = 0; i < _utilizedHandleSets.size(); i++)
        _utilizedHandleSets[i].clear();
    _utilizedHandleSets.clear();
}


//tim_t Agent::stimulateAtom(Handle h, stim_t amount)
//
//   {
//       std::lock_guard<std::mutex> lock(stimulatedAtomsMutex);
//
//       // Add atom to the map of atoms with stimulus
//       // and add stimulus to it
//       (*stimulatedAtoms)[h] += amount;
//   }
//
//   // update record of total stimulus given out
//   totalStimulus += amount;
//
//   logger().fine("Atom %d received stimulus of %d, total now %d",
//                 h.value(),
//                 amount,
//                 totalStimulus.load(std::memory_order_relaxed));
//
//ifdef DEBUG
//   std::cout << "Atom " << h.value() << " received stimulus of " << amount <<
//                ", total now " << totalStimulus.load(std::memory_order_relaxed)
//                << "\n";
//endif
//
//   return totalStimulus.load(std::memory_order_relaxed);
//
//
//oid Agent::removeAtomStimulus(Handle h)
//
//   stim_t amount;
//   {
//       std::lock_guard<std::mutex> lock(stimulatedAtomsMutex);
//       // if handle not in map then return
//       if (stimulatedAtoms->find(h) == stimulatedAtoms->end())
//           return;
//
//       amount = (*stimulatedAtoms)[h];
//       stimulatedAtoms->erase(h);
//   }
//
//   // update record of total stimulus given out
//   totalStimulus -= amount;
//
//
//tim_t Agent::stimulateAtom(HandleSeq hs, stim_t amount)
//
//   stim_t split;
//
//   // how much to give each atom
//   split = amount / hs.size();
//
//   for (Handle handle : hs) {
//       stimulateAtom(handle, split);
//   }
//   // return unused stimulus
//   return amount - (split * hs.size());
//
//
//tim_t Agent::resetStimulus()
//
//   {
//       std::lock_guard<std::mutex> lock(stimulatedAtomsMutex);
//       stimulatedAtoms->clear();
//   }
//   // reset stimulus counter
//   totalStimulus = 0;
//   return totalStimulus.load(std::memory_order_relaxed);
//
//
//tim_t Agent::getTotalStimulus() const
//
//   return totalStimulus;
//
//
//tim_t Agent::getAtomStimulus(Handle h) const
//
//   std::lock_guard<std::mutex> lock(stimulatedAtomsMutex);
//   if (stimulatedAtoms->find(h) == stimulatedAtoms->end()) {
//       return 0;
//   } else {
//       return (*stimulatedAtoms)[h];
//   }
//

void Agent::stimulateAtom(Handle h,float stimulus)
{
    int sti = h->getAttentionValue()->getSTI();
    int lti = h->getAttentionValue()->getLTI();
    int stiWage = calculate_STI_Wage() * stimulus;
    int ltiWage = calculate_LTI_Wage() * stimulus;

    h->setSTI(sti + stiWage);
    _as.update_STI_funds(-stiWage);

    h->setLTI(lti + ltiWage);
    _as.update_LTI_funds(-ltiWage);

    hebbianUpdatingUpdate(h);
}

sti_t Agent::calculate_STI_Wage()
{
    int funds = _as.get_STI_funds();
    float diff  = funds - targetSTI;
    float ndiff = diff / stiFundsBuffer;
    ndiff = std::min(ndiff,1.0f);
    ndiff = std::max(ndiff,-1.0f);

    return STIAtomWage + (STIAtomWage * ndiff);
}

lti_t Agent::calculate_LTI_Wage()
{
    int funds = _as.get_LTI_funds();
    float diff  = funds - targetLTI;
    float ndiff = diff / ltiFundsBuffer;
    ndiff = std::min(ndiff,1.0f);
    ndiff = std::max(ndiff,-1.0f);

    return LTIAtomWage + (LTIAtomWage * ndiff);
}
