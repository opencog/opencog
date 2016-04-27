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
#include <opencog/attention/atom_types.h>
#include <opencog/truthvalue/SimpleTruthValue.h>

//#define DEBUG

using namespace opencog;

Agent::Agent(CogServer& cs, const unsigned int f) : _cogserver(cs), _frequency(f)
{
  //STIAtomWage = config().get_int("ECAN_STARTING_ATOM_STI_RENT");
  //LTIAtomWage = config().get_int("ECAN_STARTING_ATOM_LTI_RENT");

  //targetSTI = config().get_int("STARTING_STI_FUNDS");
  //stiFundsBuffer = config().get_int("STI_FUNDS_BUFFER");
  //targetLTI = config().get_int("STARTING_LTI_FUNDS");
  //ltiFundsBuffer = config().get_int("LTI_FUNDS_BUFFER");


    STIAtomWage = 100;
    LTIAtomWage = 100;

    targetSTI = 10000;
    stiFundsBuffer = 10000;
    targetLTI = 10000;
    ltiFundsBuffer = 10000;


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

    as = &_cogserver.getAtomSpace();
    conn = as->removeAtomSignal(
            boost::bind(&Agent::atomRemoved, this, _1));
}

Agent::~Agent()
{
    // give back funds
    as->update_STI_funds(_attentionValue->getSTI());
    as->update_LTI_funds(_attentionValue->getLTI());

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
    //removeAtomStimulus(h);
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
    as->update_STI_funds(-stiWage);

    h->setLTI(lti + ltiWage);
    as->update_LTI_funds(-ltiWage);

    updateHebbianLinks(h);
}

AttentionValue::sti_t Agent::calculate_STI_Wage()
{
    int funds = as->get_STI_funds();
    float diff  = funds - targetSTI;
    float ndiff = diff / stiFundsBuffer;
    ndiff = std::min(ndiff,1.0f);
    ndiff = std::max(ndiff,-1.0f);

    return STIAtomWage + (STIAtomWage * ndiff);
}

AttentionValue::lti_t Agent::calculate_LTI_Wage()
{
    int funds = as->get_LTI_funds();
    float diff  = funds - targetLTI;
    float ndiff = diff / ltiFundsBuffer;
    ndiff = std::min(ndiff,1.0f);
    ndiff = std::max(ndiff,-1.0f);

    return LTIAtomWage + (LTIAtomWage * ndiff);
}

void Agent::updateHebbianLinks(Handle source)
{
    float tcDecayRate = 0.1f;
    float tc, old_tc, new_tc;

    IncomingSet links = source->getIncomingSetByType(ASYMMETRIC_HEBBIAN_LINK);

    for (LinkPtr h : links) {
        if (source != h->getOutgoingAtom(0))
            continue;
        HandleSeq outgoing = h->getOutgoingSet();
        new_tc = targetConjunction(outgoing);

        // old link strength decays
        TruthValuePtr oldtv  = h->getTruthValue();
        old_tc = oldtv->getMean();
        tc = tcDecayRate * new_tc + (1.0f - tcDecayRate) * old_tc;

        //if (tc < 0)
        //  printf("old_tc: %f new_tc: %f  tc: %f \n",old_tc,new_tc,tc);

        //update truth value accordingly
        TruthValuePtr newtv(SimpleTruthValue::createTV(tc, 1));
        h->merge(newtv);
    }
    //h->setTruthValue(SimpleTruthValue::createTV(tc, 1));
}

float Agent::targetConjunction(HandleSeq handles)
{
    if (handles.size() != 2) {
        throw RuntimeException(
                TRACE_INFO,
                "Size of outgoing set of a hebbian link must be 2.");
    }
    //XXX: Should this be normalised to 0->1 Range
  //auto normsti_i = as->get_normalised_STI(handles[0],true,true);
  //auto normsti_j = as->get_normalised_STI(handles[1],true,true);
  //float conj = (normsti_i * normsti_j);
    auto normsti_i = as->get_normalised_zero_to_one_STI(handles[0],true,true);
    auto normsti_j = as->get_normalised_zero_to_one_STI(handles[1],true,true);
    float conj = (normsti_i * normsti_j) + ((normsti_j - normsti_i) * std::abs(normsti_j -normsti_i));
    conj = (conj + 1.0f) / 2.0f;

    //printf("normsti_i: %f   normsti_j: %f   conj: %f    nconj: %f \n",normsti_i,normsti_j,conj,nconj);

    return conj;
}
