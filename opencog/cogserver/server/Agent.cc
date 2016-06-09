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

#include <opencog/util/Config.h>

//#define DEBUG

using namespace opencog;

concurrent_queue<Handle> opencog::newAtomsInAV;

float Agent::mean = 1;
float Agent::std = 1;

Agent::Agent(CogServer& cs, const unsigned int f) : _cogserver(cs), _frequency(f)
{
    STIAtomWage = config().get_int("ECAN_STARTING_ATOM_STI_WAGE");
    LTIAtomWage = config().get_int("ECAN_STARTING_ATOM_LTI_WAGE");

    targetSTI = config().get_int("TARGET_STI_FUNDS");
    stiFundsBuffer = config().get_int("STI_FUNDS_BUFFER");
    targetLTI = config().get_int("TARGET_LTI_FUNDS");
    ltiFundsBuffer = config().get_int("LTI_FUNDS_BUFFER");

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

void Agent::stimulateAtom(Handle h,float stimulus)
{
    int sti = h->getAttentionValue()->getSTI();
    int lti = h->getAttentionValue()->getLTI();
    int stiWage = calculate_STI_Wage() * stimulus;
    int ltiWage = calculate_LTI_Wage() * stimulus;

    h->setSTI(sti + stiWage);
    h->setLTI(lti + ltiWage);

    //newAtomsInAV.push(h);

  //updateHebbianLinks(h);
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
