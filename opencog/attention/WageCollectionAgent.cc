/*
 * opencog/attention/WageCollectionAgent.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
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

#include <algorithm>
#include <math.h>
#include <time.h>

#include <opencog/util/Config.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/attention/atom_types.h>

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>
#include "WageCollectionAgent.h"

//#define DEBUG

using namespace opencog;

WageCollectionAgent::WageCollectionAgent(CogServer& cs) :
        Agent(cs)
{
    // init starting wages/rents. these should quickly change and reach
    // stable values, which adapt to the system dynamics
    STIAtomRent = config().get_int("ECAN_STARTING_ATOM_STI_RENT");
    LTIAtomRent = config().get_int("ECAN_STARTING_ATOM_LTI_RENT");

    targetSTI = config().get_int("TARGET_STI_FUNDS");
    stiFundsBuffer = config().get_int("STI_FUNDS_BUFFER");
    targetLTI = config().get_int("TARGET_LTI_FUNDS");
    ltiFundsBuffer = config().get_int("LTI_FUNDS_BUFFER");

    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("WageCollectionAgent.log", Logger::FINE,
    true));
}

WageCollectionAgent::~WageCollectionAgent()
{
    if (log)
        delete log;
}

void WageCollectionAgent::setLogger(Logger* _log)
{
    if (log)
        delete log;
    log = _log;
}

Logger* WageCollectionAgent::getLogger()
{
    return log;
}

void WageCollectionAgent::run()
{
    a = &_cogserver.getAtomSpace();

    HandleSeq atoms;
    size_t size;

    std::back_insert_iterator< std::vector<Handle> > out_hi(atoms);

    a->get_handle_set_in_attentional_focus(out_hi);

    size = atoms.size();

    if (size == 0)
        return;

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,size-1);

    Handle h = atoms[distribution(generator)];

    int sti = h->getAttentionValue()->getSTI();
    int lti = h->getAttentionValue()->getLTI();
    int stiRent = calculate_STI_Rent();
    int ltiRent = calculate_LTI_Rent();

    //printf("stiRent: %d ",stiRent);
    //printf("sti: %d ",sti);

    if (stiRent > sti)
        stiRent = sti;

    if (ltiRent > lti)
        ltiRent = lti;

    //printf("stiRent: %d \n",stiRent);

    h->setSTI(sti - stiRent);
    h->setLTI(lti - ltiRent);
}

int WageCollectionAgent::calculate_STI_Rent()
{
    int funds = a->get_STI_funds();
    float diff  = targetSTI - funds;
    float ndiff = diff / stiFundsBuffer;
    ndiff = std::min(ndiff,1.0f);
    ndiff = std::max(ndiff,-0.9f);
    //printf("ndiff: %f   ",ndiff);

    return STIAtomRent + floor(STIAtomRent * ndiff);
}

int WageCollectionAgent::calculate_LTI_Rent()
{
    int funds = a->get_LTI_funds();
    float diff  = targetLTI - funds;
    float ndiff = diff / ltiFundsBuffer;
    ndiff = std::min(ndiff,1.0f);
    ndiff = std::max(ndiff,-1.0f);

    return LTIAtomRent + (LTIAtomRent * ndiff);
}
