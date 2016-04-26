/*
 * opencog/attention/StochasticImportanceUpdatingAgent.cc
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
#include "StochasticImportanceUpdatingAgent.h"

//#define DEBUG

using namespace opencog;

StochasticImportanceUpdatingAgent::StochasticImportanceUpdatingAgent(CogServer& cs) :
        Agent(cs)
{
    // init starting wages/rents. these should quickly change and reach
    // stable values, which adapt to the system dynamics
    STIAtomRent = config().get_int("ECAN_STARTING_ATOM_STI_RENT");
    LTIAtomRent = config().get_int("ECAN_STARTING_ATOM_LTI_RENT");

  //targetSTI = config().get_int("STARTING_STI_FUNDS");
  //stiFundsBuffer = config().get_int("STI_FUNDS_BUFFER");
  //targetLTI = config().get_int("STARTING_LTI_FUNDS");
  //ltiFundsBuffer = config().get_int("LTI_FUNDS_BUFFER");

    targetSTI = 10000;
    stiFundsBuffer = 10000;
    targetLTI = 10000;
    ltiFundsBuffer = 10000;

    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("StochasticImportanceUpdatingAgent.log", Logger::FINE,
    true));
}

StochasticImportanceUpdatingAgent::~StochasticImportanceUpdatingAgent()
{
    if (log)
        delete log;
}

void StochasticImportanceUpdatingAgent::setLogger(Logger* _log)
{
    if (log)
        delete log;
    log = _log;
}

Logger* StochasticImportanceUpdatingAgent::getLogger()
{
    return log;
}



void StochasticImportanceUpdatingAgent::run()
{
    a = &_cogserver.getAtomSpace();

    HandleSeq atoms;
    a->get_all_atoms(atoms);

    atoms.erase(remove_if(atoms.begin(),atoms.end(),
                    [](Handle h) -> bool {
                    return classserver().isA(h->getType(),HEBBIAN_LINK);
                    }),atoms.end());

    size_t size = atoms.size();

    //printf("size: %d    ",size);

    if (size == 0)
        return;

    int index = rand() % size;

    Handle h = atoms[index];

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
    a->update_STI_funds(stiRent);

    h->setLTI(lti - ltiRent);
    a->update_LTI_funds(ltiRent);
}

int StochasticImportanceUpdatingAgent::calculate_STI_Rent()
{
    int funds = a->get_STI_funds();
    float diff  = targetSTI - funds;
    float ndiff = diff / stiFundsBuffer;
    ndiff = std::min(ndiff,1.0f);
    ndiff = std::max(ndiff,-1.0f);
    //printf("ndiff: %f   ",ndiff);

    return STIAtomRent + floor(STIAtomRent * ndiff);
}

int StochasticImportanceUpdatingAgent::calculate_LTI_Rent()
{
    int funds = a->get_LTI_funds();
    float diff  = targetLTI - funds;
    float ndiff = diff / ltiFundsBuffer;
    ndiff = std::min(ndiff,1.0f);
    ndiff = std::max(ndiff,-1.0f);

    return LTIAtomRent + (LTIAtomRent * ndiff);
}
