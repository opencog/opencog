/*
 * opencog/attention/ForgettingAgent.cc
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
#include <sstream>

#include <opencog/atomspace/AtomSpace.h>

#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/attention/atom_types.h>
#include <opencog/util/Config.h>
#include "ForgettingAgent.h"

using namespace opencog;

ForgettingAgent::ForgettingAgent(CogServer& cs) :
    Agent(cs)
{
    // No limit to lti of removed atoms
    // Convert MAXLTI to a string for storing in the configuration
    std::ostringstream buf;
    buf << AttentionValue::MAXLTI;
    std::string defaultForgetThreshold = buf.str();
    config().set("ECAN_FORGET_THRESHOLD", defaultForgetThreshold);

    forgetThreshold = (AttentionValue::lti_t)
                      (config().get_int("ECAN_FORGET_THRESHOLD", AttentionValue::MAXLTI));

    //Todo: Make configurable
    maxSize = config().get_int("ECAN_ATOMSPACE_MAXSIZE", 10000);
    accDivSize = config().get_int("ECAN_ATOMSPACE_ACCEPTABLE_SIZE_SPREAD", 100);

    // Provide a logger, but disable it initially
    setLogger(new opencog::Logger("ForgettingAgent.log", Logger::WARN, true));
}

ForgettingAgent::~ForgettingAgent() {}

void ForgettingAgent::run()
{
    _log->fine("=========== ForgettingAgent::run =======");
    forget();
}

void ForgettingAgent::forget()
{
    HandleSeq atomsVector;
    std::back_insert_iterator<HandleSeq> output2(atomsVector);
    int count = 0;
    int removalAmount;
    bool recursive;

    _as->get_handles_by_type(output2, ATOM, true);

    int asize = atomsVector.size();
    if (asize < (maxSize + accDivSize)) {
        return;
    }

    fprintf(stdout,"Forgetting Stuff, Atomspace Size: %d \n",asize);
    // Sort atoms by lti, remove the lowest unless vlti is NONDISPOSABLE
    std::sort(atomsVector.begin(), atomsVector.end(), ForgettingLTIThenTVAscendingSort(_as));

    removalAmount = asize - (maxSize - accDivSize);
    _log->info("ForgettingAgent::forget - will attempt to remove %d atoms", removalAmount);

    for (unsigned int i = 0; i < atomsVector.size(); i++)
    {
        if (atomsVector[i]->getAttentionValue()->getLTI() <= forgetThreshold
                && count < removalAmount)
        {
            if (atomsVector[i]->getAttentionValue()->getVLTI() == AttentionValue::DISPOSABLE )
            {
                std::string atomName = atomsVector[i]->toString();
                _log->fine("Removing atom %s", atomName.c_str());
                // TODO: do recursive remove if neighbours are not very important
                IncomingSet iset = atomsVector[i]->getIncomingSet(_as);
                recursive = true;
                for (const LinkPtr& h : iset)
                {
                    if (h->getType() != ASYMMETRIC_HEBBIAN_LINK) {
                        recursive = false;
                        break;
                    }
                }
                if (!recursive)
                    continue;

                atomsVector[i]->setSTI(0);
                atomsVector[i]->setLTI(0);
                if (!_as->remove_atom(atomsVector[i],recursive)) {
                    // Atom must have already been removed through having
                    // previously removed atoms in it's outgoing set.
                    _log->error("Couldn't remove atom %s", atomName.c_str());
                }
                count++;
                count += iset.size();
            }
        } else {
            break;
        }
    }
    _log->info("ForgettingAgent::forget - %d atoms removed.", count);
}
