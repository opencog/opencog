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

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>

#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/attention/atom_types.h>
#include <opencog/util/Config.h>
#include "ForgettingAgent.h"

using namespace opencog;

int ForgettingAgent::maxSize = 100000;

ForgettingAgent::ForgettingAgent(CogServer& cs) :
    Agent(cs)
{
    std::string defaultForgetThreshold;
    std::ostringstream buf;

    // No limit to lti of removed atoms
    // Convert MAXLTI to a string for storing in the configuration
    buf << AttentionValue::MAXLTI;
    defaultForgetThreshold = buf.str();
    config().set("ECAN_FORGET_THRESHOLD",defaultForgetThreshold);

    forgetPercentage = (float) (config().get_double("ECAN_FORGET_PERCENTAGE"));

    forgetThreshold = (AttentionValue::lti_t)
                      (config().get_int("ECAN_FORGET_THRESHOLD"));


    //Todo: Make configurable
    maxSize = 100000;
    accDivSize = 10;

    // Provide a logger, but disable it initially
    log = NULL;
    setLogger(new opencog::Logger("ForgettingAgent.log", Logger::WARN, true));
}

ForgettingAgent::~ForgettingAgent()
{
    if (log) delete log;
}

Logger* ForgettingAgent::getLogger()
{
    return log;
}

void ForgettingAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

void ForgettingAgent::run()
{
    log->fine("=========== ForgettingAgent::run =======");
    a = &_cogserver.getAtomSpace();
    forget(forgetPercentage);
}

void ForgettingAgent::forget(float proportion = 0.10f)
{
    HandleSeq atomsVector;
    std::back_insert_iterator<HandleSeq> output2(atomsVector);
    int count = 0;
    int removalAmount;
    bool recursive;

    a->get_handles_by_type(output2, ATOM, true);

    int asize = atomsVector.size();
    if (asize < (maxSize + accDivSize)) {
        return;
    }

    fprintf(stdout,"Forgetting Stuff, Atomspace Size: %d \n",asize);
    // Sort atoms by lti, remove the lowest unless vlti is NONDISPOSABLE
    std::sort(atomsVector.begin(), atomsVector.end(), ForgettingLTIThenTVAscendingSort(a));

    removalAmount = asize - (maxSize - accDivSize); //(int) (atomsVector.size() * proportion);
    log->info("ForgettingAgent::forget - will attempt to remove %d atoms", removalAmount);

    for (unsigned int i = 0; i < atomsVector.size(); i++)
    {
        if (atomsVector[i]->getAttentionValue()->getLTI() <= forgetThreshold
                && count < removalAmount)
        {
            if (atomsVector[i]->getAttentionValue()->getVLTI() == AttentionValue::DISPOSABLE )
            {
                std::string atomName = a->atom_as_string(atomsVector[i]);
                log->fine("Removing atom %s", atomName.c_str());
                // TODO: do recursive remove if neighbours are not very important
                IncomingSet iset = atomsVector[i]->getIncomingSet(a);
                recursive = true;
                for (LinkPtr h : iset)
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
                //fprintf(stdout,"Removing atom %s",atomsVector[i]->toString().c_str());
                if (!a->remove_atom(atomsVector[i],recursive)) {
                    // Atom must have already been removed through having
                    // previously removed atoms in it's outgoing set.
                    log->error("Couldn't remove atom %s", atomName.c_str());
                }
                count++;
                count += iset.size();
            }
        } else {
            break;
        }
    }
    fprintf(stdout,"Forgetting Stuff, Atoms Forgotten: %d \n",count);
    log->info("ForgettingAgent::forget - %d atoms removed.", count);

}
