/*
 * opencog/dynamics/attention/ForgettingAgent.cc
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

#include "ForgettingAgent.h"

#include <algorithm>
#include <sstream>

#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/util/Config.h>

using namespace opencog;

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
    std::vector<Handle> atomsVector;
    std::back_insert_iterator< std::vector<Handle> > output2(atomsVector);
    int count = 0;
    int removalAmount;

    a->get_handles_by_type(output2, ATOM, true);
    // Sort atoms by lti, remove the lowest unless vlti is NONDISPOSABLE
    std::sort(atomsVector.begin(), atomsVector.end(), ForgettingLTIThenTVAscendingSort(a));

    removalAmount = (int) (atomsVector.size() * proportion);
    log->info("ForgettingAgent::forget - will attempt to remove %d atoms", removalAmount);

    for (unsigned int i = 0; i < atomsVector.size(); i++) {
        if (a->get_LTI(atomsVector[i]) <= forgetThreshold
                && count < removalAmount) {
            if (a->get_VLTI(atomsVector[i]) == AttentionValue::DISPOSABLE ) {
                std::string atomName = a->atom_as_string(atomsVector[i]);
                log->fine("Removing atom %s", atomName.c_str());
                // TODO: do recursive remove if neighbours are not very important
                if (!a->remove_atom(atomsVector[i])) {
                    // Atom must have already been removed through having 
                    // previously removed atoms in it's outgoing set.
                    log->error("Couldn't remove atom %s", atomName.c_str());
                    count++;
                }
                count++;
            }
        } else {
            i = atomsVector.size();
        }
    }
    log->info("ForgettingAgent::forget - %d atoms removed.", count);

}
