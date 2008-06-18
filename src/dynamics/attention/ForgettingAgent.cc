/*
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#include <CogServer.h>

namespace opencog
{

ForgettingAgent::ForgettingAgent()
{
    // forget 0.1% of atoms
    forgetPercentage = 0.001f;
    // No limit to lti of removed atoms
    forgetThreshold = AttentionValue::MAXLTI;
}

ForgettingAgent::~ForgettingAgent()
{

}

void ForgettingAgent::run(CogServer *c)
{
    a = c->getAtomSpace();
    forget(forgetPercentage);

}

void ForgettingAgent::forget(float proportion = 0.10f)
{
    HandleEntry *atoms;
    std::vector<Handle> atomsVector;
    int count = 0;
    int removalAmount;

    atoms = a->getAtomTable().getHandleSet(ATOM, true);
    // Sort atoms by lti, remove the lowest unless vlti is NONDISPOSABLE
    atoms->toHandleVector(atomsVector);
    std::sort(atomsVector.begin(), atomsVector.end(), ForgettingLTIThenTVAscendingSort());
    delete atoms;

    removalAmount = (int) (atomsVector.size() * proportion);

    for (unsigned int i = 0; i < atomsVector.size() ; i++) {
        if (a->getLTI(atomsVector[i]) <= forgetThreshold
                && count < removalAmount) {
            if (a->getVLTI(atomsVector[i]) != AttentionValue::NONDISPOSABLE ) {
                //cout << "Removing atom " <<  TLB::getAtom(atomsVector[i])->toString().c_str() << endl;
                logger().fine("Removing atom %s", TLB::getAtom(atomsVector[i])->toString().c_str());
                if (!a->removeAtom(atomsVector[i])) {
                    logger().error("Couldn't remove atom %s", TLB::getAtom(atomsVector[i])->toString().c_str());
                    logger().error("Aborting forget process");
                    return;
                }
                count++;
            }
        } else {
            i = atomsVector.size();
        }
    }

}

}; //namespace
