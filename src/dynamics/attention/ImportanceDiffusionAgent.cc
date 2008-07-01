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
#include <gslwrap/matrix_float.h>

#include "ImportanceDiffusionAgent.h"

#include <platform.h>

#include <CogServer.h>
#include <Link.h>

namespace opencog
{

ImportanceDiffusionAgent::ImportanceDiffusionAgent()
{

}

ImportanceDiffusionAgent::~ImportanceDiffusionAgent()
{

}

void ImportanceDiffusionAgent::run(CogServer* server)
{

    a = server->getAtomSpace();
    spreadImportance();

}


void ImportanceDiffusionAgent::spreadImportance()
{
    gsl::matrix connections;
    gsl::matrix inverse;
    gsl::matrix stiVector;

    // The source and destinations of STI
    std::set<Handle> diffusionAtoms;

    std::vector<Handle> links;
    std::vector<Handle>::iterator hi;
    std::back_insert_iterator< std::vector<Handle> > out_hi(links);

    // Get all HebbianLinks
    a->getHandleSet(out_hi, HEBBIAN_LINK, true);

    hi = atoms.begin();
    while (hi != links.end()) {
        // Get all atoms in outgoing set of link
        std::vector<Handle> targets;
        std::vector<Handle>::iterator targetItr;
        Handle linkHandle = *hi;
        targets = TLB::getAtom(linkHandle)->getOutgoingSet();

        for (targetItr = targets.begin(); targetItr++;
                targetItr != targets.end()) {
            diffusionAtoms.insert((Handle&) targetItr);
        }
    }
    // diffusionNodes now contains all atoms involved in spread.
    
    // go through diffusionNodes and add each to the sti vector.
    // position in stiVector matches set (set is ordered and won't change
    // unless you add to it.
    stiVector.set_dimensions( diffusionNodes.size, 1 );
    for (int i=0; i++; i < diffusionAtoms.size) {
        Handle dAtom = diffusionAtoms[i];
        // TODO: The sti needs to be scaled.
        stiVector.set_element(i,0,a->getSTI(dAtom));
    }

#ifdef DEBUG
    cout << diffusionNodes;
    cout << inverse;
#endif

    // set connectivity matrix size
    // size is dependent on the number of atoms that are connected
    // by a HebbianLink in some way.
    connections.set_dimensions( diffusionNodes.size, diffusionNodes.size );
    
    // calculate inverse
    inverse = connections.inverse();

    // perform matrix multiplication
    //
    // set the sti of all atoms based on new values in new 
    // vector from multiplication 

    AttentionValue::sti_t current;

    std::vector<Handle> atoms;
    std::vector<Handle>::iterator hi;
    std::back_insert_iterator< std::vector<Handle> > out_hi(atoms);

    a->getHandleSet(out_hi, NODE, true);
    logger().fine("---------- Spreading importance for atoms with threshold above %d", spreadThreshold);

    hi = atoms.begin();
    while (hi != atoms.end()) {
        Handle h = *hi;

        current = a->getSTI(h);
        // spread if STI > spread threshold
        if (current > spreadThreshold )
            // spread fraction of importance to nodes it's linked to
            spreadAtomImportance(h);

        hi++;
    }

}

};

