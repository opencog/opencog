/*
 * opencog/dynamics/attention/ImportanceSpreadingAgent.cc
 *
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

#include "ImportanceSpreadingAgent.h"

#include <opencog/atomspace/Link.h>
#include <opencog/dynamics/attention/atom_types.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/platform.h>
#include <opencog/util/Config.h>

using namespace opencog;

ImportanceSpreadingAgent::ImportanceSpreadingAgent()
{
    static const std::string defaultConfig[] = {
        "ECAN_DEFAULT_SPREAD_THRESHOLD","0",
        "ECAN_DEFAULT_SPREAD_MULTIPLIER","10.0",
        "", ""
    };
    setParameters(defaultConfig);

    spreadThreshold = (float) (config().get_double
                               ("ECAN_DEFAULT_SPREAD_THRESHOLD"));
}

ImportanceSpreadingAgent::~ImportanceSpreadingAgent()
{
}

void ImportanceSpreadingAgent::run(CogServer* server)
{
    a = server->getAtomSpace();
    spreadImportance();
}


void ImportanceSpreadingAgent::spreadImportance()
{
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

// for a bunch of links from the source, get links passed as pointer
// to avoid retrieving them twice 
int ImportanceSpreadingAgent::sumTotalDifference(Handle source, HandleEntry* links)
{
    HandleEntry *he;
    std::vector<Handle> targets;
    int totalDifference = 0;
    // sum total difference
    he = links;
    while (he) {
        totalDifference += sumDifference(source,he->handle);
        he = he->next;
    }
    return totalDifference;
}

// For one link
int ImportanceSpreadingAgent::sumDifference(Handle source, Handle link)
{
    float linkWeight;
    float linkDifference = 0.0f;
    std::vector<Handle> targets;
    std::vector<Handle>::iterator t;
    AttentionValue::sti_t sourceSTI;
    AttentionValue::sti_t targetSTI;
    
    // If this link doesn't have source as a source return 0
    if (! ((Link*) TLB::getAtom(link))->isSource(source) ) {
        return 0;
    }

    // Get outgoing set and sum difference for all non source atoms
    linkWeight = a->getTV(link).toFloat();
    sourceSTI = a->getSTI(source);
    targets = dynamic_cast<Link*>(TLB::getAtom(link))->getOutgoingSet();


    if (TLB::getAtom(link)->getType() == INVERSE_HEBBIAN_LINK) {
        for (t = targets.begin(); t != targets.end(); t++) {
            Handle target_h = *t;
            if (target_h == source) continue;
            targetSTI = a->getSTI(target_h);

            // pylab code for playing with inverse link stealing schemes:
            // s=10; w=0.5; t= frange(-20,20,0.05x)
            // plot(t,[max(y,0)*s*w for y in log([max(x,0.000001) for x in t+(s*w)])])
            // plot(t,[max(x,0) for x in w*(t+(w*s))])
            // plot(t,[max(min(x,w*s),0) for x in w/2*t+(w*w*s)])
            // ... using this last one.
            linkDifference += calcInverseDifference(sourceSTI, targetSTI, linkWeight);
        }
    } else {
        for (t = targets.begin(); t != targets.end(); t++) {
            Handle target_h = *t;
            if (target_h == source) continue;
            targetSTI = a->getSTI(target_h);
                
            linkDifference += calcDifference(sourceSTI,targetSTI,linkWeight);
        }
    }
    if (TLB::getAtom(link)->getType() == INVERSE_HEBBIAN_LINK) {
        linkDifference *= -1.0f;
    }
    return (int) linkDifference;
}

float ImportanceSpreadingAgent::calcInverseDifference(AttentionValue::sti_t s, AttentionValue::sti_t t, float weight)
{
    float amount;
    amount = weight * t + (weight * weight * s);
    if (amount < 0) amount = 0;
    else if (amount > weight * s) amount = weight * s;
    return amount;
}

float ImportanceSpreadingAgent::calcDifference(AttentionValue::sti_t s, AttentionValue::sti_t t, float weight)
{
    // importance can't spread uphill
    if (s - t > 0) {
        return weight * (s - t);
    }
    return 0.0f;
}

void ImportanceSpreadingAgent::spreadAtomImportance(Handle h)
{
    HandleEntry *links;
    float totalDifference, differenceScaling;
    int totalTransferred;
    AttentionValue::sti_t sourceSTI;

    std::vector<Handle> linksVector;
    std::vector<Handle>::iterator linksVector_i;

    totalDifference = 0.0f;
    differenceScaling = 1.0f;
    totalTransferred = 0;

    logger().fine("+Spreading importance for atom %s", TLB::getAtom(h)->toString().c_str());

    links = TLB::getAtom(h)->getIncomingSet()->clone();
    links = HandleEntry::filterSet(links, HEBBIAN_LINK, true);

    logger().fine("  +Hebbian links found %d", links->getSize());

    totalDifference = static_cast<float>(sumTotalDifference(h, links));
    sourceSTI = a->getSTI(h);

    // if there is no hebbian links with > 0 weight
    // or no lower STI atoms to spread to.
    if (totalDifference == 0.0f) {
        logger().fine("  |totalDifference = 0, spreading nothing");
        delete links;
        return;
    }

    // Find out the scaling factor required on totalDifference
    // to prevent moving the atom below the spreadThreshold
    if (a->getSTI(h) - totalDifference < spreadThreshold) {
        differenceScaling = (a->getSTI(h) - spreadThreshold) / totalDifference;
    }
    logger().fine("  +totaldifference %.2f, scaling %.2f", totalDifference,
            differenceScaling);

    linksVector = links->toHandleVector();

    for (linksVector_i = linksVector.begin();
            linksVector_i != linksVector.end(); linksVector_i++) {
        double transferWeight;
        std::vector<Handle> targets;
        std::vector<Handle>::iterator t;
        Handle lh = *linksVector_i;
        const TruthValue &linkTV = a->getTV(lh);

        // For the case of an asymmetric link without this atom as a source
        if (!dynamic_cast<Link*>(TLB::getAtom(lh))->isSource(h)) {
            //logger().fine("Link %s does not have this atom as a source.",
            // TLB::getAtom(lh)->toString().c_str() );
            continue;
        }

        targets = dynamic_cast<Link*>(TLB::getAtom(lh))->getOutgoingSet();
        transferWeight = linkTV.toFloat();

        logger().fine("  +Link %s", TLB::getAtom(lh)->toString().c_str() );
        logger().fine("    |weight %f, quanta %.2f, size %d", \
                transferWeight, targets.size());

        for (t = targets.begin();
                t != targets.end();
                t++) {
            Handle target_h = *t;
            AttentionValue::sti_t targetSTI;
            double transferAmount;

            // Then for each target of link except source...
            if ( TLB::getAtom(target_h) == TLB::getAtom(h) )
                continue;

            targetSTI = a->getSTI(target_h);

            // calculate amount to transfer, based on difference and scaling
            if (TLB::getAtom(lh)->getType() == INVERSE_HEBBIAN_LINK) {
                // if the link is inverse, then scaling is unnecessary
                // note the negative sign
                transferAmount = -calcInverseDifference(sourceSTI,targetSTI, \
                        linkTV.toFloat());
            } else {
                // Check removing STI doesn't take node out of attentional
                // focus...
                transferAmount = calcInverseDifference(sourceSTI,targetSTI, \
                        linkTV.toFloat()) * differenceScaling;
            }

            a->setSTI( h, a->getSTI(h) - (AttentionValue::sti_t) transferAmount );
            a->setSTI( target_h, a->getSTI(target_h) + (AttentionValue::sti_t) transferAmount );
            logger().fine("    |%d sti from %s to %s", (int) transferAmount, TLB::getAtom(h)->toString().c_str(), TLB::getAtom(target_h)->toString().c_str() );
        }
    }
    delete links;

}
