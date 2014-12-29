/*
 * opencog/dynamics/attention/ImportanceSpreadingAgent.cc
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

#include "ImportanceSpreadingAgent.h"

#include <opencog/atomspace/Link.h>
#include <opencog/dynamics/attention/atom_types.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/util/platform.h>
#include <opencog/util/foreach.h>

using namespace opencog;

ImportanceSpreadingAgent::ImportanceSpreadingAgent(CogServer& cs) :
    Agent(cs)
{
    static const std::string defaultConfig[] = {
        "ECAN_DEFAULT_SPREAD_THRESHOLD","0",
        "ECAN_DEFAULT_SPREAD_MULTIPLIER","10.0",
        "ECAN_ALL_LINKS_SPREAD","false",
        "", ""
    };
    setParameters(defaultConfig);

    spreadThreshold = (float) (config().get_double
                               ("ECAN_DEFAULT_SPREAD_THRESHOLD"));
    allLinksSpread = config().get_bool("ECAN_ALL_LINKS_SPREAD");

    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("ImportanceSpreadingAgent.log", Logger::FINE, true));
}

ImportanceSpreadingAgent::~ImportanceSpreadingAgent()
{
}

void ImportanceSpreadingAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

Logger* ImportanceSpreadingAgent::getLogger()
{
    return log;
}

void ImportanceSpreadingAgent::run()
{
    a = &_cogserver.getAtomSpace();
    spreadImportance();
}


void ImportanceSpreadingAgent::spreadImportance()
{
    AttentionValue::sti_t current;

    std::vector<Handle> atoms;
    std::vector<Handle>::iterator hi;
    std::back_insert_iterator< std::vector<Handle> > out_hi(atoms);

    a->getHandlesByType(out_hi, NODE, true);
    log->fine("---------- Spreading importance for atoms with threshold above %d", spreadThreshold);

    hi = atoms.begin();
    while (hi != atoms.end()) {
        Handle h = *hi;

        current = a->getSTI(h);
        // spread if STI > spread threshold
        if (current > spreadThreshold )
            // spread fraction of importance to nodes it's linked to
            spreadAtomImportance(h);

        ++hi;
    }
}

// for a bunch of links from the source, get links passed as pointer
// to avoid retrieving them twice 
int ImportanceSpreadingAgent::sumTotalDifference(Handle source, HandleSeq& links)
{
    int totalDifference = 0;
    // sum total difference
    foreach(Handle handle, links) {
        totalDifference += sumDifference(source,handle);
    }
    return totalDifference;
}

#define toFloat getMean
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
    if (! a->isSource(source,link)) {
        log->debug("Skipping link because link doesn't have this source as a source: " + std::to_string(link.value()));
        return 0;
    }

    // Get outgoing set and sum difference for all non source atoms
    linkWeight = a->getTV(link)->toFloat();
    sourceSTI = a->getSTI(source);
    targets = a->getOutgoing(link);

    if (a->getType(link) == INVERSE_HEBBIAN_LINK) {
        for (t = targets.begin(); t != targets.end(); ++t) {
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
        for (t = targets.begin(); t != targets.end(); ++t) {
            Handle target_h = *t;

            if (target_h == source) {
                log->debug("Skipping link because link has source as target: " + std::to_string(link.value()));
                continue;
            }

            log->fine("Target atom %s", a->atomAsString(target_h, false).c_str());

            targetSTI = a->getSTI(target_h);  // why is it 0?
                
            linkDifference += calcDifference(sourceSTI,targetSTI,linkWeight);
        }
    }
    if (a->getType(link) == INVERSE_HEBBIAN_LINK) {
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

bool ImportanceSpreadingAgent::IsHebbianLink::operator()(Handle h) {
    if (classserver().isA(a->getType(h),HEBBIAN_LINK)) return true;
    return false;
}

void ImportanceSpreadingAgent::spreadAtomImportance(Handle h)
{
    HandleSeq links;
    float totalDifference, differenceScaling;
    AttentionValue::sti_t sourceSTI;

    std::vector<Handle> linksVector;
    std::vector<Handle>::iterator linksVector_i;

    totalDifference = 0.0f;
    differenceScaling = 1.0f;

    log->fine("+Spreading importance for atom %s", a->atomAsString(h, false).c_str());

    linksVector = a->getIncoming(h);
    IsHebbianLink isHLPred(a);
    if (allLinksSpread) {
        log->fine("  +Spreading across all links. Found %d", linksVector.size());
    } else {
      linksVector.erase(std::remove_if(linksVector.begin(),linksVector.end(),isHLPred), linksVector.end());
      log->fine("  +Hebbian links found %d", linksVector.size());
    }

    totalDifference = static_cast<float>(sumTotalDifference(h, linksVector));
    sourceSTI = a->getSTI(h);

    // if there is no hebbian links with > 0 weight
    // or no lower STI atoms to spread to.
    if (totalDifference == 0.0f) {
        log->fine("  |totalDifference = 0, spreading nothing");
        return;
    }

    // Find out the scaling factor required on totalDifference
    // to prevent moving the atom below the spreadThreshold
    if (a->getSTI(h) - totalDifference < spreadThreshold) {
        differenceScaling = (a->getSTI(h) - spreadThreshold) / totalDifference;
    }
    log->fine("  +totaldifference %.2f, scaling %.2f", totalDifference,
            differenceScaling);

    for (linksVector_i = linksVector.begin();
            linksVector_i != linksVector.end(); ++linksVector_i) {
        double transferWeight;
        std::vector<Handle> targets;
        std::vector<Handle>::iterator t;
        Handle lh = *linksVector_i;
        TruthValuePtr linkTV = a->getTV(lh);

        // For the case of an asymmetric link without this atom as a source
        if (!a->isSource(h,lh)) {
            log->fine("Skipping link due to assymetric link without this atom as a source: " + h.value());
            continue;
        }

        targets = a->getOutgoing(lh);
        transferWeight = linkTV->toFloat();

        log->fine("  +Link %s", a->atomAsString(lh).c_str() );
        //log->fine("    |weight %f, quanta %.2f, size %d",
        log->fine("    |weight %f, size %d", \
                transferWeight, targets.size());

        for (t = targets.begin();
                t != targets.end();
                ++t) {
            Handle target_h = *t;
            AttentionValue::sti_t targetSTI;
            double transferAmount;

            // Then for each target of link except source...
            if ( target_h == h ) continue;

            targetSTI = a->getSTI(target_h);

            // calculate amount to transfer, based on difference and scaling
            if (a->getType(lh) == INVERSE_HEBBIAN_LINK) {
                // if the link is inverse, then scaling is unnecessary
                // note the negative sign
                transferAmount = -calcInverseDifference(sourceSTI,targetSTI, \
                        linkTV->toFloat());
            } else {
                // Check removing STI doesn't take node out of attentional
                // focus...
                transferAmount = calcInverseDifference(sourceSTI,targetSTI, \
                        linkTV->toFloat()) * differenceScaling;
            }

            a->setSTI( h, a->getSTI(h) - (AttentionValue::sti_t) transferAmount );
            a->setSTI( target_h, a->getSTI(target_h) + (AttentionValue::sti_t) transferAmount );
            log->fine("    |%d sti from %s to %s", (int) transferAmount,
                    a->atomAsString(h).c_str(), a->atomAsString(target_h).c_str() );
        }
    }

}
