/*
 * opencog/dynamics/attention/HebbianUpdatingAgent.cc
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
#include "HebbianUpdatingAgent.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/truthvalue/IndefiniteTruthValue.h>
#include <opencog/truthvalue/SimpleTruthValue.h>
#include <opencog/dynamics/attention/atom_types.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Config.h>

using namespace opencog;

void HebbianUpdatingAgent::setMean(Handle h, float tc)
{
	TruthValuePtr oldtv(h->getTruthValue());
	switch (oldtv->getType())
	{
		case SIMPLE_TRUTH_VALUE: {
			TruthValuePtr newtv(SimpleTruthValue::createTV(tc, oldtv->getCount()));
			h->merge(newtv);
			break;
		}
		case INDEFINITE_TRUTH_VALUE: {
			IndefiniteTruthValuePtr newtv(IndefiniteTruthValue::createITV(oldtv));
			newtv->setMean(tc);
			h->merge(newtv);
			break;
		}
		default:
			throw InvalidParamException(TRACE_INFO, "Unsupported TV type in Hebbian");
	}
}

HebbianUpdatingAgent::HebbianUpdatingAgent(CogServer& cs) :
    Agent(cs)
{
    convertLinks = config().get_bool("ECAN_CONVERT_LINKS");
    conversionThreshold = config().get_int("ECAN_CONVERSION_THRESHOLD");

    // Provide a logger, but disable it initially
    log = NULL;
    setLogger(new opencog::Logger("HebbianUpdatingAgent.log", Logger::FINE, true));
}

HebbianUpdatingAgent::~HebbianUpdatingAgent()
{
    if (log) delete log;
}

void HebbianUpdatingAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

Logger* HebbianUpdatingAgent::getLogger()
{
    return log;
}

void HebbianUpdatingAgent::run()
{
    a = &_cogserver.getAtomSpace();
    hebbianUpdatingUpdate();
}

void HebbianUpdatingAgent::hebbianUpdatingUpdate()
{

    // tc affects the truthvalue
    float tc, old_tc, new_tc;
    float tcDecayRate = 0.1f;

    log->info("HebbianUpdatingAgent::hebbianUpdatingupdate "
                   "(convert links = %d)", convertLinks);

    // get links again to include the new ones
    HandleSeq links;
    std::back_insert_iterator<HandleSeq> link_output(links);
    a->get_handles_by_type(link_output, HEBBIAN_LINK, true);

    for(Handle h:links){
        // for each hebbian link, find targets, work out conjunction and convert
        // that into truthvalue change. the change should be based on existing TV.
		bool isDifferent = false;

        // get out going set
        HandleSeq outgoing = a->get_outgoing(h);
        new_tc = targetConjunction(outgoing);
        // old link strength decays
        old_tc = a->get_mean(h);
		if (new_tc != old_tc) isDifferent = true;

        if (convertLinks && a->get_LTI(h) < conversionThreshold) {
            // If mind agent is set to convert hebbian links then
            // inverse and symmetric links will convert between one
            // another when conjunction between sti values is correct
            // (initially for hopfield emulation, but could
            // be useful in other cases)
            if (a->get_type(h) == INVERSE_HEBBIAN_LINK) {
                // if inverse normalised sti of source is negative
                // (i.e. hebbian link is pointing in the wrong direction)
                if (a->get_normalised_STI(outgoing[0],true,false) < 0.0f) {
                    tc = (tcDecayRate * new_tc) + ( (1.0f - tcDecayRate) * old_tc);
                    if (tc < 0) {
                        // link no longer representative
                        // swap inverse hebbian link direction
                        log->fine("HebbianUpdatingAgent: swapping direction of inverse link %s", a->atom_as_string(h).c_str());
                        // save STI/LTI
                        AttentionValuePtr backupAV = a->get_AV(h);
                        a->remove_atom(h);
                        outgoing = moveSourceToFront(outgoing);
                        h = a->add_link(INVERSE_HEBBIAN_LINK, outgoing);
                        h->setTruthValue(SimpleTruthValue::createTV(-tc, 0));
                        // restore STI/LTI
                        a->set_AV(h,backupAV);
                    }
                // else source is +ve, as it should be
                } else {
                    // Work out whether to convert to symmetric, if not, just
                    // update weight
                    tc = (tcDecayRate * -new_tc) + ( (1.0f - tcDecayRate) * old_tc);

                    if (tc < 0) {
                        // Inverse link no longer representative
                        // change to symmetric hebbian link
                        log->fine("HebbianUpdatingAgent: change old inverse %s to sym link", a->atom_as_string(h).c_str());
                        // save STI/LTI
                        AttentionValuePtr backupAV = a->get_AV(h);
                        a->remove_atom(h);
                        h = a->add_link(SYMMETRIC_HEBBIAN_LINK, outgoing);
                        h->setTruthValue(SimpleTruthValue::createTV(-tc, 1));
                        // restore STI/LTI
                        a->set_AV(h,backupAV);
                    } else {
                        // link type is fine, just update TV
                        setMean(h, tc);
                    }
                }
            } else {
                tc = (tcDecayRate * new_tc) + ( (1.0f - tcDecayRate) * old_tc);
                if (tc < 0) {
                    // link no longer representative
                    // change to inverse hebbian link
                    log->fine("HebbianUpdatingAgent: change old sym %s to inverse link", a->atom_as_string(h).c_str());
                    // save STI/LTI
                    AttentionValuePtr backupAV = a->get_AV(h);
                    a->remove_atom(h);
                    outgoing = moveSourceToFront(outgoing);
                    h = a->add_link(INVERSE_HEBBIAN_LINK, outgoing);
                    h->setTruthValue(SimpleTruthValue::createTV(-tc, 0));
                    // restore STI/LTI
                    a->set_AV(h,backupAV);
                } else {
                    // link type is fine, just update TV
                    setMean(h, tc);
                }
            }

        } else {
            // otherwise just update link weights
            // if inverse normalised sti of source is positive
            // (i.e. hebbian link is pointing in the right direction)
            if (a->get_type(h) == INVERSE_HEBBIAN_LINK &&
                    a->get_normalised_STI(outgoing[0],true,false) < 0.0f) {
				new_tc = -new_tc;
            }
            tc = (tcDecayRate * new_tc) + ( (1.0f - tcDecayRate) * old_tc);
            if (tc < 0.0f) tc = 0.0f;
            setMean(h, tc);
        }
		if (isDifferent)
			log->fine("HebbianUpdatingAgent: %s old tv %f", a->atom_as_string(h).c_str(), old_tc);

    }
    // if not enough links, try and create some more either randomly
    // or by looking at what nodes are in attentional focus
}


HandleSeq& HebbianUpdatingAgent::moveSourceToFront(HandleSeq &outgoing)
{
    // find source, atom with positive norm_sti, and make first in outgoing
    Handle theSource;
    bool foundTheSource = false;
    for (auto outgoing_i = outgoing.begin();outgoing_i != outgoing.end();) {
        float normsti;
        Handle oh = *outgoing_i;
        normsti = a->get_normalised_STI(oh,true,false);
        if (normsti > 0.0f) {
            theSource = oh;
            foundTheSource = true;
            outgoing_i = outgoing.erase(outgoing_i);
        } else
            ++outgoing_i;
    }
    if (foundTheSource) {
        outgoing.insert(outgoing.begin(), theSource);
    } else {
        log->error("Can't find source atom for new Asymmetric Hebbian Link");
    }
    return outgoing;

}
float HebbianUpdatingAgent::targetConjunction(HandleSeq handles)
{
    // TODO: this won't work for Hebbian Links with arity > 2

    // indicates whether at least one of the source/target are
    // in the attentional focus
    bool inAttention = false;
    AttentionValue::sti_t sti;
    float tc = 0.0f, normsti;
    std::vector<float> normsti_v;
    bool tcInit = true;

	log->fine("HebbianUpdatingAgent::targetConjunction");

    for (Handle h: handles) {
        sti = a->get_STI(h);

        // if none in attention return 0 at end
        if (sti > a->get_attentional_focus_boundary()) {
            log->fine("HebbianUpdatingAgent: handle %d STI value %d in attention, focus boundary = %d", 
                      h.value(),
                      sti,
                      a->get_attentional_focus_boundary());
            inAttention = true;
		}

        // normalise each sti and multiply each
        normsti = a->get_normalised_STI(h,true,false);

        // For debugging:
        normsti_v.push_back(normsti);

        if (tcInit) {
            tc = normsti;
            tcInit = false;
        } else tc *= normsti;
		log->fine("HebbianUpdatingAgent: normsti %.3f, tc %.3f", normsti, tc);

    }

    if (!inAttention) return 0.0f;


    // cap conjunction to range [-1,1]
    if (tc > 1.0f) tc = 1.0f;
    if (tc < -1.0f) tc = -1.0f;

    log->fine("HebbianUpdatingAgent: normstis [%.3f,%.3f], tc = %.3f", normsti_v[0], normsti_v[1], tc);

    return tc;

}
