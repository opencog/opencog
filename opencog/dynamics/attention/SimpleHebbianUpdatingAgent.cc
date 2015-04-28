/*
 * SimpleHebbianUpdatingAgent.cc
 *
 * Copyright (C) 2015 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Misgana Bayeta <misgana.bayetta@gmail.com>
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

#include "SimpleHebbianUpdatingAgent.h"

#include <opencog/dynamics/attention/atom_types.h>

SimpleHebbianUpdatingAgent::SimpleHebbianUpdatingAgent(CogServer& cs):HebbianUpdatingAgent(cs) {

}

SimpleHebbianUpdatingAgent::~SimpleHebbianUpdatingAgent() {
}

void SimpleHebbianUpdatingAgent::hebbianUpdatingUpdate(){
    // tc affects the truthvalue
    float tcDecayRate = 0.1f;
    log->info("HebbianUpdatingAgent::hebbianUpdatingupdate "
                   "(convert links = %d)", convertLinks);
    // get links again to include the new ones
    HandleSeq links;
    std::back_insert_iterator<HandleSeq> link_output(links);
    a->getHandlesByType(link_output, HEBBIAN_LINK, true);
    float tc, old_tc, new_tc;
    for(Handle h:links){
        // for each hebbian link, find targets, work out conjunction and convert
        // that into truthvalue change. the change should be based on existing TV.
        // get out going set
        HandleSeq outgoing = a->getOutgoing(h);
        new_tc = targetConjunction(outgoing);
        // old link strength decays
        old_tc = a->getMean(h);
        tc = tcDecayRate * new_tc + (1.0f - tcDecayRate) * old_tc;
        if (tc < 0.0f) tc = 0.0f;
        //update truth value accordingly
        setMean(h, tc);
        if (new_tc != old_tc)
        	log->fine("HebbianUpdatingAgent: %s old tv %f", a->atomAsString(h).c_str(), old_tc);
    }
}

float SimpleHebbianUpdatingAgent::targetConjunction(HandleSeq handles)
{
    if(handles.size() > 2) throw RuntimeException(TRACE_INFO,"Don't know how to handle arity greater than two");
    std::vector<float> normsti_v;
    auto af =  a->getAttentionalFocusBoundary();
    log->fine("HebbianUpdatingAgent::targetConjunction");
    AttentionValue::sti_t normi[2];
    for (HandleSeq::size_type i = 0 ; i < handles.size();i++) {
        auto si = a->getSTI(handles[i]);
        normi[i]=si > af ? si/a->getMaxSTI(false): si/a->getMinSTI(false);
    }
    log->fine("HebbianUpdatingAgent: normstis [%.3f,%.3f]", normi[0],normi[1]);
    auto conj =normi[0]*normi[1];
    return (conj>1.0f ? conj: 1.0f);

}
