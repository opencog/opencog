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

#include <opencog/attention/atom_types.h>

#include "SimpleHebbianUpdatingAgent.h"

using namespace opencog;

SimpleHebbianUpdatingAgent::SimpleHebbianUpdatingAgent(CogServer& cs) :
        HebbianUpdatingAgent(cs)
{

}

SimpleHebbianUpdatingAgent::~SimpleHebbianUpdatingAgent()
{
}

void SimpleHebbianUpdatingAgent::hebbianUpdatingUpdate()
{
    // tc affects the truthvalue
    double tcDecayRate = 0.1;
    _log->info("HebbianUpdatingAgent::hebbianUpdatingupdate "
              "(convert links = %d)",
              convertLinks);

    // get links again to include the new ones
    HandleSeq links;
    std::back_insert_iterator<HandleSeq> link_output(links);
    _as->get_handles_by_type(link_output, HEBBIAN_LINK, true);
    double tc, old_tc, new_tc;

    // for each hebbian link, find targets, work out conjunction and convert
    // that into truthvalue change. the change should be based on existing TV.
    for (const Handle& h : links) {
        // get out going set
        HandleSeq outgoing = h->getOutgoingSet();
        new_tc = targetConjunction(outgoing);

        // old link strength decays
        old_tc = h->getTruthValue()->getMean();
        tc = tcDecayRate * new_tc + (1.0 - tcDecayRate) * old_tc;
        if (tc < 0.0)
            tc = 0.0;

        //update truth value accordingly
        setMean(h, tc);
        if (new_tc != old_tc) {
            _log->fine("HebbianUpdatingAgent: %s old tv %f",
                       h->toString().c_str(), old_tc);
        }
    }
}

double SimpleHebbianUpdatingAgent::targetConjunction(HandleSeq handles)
{
    if (handles.size() != 2) {
        throw RuntimeException(
                TRACE_INFO,
                "Size of outgoing set of a hebbian link must be 2.");
    }

    double normsti_i = _as->get_normalised_STI(handles[0]);
    double normsti_j = _as->get_normalised_STI(handles[1]);
    double conj = std::max(normsti_i * normsti_j, 1.0);

    _log->fine("HebbianUpdatingAgent: normstis [%.3f,%.3f], tc %.3f", normsti_i,
              normsti_j, conj);

    return conj;
}
