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
    float tcDecayRate = 0.1f;
    log->info("HebbianUpdatingAgent::hebbianUpdatingupdate "
              "(convert links = %d)",
              convertLinks);

    // get links again to include the new ones
    HandleSeq links;
    std::back_insert_iterator<HandleSeq> link_output(links);
    a->get_handles_by_type(link_output, HEBBIAN_LINK, true);
    float tc, old_tc, new_tc;

    // for each hebbian link, find targets, work out conjunction and convert
    // that into truthvalue change. the change should be based on existing TV.
    for (Handle h : links) {
        // get out going set
        HandleSeq outgoing = a->get_outgoing(h);
        new_tc = targetConjunction(outgoing);

        // old link strength decays
        old_tc = a->get_mean(h);
        tc = tcDecayRate * new_tc + (1.0f - tcDecayRate) * old_tc;
        if (tc < 0.0f)
            tc = 0.0f;

        //update truth value accordingly
        setMean(h, tc);
        if (new_tc != old_tc) {
            log->fine("HebbianUpdatingAgent: %s old tv %f",
                      a->atom_as_string(h).c_str(), old_tc);
        }
    }
}

float SimpleHebbianUpdatingAgent::targetConjunction(HandleSeq handles)
{
    if (handles.size() != 2) {
        throw RuntimeException(
                TRACE_INFO,
                "Size of outgoing set of a hebbian link must be 2.");
    }

    auto normsti_i = a->get_normalised_STI(handles[0]);
    auto normsti_j = a->get_normalised_STI(handles[1]);
    auto conj = std::max(normsti_i * normsti_j, 1.0f);

    log->fine("HebbianUpdatingAgent: normstis [%.3f,%.3f], tc %.3f", normsti_i,
              normsti_j, conj);

    return conj;
}
