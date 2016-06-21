/*
 * opencog/attention/HebbianUpdatingAgent.cc
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
#include <math.h>
#include <time.h>

#include <opencog/util/Config.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/attention/atom_types.h>

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/SimpleTruthValue.h>
#include <opencog/atomutils/Neighbors.h>
#include "HebbianUpdatingAgent.h"

//#define DEBUG

using namespace opencog;

HebbianUpdatingAgent::HebbianUpdatingAgent(CogServer& cs) :
        Agent(cs)
{
    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("HebbianUpdatingAgent.log", Logger::FINE,
    true));
}

HebbianUpdatingAgent::~HebbianUpdatingAgent()
{
    if (log)
        delete log;
}

void HebbianUpdatingAgent::setLogger(Logger* _log)
{
    if (log)
        delete log;
    log = _log;
}

Logger* HebbianUpdatingAgent::getLogger()
{
    return log;
}

void HebbianUpdatingAgent::run()
{
    a = &_cogserver.getAtomSpace();

    HandleSeq atoms;
    size_t size;

    std::back_insert_iterator< std::vector<Handle> > out_hi(atoms);

    a->get_handle_set_in_attentional_focus(out_hi);

    size = atoms.size();

    if (size == 0)
        return;

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,size-1);

    Handle source = atoms[distribution(generator)];

    updateHebbianLinks(source);

  //Experimental Coce
  //HandleSeq targetSet = get_target_neighbors(source, ASYMMETRIC_HEBBIAN_LINK);

  //for (Handle target : targetSet)
  //{
  //    Handle link = a->get_handle(ASYMMETRIC_HEBBIAN_LINK, source, target);

  //    float learningRate = 0.01f;
  //    float new_tc, old_tc;

  //    auto magnitued = as->get_normalised_zero_to_one_STI(source,true,true);
  //    auto direction = as->get_normalised_STI(target,true,true);

  //    old_tc = link->getTruthValue()->getMean();

  //    new_tc = new_tc + magnitued * direction * learningRate;
  //    new_tc = std::min(std::max(new_tc,0.0f),1.0f);

  //    //update truth value accordingly
  //    TruthValuePtr newtv = SimpleTruthValue::createTV(new_tc, 0.1);
  //    link->merge(newtv);
  //}
}

void HebbianUpdatingAgent::updateHebbianLinks(Handle source)
{
    float tcDecayRate = 0.1f;
    float tc, old_tc, new_tc;

    IncomingSet links = source->getIncomingSetByType(ASYMMETRIC_HEBBIAN_LINK);

    for (LinkPtr h : links) {
        if (source != h->getOutgoingAtom(0))
            continue;
        HandleSeq outgoing = h->getOutgoingSet();
        new_tc = targetConjunction(outgoing);

        // old link strength decays
        TruthValuePtr oldtv  = h->getTruthValue();
        old_tc = oldtv->getMean();
        tc = tcDecayRate * new_tc + (1.0f - tcDecayRate) * old_tc;

        //if (tc < 0)
        //  printf("old_tc: %f new_tc: %f  tc: %f \n",old_tc,new_tc,tc);

        //update truth value accordingly
        TruthValuePtr newtv = SimpleTruthValue::createTV(tc, 0.01);
        h->merge(newtv);

      //truthvalueptr mergedTV = h->getTruthValue();
      //if (mergedTV->getMean() == 0.5) {
      //    printf(("NewTV: " + newtv->toString() + " ").c_str());
      //    printf(("OldTV: " + oldtv->toString() + " ").c_str());
      //    printf(("MergedTV: " + mergedTV->toString() + "\n").c_str());
      //}
    }
    //h->setTruthValue(SimpleTruthValue::createTV(tc, 1));
}

float HebbianUpdatingAgent::targetConjunction(HandleSeq handles)
{
    if (handles.size() != 2) {
        throw RuntimeException(
                TRACE_INFO,
                "Size of outgoing set of a hebbian link must be 2.");
    }

    auto normsti_i = as->get_normalised_zero_to_one_STI(handles[0],true,true);
    auto normsti_j = as->get_normalised_zero_to_one_STI(handles[1],true,true);
    float conj = (normsti_i * normsti_j) + ((normsti_j - normsti_i) * std::abs(normsti_j -normsti_i));

    conj = (conj + 1.0f) / 2.0f;

    return conj;
}
