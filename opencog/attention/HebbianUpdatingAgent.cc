/*
 * opencog/attention/HebbianCreationAgent.cc
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
#include <opencog/util/Config.h>

#include <opencog/atoms/base/Link.h>
#include <opencog/truthvalue/IndefiniteTruthValue.h>
#include <opencog/truthvalue/SimpleTruthValue.h>
#include <opencog/attention/atom_types.h>

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/cogserver/server/CogServer.h>

#include "HebbianCreationAgent.h"

using namespace opencog;

HebbianCreationAgent::HebbianCreationAgent(CogServer& cs) :
    Agent(cs)
{
    // Provide a logger, but disable it initially
    log = NULL;
    setLogger(new opencog::Logger("HebbianCreationAgent.log", Logger::FINE, true));
}

HebbianCreationAgent::~HebbianCreationAgent()
{
    if (log) delete log;
}

void HebbianCreationAgent::setLogger(Logger* _log)
{
    if (log) delete log;
    log = _log;
}

Logger* HebbianCreationAgent::getLogger()
{
    return log;
}

void HebbianCreationAgent::run()
{
    a = &_cogserver.getAtomSpace();

    Handle source;
    newAtomsInAV.pop(source);

    //HebbianLinks should not normally enter to the AF boundary since they
    //should not normally have STI values.The below check will avoid such
    //Scenarios from happening which could lead to HebbianLink creation
    //bn atoms containing HebbianLink.
    if (classserver().isA(source->getType(), HEBBIAN_LINK))
        return;

    // Retrieve the atoms in the AttentionalFocus
    HandleSeq attentionalFocus;
    as->get_handle_set_in_attentional_focus(back_inserter(attentionalFocus));

    // Exclude the source atom
    attentionalFocus.erase(std::remove(attentionalFocus.begin(),
                                       attentionalFocus.end(),
                                       source), attentionalFocus.end());

    // Get the neighboring atoms, where the connecting edge
    // is an AsymmetricHebbianLink in either direction
    HandleSeq existingAsSource =
            get_target_neighbors(source, ASYMMETRIC_HEBBIAN_LINK);
    HandleSeq existingAsTarget =
            get_source_neighbors(source, ASYMMETRIC_HEBBIAN_LINK);

    // Get the set differences between the AttentionalFocus
    // and the sets of existing sources and targets
    std::sort(existingAsSource.begin(), existingAsSource.end());
    std::sort(existingAsTarget.begin(), existingAsTarget.end());
    HandleSeq needToBeSource = set_difference(attentionalFocus,
                                              existingAsSource);
    HandleSeq needToBeTarget = set_difference(attentionalFocus,
                                              existingAsTarget);

    // Resulting in the sets of nodes that require
    // a new AsymmetricHebbianLink in either direction
    for (Handle atom : needToBeSource) {
        addHebbian(atom,source);
    }

    for (Handle atom : needToBeTarget) {
        addHebbian(source,atom);
    }
}

void HebbianCreationAgent::addHebbian(Handle atom,Handle source)
{
    float mean = targetConjunction(atom,source);
    Handle link = as->add_link(ASYMMETRIC_HEBBIAN_LINK, atom, source);
    link->setTruthValue(SimpleTruthValue::createTV(mean, 1));
    link->setVLTI(1);
}

float HebbianCreationAgent::targetConjunction(Handle handle1,Handle handle2)
{
  //auto normsti_i = as->get_normalised_STI(handle1,true,true);
  //auto normsti_j = as->get_normalised_STI(handle2,true,true);
  //float conj = (normsti_i * normsti_j);
    auto normsti_i = as->get_normalised_zero_to_one_STI(handle1,true,true);
    auto normsti_j = as->get_normalised_zero_to_one_STI(handle2,true,true);
    float conj = (normsti_i * normsti_j)+ (normsti_j - normsti_i) * std::abs(normsti_j -normsti_i);

    conj = conj / 2.0f;
    conj = (conj + 1.0f) / 2.0f;


    //printf("normsti_i: %f   normsti_j: %f   conj: %f    nconj: %f \n",normsti_i,normsti_j,conj,nconj);

    return conj;
}
