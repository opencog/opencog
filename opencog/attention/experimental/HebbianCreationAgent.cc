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
#include <opencog/util/algorithm.h>

#include <opencog/atoms/base/Link.h>
#include <opencog/truthvalue/IndefiniteTruthValue.h>
#include <opencog/truthvalue/SimpleTruthValue.h>
#include <opencog/attention/atom_types.h>
#include <opencog/atomutils/Neighbors.h>

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

    maxLinkNum = 300;
    localToFarLinks = 5;

    as = &_cogserver.getAtomSpace();
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
    Handle source;
    newAtomsInAV.pop(source);

    //HebbianLinks should not normally enter to the AF boundary since they
    //should not normally have STI values.The below check will avoid such
    //Scenarios from happening which could lead to HebbianLink creation
    //bn atoms containing HebbianLink.
    if (classserver().isA(source->getType(), HEBBIAN_LINK))
        return;

    HandleSeq notAttentionalFocus;
    int afb = as->get_attentional_focus_boundary();
    as->get_handles_by_AV(back_inserter(notAttentionalFocus),0,afb);

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
    std::sort(attentionalFocus.begin(), attentionalFocus.end());
    HandleSeq needToBeSource = set_difference(attentionalFocus,
                                              existingAsSource);
    HandleSeq needToBeTarget = set_difference(attentionalFocus,
                                              existingAsTarget);


    int count = 0;

    // Resulting in the sets of nodes that require
    // a new AsymmetricHebbianLink in either direction
    for (Handle atom : needToBeSource) {
        if (atom == Handle::UNDEFINED)
            continue;
        if (source == Handle::UNDEFINED)
            return;
        addHebbian(atom,source);
        count++;
    }

    for (Handle atom : needToBeTarget) {
        if (atom == Handle::UNDEFINED)
            continue;
        if (source == Handle::UNDEFINED)
            return;
        addHebbian(source,atom);
    }

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,notAttentionalFocus.size()-1);

    //How many links outside the AF should be created
    int farLinks = count > localToFarLinks ? count / localToFarLinks : 1;

    Handle link = Handle::UNDEFINED;
    Handle target = Handle::UNDEFINED;

    //Pick a random target and create the link if it doesn't exist already
    for (int i = 0; i < farLinks; i++) {
        target = notAttentionalFocus[distribution(generator)];
        link = as->get_handle(ASYMMETRIC_HEBBIAN_LINK, source, target);
        if (link == Handle::UNDEFINED)
            addHebbian(source,target);
    }

    //Check the ammount of HebbianLinks the Atom has
    IncomingSet iset = source->getIncomingSet();
    iset.erase(remove_if(iset.begin(),iset.end(),
                            [](LinkPtr lp) -> bool {
                            return !(classserver().isA(lp->getType(), HEBBIAN_LINK));
                            }),iset.end());

    //If it is more then the allowed max delete some randomly.
    //TODO: find a simple rule to decide which atoms to forget
    //      that will keep some of the weak links intact
    if (iset.size() >= maxLinkNum) {
        std::uniform_int_distribution<int> distribution2(0,iset.size()-1);
        size_t s = iset.size();
        do {
            as->remove_atom(iset[distribution2(generator)]->getHandle(),true);
            s--;
        } while (s >= maxLinkNum);
    }
}

void HebbianCreationAgent::addHebbian(Handle atom,Handle source)
{
    Handle link = as->add_link(ASYMMETRIC_HEBBIAN_LINK, atom, source);
    link->setTruthValue(SimpleTruthValue::createTV(0.5, 0.1));
    link->setVLTI(1);
}
