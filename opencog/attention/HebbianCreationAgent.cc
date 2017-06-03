/*
 * opencog/attention/HebbianCreationAgent.cc
 *
 * Written by Roman Treutlein
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/AttentionBank.h>
#include <opencog/cogserver/server/CogServer.h>

#include "HebbianCreationAgent.h"

#ifdef DEBUG
#undef DEBUG
#endif

using namespace opencog;

HebbianCreationAgent::HebbianCreationAgent(CogServer& cs) :
    Agent(cs), _atq(&cs.getAtomSpace())
{
    _bank = &attentionbank(_as);

    // Provide a logger, but disable it initially
    setLogger(new opencog::Logger("HebbianCreationAgent.log", Logger::FINE, true));
}

void HebbianCreationAgent::run()
{
    maxLinkNum = std::stoi(_atq.get_param_value
                           (AttentionParamQuery::heb_maxlink));
    localToFarLinks = std::stoi(_atq.get_param_value
                               (AttentionParamQuery::heb_local_farlink_ratio));

    Handle source;
    newAtomsInAV.pop(source);

    // HebbianLinks should not normally enter to the AF boundary since they
    // should not normally have STI values.The below check will avoid such
    // Scenarios from happening which could lead to HebbianLink creation
    // bn atoms containing HebbianLink.
    if (classserver().isA(source->getType(), HEBBIAN_LINK))
        return;

    HandleSeq notAttentionalFocus;
    // int afb = _bank->getAttentionalFocusBoundary();
    //_bank->get_handles_by_AV(back_inserter(notAttentionalFocus), 0, afb);

    // Retrieve the atoms in the AttentionalFocus
    OrderedHandleSet attentionalFocus;
    _bank->get_handle_set_in_attentional_focus(std::inserter(attentionalFocus,attentionalFocus.begin()));

    // Exclude the source atom
    attentionalFocus.erase(source);

    if (source == Handle::UNDEFINED)
        return;

    // Get the neighboring atoms, where the connecting edge
    // is an AsymmetricHebbianLink in either direction
    HandleSeq existingAsSourceHS =
            get_target_neighbors(source, ASYMMETRIC_HEBBIAN_LINK);
    HandleSeq existingAsTargetHS =
            get_source_neighbors(source, ASYMMETRIC_HEBBIAN_LINK);

    OrderedHandleSet existingAsSource(existingAsSourceHS.begin(),existingAsSourceHS.end());
    OrderedHandleSet existingAsTarget(existingAsTargetHS.begin(),existingAsTargetHS.end());

    // Get the set differences between the AttentionalFocus
    // and the sets of existing sources and targets
    OrderedHandleSet needToBeSource = set_difference(attentionalFocus,
                                              existingAsSource);
    OrderedHandleSet needToBeTarget = set_difference(attentionalFocus,
                                              existingAsTarget);

    int count = 0;

    // Resulting in the sets of nodes that require
    // a new AsymmetricHebbianLink in either direction
    for (const Handle atom : needToBeSource) {
        addHebbian(atom,source);
    }

    for (const Handle atom : needToBeTarget) {
        addHebbian(source,atom);
        count++;
    }

    std::default_random_engine generator;
    if(notAttentionalFocus.size() > 0 ){
        std::uniform_int_distribution<int> distribution(0,(int)notAttentionalFocus.size()-1);

        //How many links outside the AF should be created
        int farLinks = round(count / localToFarLinks);

        //Pick a random target and create the link if it doesn't exist already
        for (int i = 0; i < farLinks; i++) {
            Handle target = notAttentionalFocus[distribution(generator)];
            Handle link = _as->get_handle(ASYMMETRIC_HEBBIAN_LINK, source, target);
            if (link == Handle::UNDEFINED)
                addHebbian(source,target);
        }
    }
    //Check the ammount of HebbianLinks the Atom has
    IncomingSet iset = source->getIncomingSetByType(HEBBIAN_LINK,true);

    //If it is more then the allowed max delete some randomly.
    //TODO: find a simple rule to decide which atoms to forget
    //      that will keep some of the weak links intact
    if (iset.size() >= maxLinkNum) {
        std::uniform_int_distribution<int> distribution2(0,iset.size()-1);
        size_t s = iset.size();
        do {
            _as->remove_atom(iset[distribution2(generator)]->getHandle(),true);
            s--;
        } while (s >= maxLinkNum);
    }
}

void HebbianCreationAgent::addHebbian(Handle source,Handle target)
{
    Handle link = _as->add_link(ASYMMETRIC_HEBBIAN_LINK, source, target);
    link->setTruthValue(SimpleTruthValue::createTV(0.5, 0.1));
    _bank->inc_vlti(link);
}
