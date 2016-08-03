/*
 * opencog/attention/HebbianCreationModule.cc
 *
 * Copyright (C) 2014 Cosmo Harrigan
 * All Rights Reserved
 *
 * Written by Cosmo Harrigan
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


#include <tbb/task.h>

#include <opencog/util/algorithm.h>
#include <opencog/util/Config.h>
#include <opencog/util/tbb.h>

#include <opencog/atomutils/Neighbors.h>
#include <opencog/truthvalue/SimpleTruthValue.h>

#include <opencog/attention/atom_types.h>

#include <opencog/atomspace/AtomSpace.h>

#include <opencog/cogserver/server/CogServer.h>

#include "HebbianCreationModule.h"

using namespace std;
using namespace opencog;

DECLARE_MODULE(HebbianCreationModule)

HebbianCreationModule::HebbianCreationModule(CogServer& cs) : Module(cs)
{
    logger().info("[HebbianCreationModule] constructor");
    this->as = &cs.getAtomSpace();
    addAFConnection = as->AddAFSignal(
               boost::bind(&HebbianCreationModule::addAFSignal,
                           this, _1, _2, _3));
}

void HebbianCreationModule::init(void)
{
   logger().info("Initializing HebbianCreationModule.");
}

void HebbianCreationModule::run()
{
}

HebbianCreationModule::~HebbianCreationModule()
{
   logger().info("Terminating HebbianCreationModule.");
   addAFConnection.disconnect();
}

/*
 * Create a task to handle the AddAFSignal event to avoid blocking
 */
void HebbianCreationModule::addAFSignal(const Handle& source,
                                        const AttentionValuePtr& av_old,
                                        const AttentionValuePtr& av_new)
{
    tbb_enqueue_lambda([=] {
        addAFSignalHandler(source, av_old, av_new); 
    });
}

/*
 * When an atom enters the AttentionalFocus, a search is done to
 * identify to which other atoms in the AttentionalFocus there is not already
 * a pair of AsymmetricHebbianLinks connecting them, and those links are
 * created for later updating by the HebbianUpdatingAgent.
 */
void HebbianCreationModule::addAFSignalHandler(const Handle& source,
                                        const AttentionValuePtr& av_old,
                                        const AttentionValuePtr& av_new)
{
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
        as->add_link(ASYMMETRIC_HEBBIAN_LINK, atom, source)->setTruthValue(
                     SimpleTruthValue::createTV(0, 1));
    }

    for (Handle atom : needToBeTarget) {
        as->add_link(ASYMMETRIC_HEBBIAN_LINK, source, atom)->setTruthValue(
                     SimpleTruthValue::createTV(0, 1));
    }
}
