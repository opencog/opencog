/*
* opencog/dynamics/attention/HebbianCreationModule.cc
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

#include "HebbianCreationModule.h"

#include <opencog/util/algorithm.h>

#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>
#include <opencog/util/Config.h>
#include <opencog/dynamics/attention/atom_types.h>
#include <opencog/atomspace/SimpleTruthValue.h>

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
 * Create a separate thread to handle the AddAFSignal event to avoid blocking
 */
void HebbianCreationModule::addAFSignal(const Handle& source,
                                        const AttentionValuePtr& av_old,
                                        const AttentionValuePtr& av_new)
{
    std::thread handler(&HebbianCreationModule::addAFSignalHandler,
                        this, source, av_old, av_new);
    handler.detach();
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
    // Retrieve the atoms in the AttentionalFocus
    HandleSeq attentionalFocus;
    as->getHandleSetInAttentionalFocus(back_inserter(attentionalFocus));

    // Exclude the source atom
    attentionalFocus.erase(std::remove(attentionalFocus.begin(),
                                       attentionalFocus.end(),
                                       source), attentionalFocus.end());

    // Get the neighboring atoms, where the connecting edge
    // is an AsymmetricHebbianLink in either direction
    HandleSeq existingAsSource =
            as->getNeighbors(source, false, true, ASYMMETRIC_HEBBIAN_LINK, false);
    HandleSeq existingAsTarget =
            as->getNeighbors(source, true, false, ASYMMETRIC_HEBBIAN_LINK, false);

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
    foreach (Handle atom, needToBeSource) {
        as->addLink(ASYMMETRIC_HEBBIAN_LINK, atom, source,
                    SimpleTruthValue::createTV(0, 1));
    }

    foreach (Handle atom, needToBeTarget) {
        as->addLink(ASYMMETRIC_HEBBIAN_LINK, source, atom,
                    SimpleTruthValue::createTV(0, 1));
    }
}
