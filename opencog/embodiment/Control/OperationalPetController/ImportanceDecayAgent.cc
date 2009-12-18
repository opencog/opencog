/*
 * opencog/embodiment/Control/OperationalPetController/ImportanceDecayAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#include "OPC.h"
#include "ImportanceDecayAgent.h"

using namespace OperationalPetController;

ImportanceDecayAgent::~ImportanceDecayAgent()
{
}

ImportanceDecayAgent::ImportanceDecayAgent()
{
    lastTickTime = 0;
    mergedAtomConnection.disconnect();
}

void ImportanceDecayAgent::connectSignals(AtomSpace& as)
{
    mergedAtomConnection = as.mergeAtomSignal().connect(boost::bind(&ImportanceDecayAgent::atomMerged, this, _1));
}

void ImportanceDecayAgent::run(opencog::CogServer *server)
{

    logger().fine(
                 "ImportanceDecayTask - Executing decayShortTermImportance().");
    ((OPC *) server)->decayShortTermImportance();

}

void ImportanceDecayAgent::atomMerged(Handle h)
{
    logger().debug("ImportanceDecayAgent::atomMerged(%lu)", h.value());
    AtomSpace* atomSpace = server().getAtomSpace();
    // Restore the default STI value if it has decayed
    // TODO: Remove this code when the merge of atoms consider the STI values this way as well.
    if (atomSpace->getSTI(h) < AttentionValue::DEFAULTATOMSTI) {
        atomSpace->setSTI(h, AttentionValue::DEFAULTATOMSTI);
    }
}
