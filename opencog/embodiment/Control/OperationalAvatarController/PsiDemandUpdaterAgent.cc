/*
 * @file opencog/embodiment/Control/OperationalPetController/DemandUpdaterAgent.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2010-10-25
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
#include "DemandUpdaterAgent.h"

using namespace OperationalPetController;

DemandUpdaterAgent::~DemandUpdaterAgent()
{
}

DemandUpdaterAgent::DemandUpdaterAgent()
{
    lastTickTime = 0;
    mergedAtomConnection.disconnect();
}

void DemandUpdaterAgent::connectSignals(AtomSpace& as)
{
    mergedAtomConnection = as.mergeAtomSignal().connect(boost::bind(&DemandUpdaterAgent::atomMerged, this, _1));
}

void DemandUpdaterAgent::run(opencog::CogServer *server)
{

    logger().fine(
                 "ImportanceDecayTask - Executing decayShortTermImportance().");
    ((OPC *) server)->decayShortTermImportance();

}

void DemandUpdaterAgent::atomMerged(Handle h)
{
    logger().debug("DemandUpdaterAgent::atomMerged(%lu)", h.value());
    AtomSpace* atomSpace = server().getAtomSpace();
    // Restore the default STI value if it has decayed
    // TODO: Remove this code when the merge of atoms consider the STI values this way as well.
    if (atomSpace->getSTI(h) < AttentionValue::DEFAULTATOMSTI) {
        atomSpace->setSTI(h, AttentionValue::DEFAULTATOMSTI);
    }
}
