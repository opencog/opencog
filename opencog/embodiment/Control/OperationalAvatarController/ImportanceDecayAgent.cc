/*
 * opencog/embodiment/Control/OperationalAvatarController/ImportanceDecayAgent.cc
 *
 * Copyright (C) 2009-2011 OpenCog Foundation
 * Copyright (C) 2002-2009 Novamente LLC
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


#include "OAC.h"
#include "ImportanceDecayAgent.h"

using namespace opencog::oac;

ImportanceDecayAgent::~ImportanceDecayAgent()
{
    mergedAtomConnection.disconnect();
}

ImportanceDecayAgent::ImportanceDecayAgent(CogServer& cs) : Agent(cs)
{
    lastTickTime = 0;
    AtomSpace& as = _cogserver.getAtomSpace();
    mergedAtomConnection = as.AVChangedSignal(
            boost::bind(&ImportanceDecayAgent::atomMerged, this, _1, _2, _3));
}

void ImportanceDecayAgent::run()
{
    logger().fine("ImportanceDecayTask - Executing decayShortTermImportance().");
    dynamic_cast<OAC *>(&_cogserver)->decayShortTermImportance();
}

void ImportanceDecayAgent::atomMerged(const Handle& h,
                                      const AttentionValuePtr& old_av,
                                      const AttentionValuePtr& new_av)
{
    logger().fine("ImportanceDecayAgent::atomMerged(%lu)", h.value());
    // Restore the default STI value if it has decayed
    // TODO: Remove this code when the merge of atoms consider the STI values
    // this way as well.
    if (new_av->getSTI() < AttentionValue::DEFAULTATOMSTI) {
        _cogserver.getAtomSpace().setSTI(h, AttentionValue::DEFAULTATOMSTI);
    }
}
