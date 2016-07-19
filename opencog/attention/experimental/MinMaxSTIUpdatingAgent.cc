/*
 * opencog/attention/MinMaxSTIUpdatingAgent.cc
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
#include "MinMaxSTIUpdatingAgent.h"

#include <opencog/cogserver/server/Agent.h>

//#define DEBUG

using namespace opencog;

MinMaxSTIUpdatingAgent::MinMaxSTIUpdatingAgent(CogServer& cs) :
        Agent(cs)
{
    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("MinMaxSTIUpdatingAgent.log", Logger::FINE,
    true));
}

MinMaxSTIUpdatingAgent::~MinMaxSTIUpdatingAgent()
{
    if (log)
        delete log;
}

void MinMaxSTIUpdatingAgent::setLogger(Logger* _log)
{
    if (log)
        delete log;
    log = _log;
}

Logger* MinMaxSTIUpdatingAgent::getLogger()
{
    return log;
}

void MinMaxSTIUpdatingAgent::run()
{
    a = &_cogserver.getAtomSpace();

    HandleSeq atoms;
    a->get_all_atoms(atoms);

    if (atoms.size() == 0)
        return;

    AttentionValue::sti_t maxSTISeen = AttentionValue::MINSTI;
    AttentionValue::sti_t minSTISeen = AttentionValue::MAXSTI;

    AttentionValue::sti_t sti;

    for (Handle atom : atoms) {
        sti = atom->getAttentionValue()->getSTI();

        if (sti > maxSTISeen) {
            maxSTISeen = sti;
        }

        if (sti < maxSTISeen) {
            minSTISeen = sti;
        }
    }

    if (minSTISeen > maxSTISeen) {
        minSTISeen = maxSTISeen;
    }

    a->update_max_STI(maxSTISeen);
    a->update_min_STI(minSTISeen);
}
