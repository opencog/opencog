/*
 * opencog/attention/MinMaxSTIUpdatingAgent.cc
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

#include <algorithm>
#include <math.h>
#include <time.h>

#include <opencog/util/Config.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/attention/atom_types.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/AttentionBank.h>
#include <opencog/cogserver/server/Agent.h>
#include "MinMaxSTIUpdatingAgent.h"


//#define DEBUG

using namespace opencog;

MinMaxSTIUpdatingAgent::MinMaxSTIUpdatingAgent(CogServer& cs) :
        Agent(cs)
{
    _bank = &attentionbank(_as);
    // Provide a logger
    setLogger(new opencog::Logger("MinMaxSTIUpdatingAgent.log", Logger::FINE, true));
}

void MinMaxSTIUpdatingAgent::run()
{
    HandleSeq atoms;
    _as->get_all_atoms(atoms);

    if (atoms.size() == 0)
        return;

    AttentionValue::sti_t maxSTISeen = AttentionValue::MINSTI;
    AttentionValue::sti_t minSTISeen = AttentionValue::MAXSTI;

    for (const Handle& atom : atoms) {
        AttentionValue::sti_t sti = _bank->get_sti(atom);

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

    _bank->updateMaxSTI(maxSTISeen);
    _bank->updateMinSTI(minSTISeen);
}
