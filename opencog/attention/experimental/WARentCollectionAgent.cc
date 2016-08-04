/*
 * opencog/attention/RentCollectionAgent.cc
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

#include "WARentCollectionAgent.h"

//#define DEBUG

using namespace opencog;

WARentCollectionAgent::WARentCollectionAgent(CogServer& cs) : RentCollectionBaseAgent(cs)
{
    _tournamentSize = config().get_int("ECAN_RENT_TOURNAMENT_SIZE", 5);

    // READ SLEEPING TIME HERE
    set_sleep_time(2000);
}

Handle WARentCollectionAgent::tournamentSelect(HandleSeq population){

    Handle tournament[_tournamentSize];
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,population.size()-1);

    for(int i = 0; i < _tournamentSize; i++){
        int idx = distribution(generator);
        tournament[i] = population[idx];
    }

    auto result = std::max_element(tournament, tournament + (_tournamentSize - 1), []
                              (const Handle& h1, const Handle & h2)
    {
        return (h1->getSTI() > h2->getSTI());
    });

    return *result;
}


void WARentCollectionAgent::selectTargets(HandleSeq &targetSetOut)
{
    HandleSeq atoms;

    _as->get_all_atoms(atoms);

    if (atoms.size() == 0) return;

    Handle h = tournamentSelect(atoms);
    targetSetOut.push_back(h);
}
