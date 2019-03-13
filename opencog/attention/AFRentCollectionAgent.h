/*
 * opencog/attention/WARentCollectionAgent.h
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

#ifndef _OPENCOG_AFRENT_COLLECTION_AGENT_H
#define _OPENCOG_AFRENT_COLLECTION_AGENT_H

#include <chrono>
#include <string>
#include <iostream>
#include <sstream>

#include <opencog/util/RandGen.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/avalue/AttentionValue.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/Agent.h>

#include "RentCollectionBaseAgent.h"

using namespace std::chrono;

namespace opencog {
    /** \addtogroup grp_attention
     *  @{
     */

    /**
     * This Agent collects wages form inside the attentional focus by iterating
     * through each atoms in the attentional focus and collects the Wage which
     * is calculate depending on the current funds in the Bank. The wage is
     * computed as a linear function form the Funds and a Target Value.
     * It is capped to the range 0-2x default Wage.
     *
     * This Agent is supposed to run in it's own Thread.
     */
    class AFRentCollectionAgent : public RentCollectionBaseAgent {
        private:
            time_point<high_resolution_clock> last_update;
            float update_freq;

        public:

            virtual const ClassInfo& classinfo() const {
                return info();
            }

            static const ClassInfo& info() {
                static const ClassInfo _ci("opencog::AFRentCollectionAgent");
                return _ci;
            }

            AFRentCollectionAgent(CogServer&);
            virtual ~AFRentCollectionAgent();
            virtual void selectTargets(HandleSeq &targetSetOut);
            void collectRent(HandleSeq& targetSet);
    }; // class

    /** @}*/
} // namespace

#endif // _OPENCOG_AFRENT_COLLECTION_AGENT_H
