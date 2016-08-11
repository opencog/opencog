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

#ifndef _OPENCOG_WARENT_COLLECTION_AGENT_H
#define _OPENCOG_WARENT_COLLECTION_AGENT_H

#include <string>
#include <iostream>
#include <sstream>

#include <opencog/util/RandGen.h>
#include <opencog/cogserver/server/Agent.h>

#include "RentCollectionBaseAgent.h"

namespace opencog {
    /** \addtogroup grp_attention
     *  @{
     */

    /**
     * This Agent collects wages form inside the Whole Atomsapce by picking a
     * atoms via sti biased random selection from the whole atomspace and
     * collects the Wage which is calculate depending on the current funds
     * in the Bank. The wage is computed as a linear function form the Funds
     * and a Target Value. It is capped to the range 0-2x default Wage.
     *
     * This Agent is supposed to run in it's own Thread.
     */
    class WARentCollectionAgent : public RentCollectionBaseAgent
    {
    private:
        unsigned int SAMPLE_SIZE = 5;
        unsigned int _tournamentSize;
        Handle tournamentSelect(HandleSeq population);

    public:
        const ClassInfo& classinfo() const { return info(); }

        static const ClassInfo& info() {
            static const ClassInfo _ci("opencog::WARentCollectionAgent");
            return _ci;
        }

        WARentCollectionAgent(CogServer&);
        void selectTargets(HandleSeq &targetSetOut);
    }; // class

    /** @}*/
} // namespace

#endif // _OPENCOG_WARENT_COLLECTION_AGENT_H
