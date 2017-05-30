/*
 * opencog/attention/FocusBoundaryUpdatingAgent.h
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

#ifndef _OPENCOG_FOCUS_BOUNDARY_UPDATING_AGENT_H
#define _OPENCOG_FOCUS_BOUNDARY_UPDATING_AGENT_H

#include <string>
#include <iostream>
#include <sstream>

#include <opencog/util/RandGen.h>

#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/Agent.h>

#include "AttentionParamQuery.h"

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

class AttentionBank;
/**
 * This agent updates the Boundary of the AttentionalFocus to equal the top 25%
 * of the STI Range.
 * The Boundary will never drop Below 100 STI.
 * TODO: Implement and upper limit to the number of Atoms in the Focus.
 */
class FocusBoundaryUpdatingAgent : public Agent
{
private:
    AttentionBank* _bank;
    AttentionParamQuery _atq;

    double afbSize;
    double decay;
    AttentionValue::sti_t bottomBoundary;
    unsigned int minAFSize, maxAFSize;

    AttentionValue::sti_t get_cutoff(HandleSeq& );

public:
    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::FocusBoundaryUpdatingAgent");
        return _ci;
    }

    FocusBoundaryUpdatingAgent(CogServer&);
    virtual void run();

}; // class

typedef std::shared_ptr<FocusBoundaryUpdatingAgent> FocusBoundaryUpdatingAgentPtr;

/** @}*/
}  // namespace

#endif // _OPENCOG_FOCUS_BOUNDARY_UPDATING_AGENT_H
