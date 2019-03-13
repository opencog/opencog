/*
 * opencog/attention/RentCollectionAgent.h
 *
 * Written by Misgana Bayetta
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

#ifndef RENTCOLLECTIONBASE_H
#define RENTCOLLECTIONBASE_H

#include <string>
#include <iostream>
#include <sstream>

#include <opencog/util/RandGen.h>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/attentionbank/bank/AttentionBank.h>

#include "AttentionParamQuery.h"

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

/**
 * This Agent collects wages form inside the AttentionalFocus
 *
 * It randomly picks an atom from the Focus and collects the Wage
 * which is calculate depending on the current funds in the Bank
 *
 * The wage is computed as a linar function form the Funds and a Target Value.
 * It is capped to the range 0-2x defaul Wage
 *
 * This Agent is supposed to run in it's own Thread.
 */
class RentCollectionBaseAgent : public Agent
{
protected:
    AttentionBank* _bank;
    AttentionParamQuery _atq;

    AttentionValue::sti_t STIAtomRent; //!< Current atom STI rent.
    AttentionValue::lti_t LTIAtomRent; //!< Current atom LTI rent.

    AttentionValue::sti_t targetSTI;
    AttentionValue::sti_t targetLTI;

    AttentionValue::sti_t stiFundsBuffer;
    AttentionValue::lti_t ltiFundsBuffer;

    void load_params(void);

public:
    RentCollectionBaseAgent(CogServer& cs);

    double calculate_STI_Rent();
    double calculate_LTI_Rent();

    virtual void selectTargets(HandleSeq &targetSetOut) = 0;
    virtual void collectRent(HandleSeq& targetSet) = 0;

    void run();
}; // class

/** @}*/
}  // namespace

#endif /* RENTCOLLECTIONBASE_H */
