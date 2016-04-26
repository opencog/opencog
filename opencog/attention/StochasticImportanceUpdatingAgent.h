/*
 * opencog/attention/StochasticImportanceUpdatingAgent.h
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

#ifndef _OPENCOG_STOCHASTIC_IMPORTANCE_UPDATING_AGENT_H
#define _OPENCOG_STOCHASTIC_IMPORTANCE_UPDATING_AGENT_H

#include <string>
#include <iostream>
#include <sstream>

#include <opencog/util/Logger.h>
#include <opencog/util/RandGen.h>
#include <opencog/util/recent_val.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/AttentionValue.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/Agent.h>

class StochasticImportanceUpdatingAgentUTest;

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

class CogServer;

/** ImportantUpdatingAgent updates the AttentionValues of atoms.
 *
 * This Agent carries out:
 *   - stimulus to STI and LTI conversion
 *   - rent collection
 *   - rent adjustment and taxing when AtomSpace funds go out of
 *     homeostatic bounds.
 *
 * See http://www.opencog.org/wiki/StochasticImportanceUpdatingAgent
 *
 * As atoms are used in mind agents they are endowed with stimulus.
 * Agents should explicit grant atoms stimulus when they make use of them,
 * although it's left up to the Agent to decide how best to do this.
 *
 * The StochasticImportanceUpdatingAgent takes this stimulus and pays wages to atoms. So
 * in a sense, the stimulus is a record of how much work the atom has been
 * involved in. The currency of the wages that the StochasticImportanceUpdatingAgent pays
 * to an atom is in the form of both STI and LTI. The amount of STI and LTI
 * conferred is based both on the total stimulus of the atom, and two internal
 * multipliers that indicate the rate that stimulus is converted into STI and
 * LTI. If an Agent has insufficient funds to pay at these rates, the Agent's
 * available funds is divided proportionately among that Atoms it has
 * stimulated.  As such, the global multipliers for stimulus act as a "cap", so
 * Agents need not go broke providing stimulus. Atoms also have to pay LTI rent
 * to exist in the the AtomSpace and STI rent to exist in the Attentional focus.
 *
 * STI rent is charged from atoms that are above the Attentional Focus boundary
 * and LTI rent for all atoms in the system (conceptually at least, once atoms
 * start being swapped to disk, LTI rent for these atoms might only be charged
 * periodically due to the expense involved in updating them). So, if the
 * attentional focus boundary was set at zero, i.e. all atoms with positive STI
 * are in the attention span of the OpenCog instance, then for our atom A1, it
 * would be charged both STI and LTI rent. If STI was < 0 then atom A1 would not
 * be charged STI rent, but would still be charged LTI rent regardless of the
 * STI or LTI of A1.
 *
 * Since both STI and LTI currency are conserved, the funds that the
 * StochasticImportanceUpdatingAgent uses to pay wages has to come from somewhere. In an
 * ideal situation, this would be balanced by the rent charged to atoms.
 * However, the number of atoms in the AtomSpace and in the attentional focus
 * will be constantly changing. In fact, an OpenCog instance may even have some
 * method of controlling the attentional focus boundary depending on how
 * focussd or quick thought and reaction needs to be. More atoms in the
 * Attentional Focus mean more atoms to consider during reasoning, although
 * reasoning methods can and do use atoms outside of the attentional focus if
 * those in attention are insufficient to come up with suitable results. To
 * manage the variable number of atoms in attention and in the Atom Space, the
 * StochasticImportanceUpdatingAgent draws STI and LTI from a pool of funds managed by the
 * Atom Space. These pools have a homeostatic range that the
 * StochasticImportanceUpdatingAgent tries to keep the Atom Space funds within. If, at the
 * end of an update cycle, the pools are outside of this range, then the agent
 * taxes all atoms to bring the pools back to the middle of the range (Note that
 * although it's called a tax, it can actually result in a refund if the agent
 * has been charging too much tax... just like real life ;-) ). The agent then
 * recalculates the optimal rent based on decaying measures of the AtomSpace
 * size and number of atoms in the attentional focus.
 *
 * @todo Remove the conversion of stimulus to STI/LTI from
 * StochasticImportanceUpdatingAgent. Create a opencog::MindAgent function that converts stimulus
 * into STI/LTI as well as resetting the stimulus map. This function should be
 * called at the end of the MindAgent run method... but individual MindAgents
 * may choose not to (if they were completely unsuccessful and don't want to
 * give any funds to the atoms).
 */
class StochasticImportanceUpdatingAgent : public Agent
{

public:

private:

    AtomSpace* a;

    AttentionValue::sti_t STIAtomRent; //!< Current atom STI rent.
    AttentionValue::lti_t LTIAtomRent; //!< Current atom LTI rent.

    AttentionValue::sti_t stiFundsBuffer;
    AttentionValue::lti_t ltiFundsBuffer;

    /** Set the agent's logger object
     *
     * Note, this will be deleted when this agent is.
     *
     * @param l The logger to associate with the agent.
     */
    void setLogger(Logger* l);

    Logger *log; //!< Logger object for Agent

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::StochasticImportanceUpdatingAgent");
        return _ci;
    }

    StochasticImportanceUpdatingAgent(CogServer&);
    virtual ~StochasticImportanceUpdatingAgent();
    virtual void run();

    /** Return the agent's logger object
     *
     * @return A logger object.
     */
    Logger* getLogger();

    // The target lobe STI and LTI values are not only initial values, but
    // also values that are returned to if the values exit their
    // acceptable ranges
    long targetSTI; //!< homeostatic centre for AtomSpace STI funds
    long targetLTI; //!< homeostatic centre for AtomSpace STI funds
    long acceptableLobeLTIRange[2];

    int calculate_STI_Rent();
    int calculate_LTI_Rent();

}; // class

typedef std::shared_ptr<StochasticImportanceUpdatingAgent> StochasticImportanceUpdatingAgentPtr;

/** @}*/
}  // namespace

#endif // _OPENCOG_IMPORTANCE_UPDATING_AGENT_H
