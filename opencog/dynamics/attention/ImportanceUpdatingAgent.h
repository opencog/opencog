/*
 * opencog/dynamics/attention/ImportanceUpdatingAgent.h
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

#ifndef _OPENCOG_IMPORTANCE_UPDATING_AGENT_H
#define _OPENCOG_IMPORTANCE_UPDATING_AGENT_H

#include <string>
#include <iostream>
#include <sstream>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/Agent.h>
#include <opencog/util/Logger.h>
#include <opencog/util/RandGen.h>
#include <opencog/util/recent_val.h>

class ImportanceUpdatingAgentUTest;

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
 * See http://www.opencog.org/wiki/ImportanceUpdatingAgent
 *
 * As atoms are used in mind agents they are endowed with stimulus.
 * Agents should explicit grant atoms stimulus when they make use of them,
 * although it's left up to the Agent to decide how best to do this.
 *
 * The ImportanceUpdatingAgent takes this stimulus and pays wages to atoms. So
 * in a sense, the stimulus is a record of how much work the atom has been
 * involved in. The currency of the wages that the ImportanceUpdatingAgent pays
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
 * ImportanceUpdatingAgent uses to pay wages has to come from somewhere. In an
 * ideal situation, this would be balanced by the rent charged to atoms.
 * However, the number of atoms in the AtomSpace and in the attentional focus
 * will be constantly changing. In fact, an OpenCog instance may even have some
 * method of controlling the attentional focus boundary depending on how
 * focussd or quick thought and reaction needs to be. More atoms in the
 * Attentional Focus mean more atoms to consider during reasoning, although
 * reasoning methods can and do use atoms outside of the attentional focus if
 * those in attention are insufficient to come up with suitable results. To
 * manage the variable number of atoms in attention and in the Atom Space, the
 * ImportanceUpdatingAgent draws STI and LTI from a pool of funds managed by the
 * Atom Space. These pools have a homeostatic range that the
 * ImportanceUpdatingAgent tries to keep the Atom Space funds within. If, at the
 * end of an update cycle, the pools are outside of this range, then the agent
 * taxes all atoms to bring the pools back to the middle of the range (Note that
 * although it's called a tax, it can actually result in a refund if the agent
 * has been charging too much tax... just like real life ;-) ). The agent then
 * recalculates the optimal rent based on decaying measures of the AtomSpace
 * size and number of atoms in the attentional focus.
 *
 * @todo Remove the conversion of stimulus to STI/LTI from
 * ImportanceUpdatingAgent. Create a opencog::MindAgent function that converts stimulus
 * into STI/LTI as well as resetting the stimulus map. This function should be
 * called at the end of the MindAgent run method... but individual MindAgents
 * may choose not to (if they were completely unsuccessful and don't want to
 * give any funds to the atoms).
 */
class ImportanceUpdatingAgent : public Agent
{

    friend class ::ImportanceUpdatingAgentUTest;

public:

    /** The different ways rent can be calculated
     * for atoms in the attentional focus.
     */
    enum rentType_t {
        RENT_FLAT, //!< Use a flat rent
        RENT_EXP, //!< Use an exponential rent
        RENT_LOG //!< Use a logarithmic rent
    };

private:

    AttentionValue::sti_t STIAtomRent; //!< Current atom STI rent.
    opencog::recent_val<AttentionValue::sti_t> STITransitionalAtomRent; //!< Decaying rent
    AttentionValue::lti_t LTIAtomRent; //!< Current atom LTI rent.

    AttentionValue::sti_t amnesty; //!< Amnesty is used in calculating rent.

    enum rentType_t rentType; //!< Current method for calculating rent.

    //! Rent function parameters that tune rent equations.
    std::vector<double> rentFunctionParams;

    /** Calculate the rent to apply to an atom with sti \a c.
     *
     * @param a The AtomSpace to work on.
     * @param c The STI of the atom to calculate rent for.
     */
    AttentionValue::sti_t calculateSTIRent(AtomSpace* a, AttentionValue::sti_t c);

    AttentionValue::sti_t STIAtomWage; //!< Max atom STI wage per stimulus
    AttentionValue::lti_t LTIAtomWage; //!< Max atom LTI wage per stimulus
    std::vector<float> STIAtomWageForAgent; //!< Atom STI wage per stimulus for each Agent
    std::vector<float> LTIAtomWageForAgent; //!< Atom LTI wage per stimulus for each Agent

    /** Calculate the wages to pay to atoms for each agent
     *
     * @param a The atom space.
     * @param agents The list of running agents.
     */
    void calculateAtomWages(AtomSpace *a, const AgentSeq &agents);

    AttentionValue::sti_t STICap; //!< Cap on STI
    AttentionValue::lti_t LTICap; //!< Cap on LTI

    bool updateLinks; //!< Update links or not

    /**
     * Randomly stimulate atoms in the AtomSpace for a given Agent.
     * Simulates a "cognitive process" (e.g. a Novamente lobe)
     * that stimulates Atoms selectively. May also be useful to
     * introduce a level of noise into the system.
     *
     * @param AtomSpace
     * @param Agent to act upon
     */
    void randomStimulation(AtomSpace *a, AgentPtr agent);

    bool noiseOn;     //!< Randomly stimulate atoms?
    float noiseOdds;  //!< Chance of randomly introduced stimulus
    stim_t noiseUnit; //!< The default stimulus unit used by random stimulation

    //! Recent amount of stimulus given per cycle
    opencog::recent_val<stim_t> totalStimulusSinceReset;

    //! Recent number of atoms observed within the AtomSpace attention focus.
    opencog::recent_val<long> attentionalFocusSize;
    //! Recent number of \b nodes observed within the AtomSpace attention focus.
    opencog::recent_val<long> attentionalFocusNodesSize;

    //! Indicates whether STI has gone out of acceptable range during this run.
    bool lobeSTIOutOfBounds;
    //! Indicates whether LTI has gone out of acceptable range during this run.
    bool lobeLTIOutOfBounds;

    /** Collect STI rent for Agents based on processor time they utilize,
     * and pay wages based on how well they acheive system goals.
     *
     * @param a The AtomSpace the Agent is working on.
     * @param agent The Agent to update.
     */
    void updateAgentSTI(AtomSpace* a, AgentPtr agent);

    /** Collect LTI rent for Agents based on processor time they utilize,
     * and pay wages based on how well they acheive system goals.
     *
     * @param a The AtomSpace the Agent is working on.
     * @param agent The Agent to update.
     */
    void updateAgentLTI(AtomSpace* a, AgentPtr agent);

    /** Collect STI rent for atoms within attentional focus
     * and pay wages based on amount of stimulus.
     *
     * @param a The AtomSpace the Agent is working on.
     * @param agents The list of running agents.
     * @param h The Handle of the atom to update.
     */
    void updateAtomSTI(AtomSpace* a, const AgentSeq& agents, Handle h);

    /** Collect LTI rent for all atoms and pay wages based on stimulation
     *
     * @param a The AtomSpace the Agent is working on.
     * @param agents The list of running agents.
     * @param h The Handle of the atom to update.
     */
    void updateAtomLTI(AtomSpace* a, const AgentSeq &agents, Handle h);

    /** Cap STI values to the maximum to prevent atoms
     * becoming all important.
     *
     * @param a The AtomSpace the Agent is working on.
     * @param h The Handle of the atom to update.
     * @return Whether any atoms had an STI above the cap.
     */
    bool enforceSTICap(AtomSpace* a, Handle h);

    /** Cap LTI values to the maximum to prevent atoms
     * becoming all important.
     *
     * @param a The AtomSpace the Agent is working on.
     * @param h The Handle of the atom to update.
     * @return Whether any atoms had an LTI above the cap.
     */
    bool enforceLTICap(AtomSpace* a, Handle h);

    /** Recalculate the STI Rent to charge atoms.
     *
     * @param a The AtomSpace the Agent is working on.
     * @param gradual Change this rent gradually
     */
    void updateSTIRent(AtomSpace* a, bool gradual = false);

	/** Recalculate the LTI Rent to charge atoms.
	 *
     * @param a The AtomSpace the MindAgent is working on.
	 */
    void updateLTIRent(AtomSpace* a);

    /** Check whether AtomSpace funds are within limits, and make changes
     * if not.
     *
     * @param a The AtomSpace to work in.
     * @return Whether the funds were in the homeostatic bounds.
     */
    bool checkAtomSpaceFunds(AtomSpace* a);

    /** Decide whether first value is in the range specified by the 2
     * values in range.
     *
     * @param val value to check.
     * @param range interval to check \a val is in.
     * @return whether val is within range.
     */
    bool inRange(long val, long range[2]) const;

    /** If STI funds are outside of acceptable limits, then the STI funds
     * are adjusted accordingly by applying an AtomSpace wide tax/rebate.
     *
     * @param a The AtomSpace to work in.
     */
    void adjustSTIFunds(AtomSpace* a);

    /** If LTI funds are outside of acceptable limits, then the LTI funds
     * are adjusted accordingly by applying an AtomSpace wide tax/rebate.
     *
     * @param a The AtomSpace to work in
     */
    void adjustLTIFunds(AtomSpace* a);

    /** Get the amount of tax/rebate to apply based on a mean tax/rebate
     * value.
     *
     * When adjusting funds, use the \a mean value to stochastically
     * determine how much rent to charge an atom. (This is because STI/LTI
     * are integers and the tax amount will possibly be < 1 in any reasonable
     * sized OpenCog instance.)
     *
     * Internally takes the integer component of the mean, and samples
     * from a Poisson distribution for the remainder.
     *
     * @param mean The mean tax that would be charged if STI/LTI were a float.
     * @return An integer amount of tax to charge
     */
    int getTaxAmount(double mean);

    /** Get Random number generator associated with Agent,
     * and instantiate if it does not already exist.
     *
     * @return Pointer to the internal random number generator.
     */
    opencog::RandGen* getRandGen();
    opencog::RandGen* rng; //!< Random number generator pointer.

    /** Update the attentional focus size variables needed for
     * tuning attention dynamics.
     *
     * @param a The AtomSpace to work on
     */
    void updateAttentionalFocusSizes(AtomSpace* a);

    /** Initialise iterative variables with suitable starting values.
     */
    void init();

    //! Has init been run to give iterative variables sensible start points
    bool initialEstimateMade;

    /** Update the total stimulus variables.
     *
     * @param agents The list of running Agents
     */
    void updateTotalStimulus(const AgentSeq &agents);

    /** Gets either handles of all Atoms or all Nodes, depending on
     * \a updateLinks.
     *
     * @param a The AtomSpace to work on.
     * @param hs The HandleSeq to add handles to.
     */
    void getHandlesToUpdate(AtomSpace* a, HandleSeq& hs);
    
    void updateRentAndWages(AtomSpace*);

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
        static const ClassInfo _ci("opencog::ImportanceUpdatingAgent");
        return _ci;
    }

    ImportanceUpdatingAgent(CogServer&);
    virtual ~ImportanceUpdatingAgent();
    virtual void run();

    virtual std::string toString();

    /** Return the agent's logger object
     *
     * @return A logger object.
     */
    Logger* getLogger();

    /** Set whether to randomly stimulate atoms.
     *
     * @param newVal flag to generate noise or not.
     */
    void setNoiseFlag(bool newVal);

    // The target lobe STI and LTI values are not only initial values, but
    // also values that are returned to if the values exit their
    // acceptable ranges
    long targetLobeSTI; //!< homeostatic centre for AtomSpace STI funds
    long targetLobeLTI; //!< homeostatic centre for AtomSpace STI funds

    //! the interval to keep AtomSpace STI funds within
    long acceptableLobeSTIRange[2];
    //! the interval to keep AtomSpace LTI funds within
    long acceptableLobeLTIRange[2];

    /** Set whether link atoms should be updated.
     *
     * @param flag flag to update links or not.
     */
    void setUpdateLinksFlag(bool f);

    /** Get whether link atoms should be updated.
     *
     * @return Whether links are updated or not.
     */
    bool getUpdateLinksFlag() const;

    inline AttentionValue::sti_t getSTIAtomWage() const
        { return STIAtomWage; }
    inline AttentionValue::lti_t getLTIAtomWage() const
        { return LTIAtomWage; }

    inline void setRentType(rentType_t t)
        { rentType = t; }
    inline rentType_t getRentType() const
        { return rentType; }

    inline void setAmnesty(AttentionValue::sti_t t)
        { amnesty = t; }
    inline AttentionValue::sti_t getAmnesty() const
        { return amnesty; }

    inline void setRentFunctionParameters(std::vector<double> newParameters)
        { rentFunctionParams = newParameters; }
    inline std::vector<double> getRentFunctionParameters()
        { return rentFunctionParams; }

}; // class

typedef std::shared_ptr<ImportanceUpdatingAgent> ImportanceUpdatingAgentPtr;

/** @}*/
}  // namespace

#endif // _OPENCOG_IMPORTANCE_UPDATING_AGENT_H
