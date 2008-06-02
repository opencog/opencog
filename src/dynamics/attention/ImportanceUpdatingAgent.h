/*
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

/**
 * ImportantUpdatingAgent.h
 */

#ifndef IMPORTANCEUPDATINGAGENT_H
#define IMPORTANCEUPDATINGAGENT_H

#include <string>
#include <iostream>
#include <sstream>
#include <AttentionValue.h>

#include <AtomSpace.h>
#include <RandGen.h>
#include <recent_val.h>
#include <Logger.h>
#include "CogServer.h"
#include "MindAgent.h"

/* Starting values for rent and wage */
#define DEFAULT_ATOM_STI_RENT 10
#define DEFAULT_ATOM_LTI_RENT 10
#define DEFAULT_ATOM_STI_WAGE 2
#define DEFAULT_ATOM_LTI_WAGE 2

namespace opencog
{

class CogServer;

class ImportanceUpdatingAgent : public MindAgent
{

    friend class ImportanceUpdatingAgentUTest;

private:

    /* Atom Rent */
    AttentionValue::sti_t STIAtomRent;
    AttentionValue::lti_t LTIAtomRent;

    /* Atom wages (to be weighted by stimulus) */
    AttentionValue::sti_t STIAtomWage;
    AttentionValue::lti_t LTIAtomWage;

    /* Cap on STI/LTI */
    AttentionValue::sti_t STICap;
    AttentionValue::lti_t LTICap;

    /* Update links or not */
    bool updateLinks;

    /* Logger object for MindAgent */
    Logger *log;

    /* Randomly stimulate atoms? */
    bool noiseOn;
    /* Change of randomly introduced stimulation */
    float noiseOdds;
    /* The default stimulus unit, used by random stimulation */
    stim_t noiseUnit;

    /**
     * Randomly stimulate atoms in the AtomSpace
     * Simulates a "cognitive process" (e.g. a Novamente lobe)
     * that stimulates Atoms selectively. May also be useful to
     * introduce a level of noise into the system.
     *
     * @param AtomSpace to act upon
     */
    void randomStimulation(AtomSpace *a);

    /* Recent amount of stimulus given per cycle */
    opencog::recent_val<stim_t> totalStimulusSinceReset;

    /* Number of atoms within attentionFocusBoundary */
    opencog::recent_val<long> attentionalFocusSize;
    opencog::recent_val<long> attentionalFocusNodesSize;
    /* Rate of decay (r) for estimating AttentionalFocusSize
     * Estimate equal to:
     * r *(attentionalFocusSize + (1-r) attentionalFocusSize.recent */

    /* for calculate the recent maximum STI value, used in Hebbian learning */
    float maxSTIDecayRate;
    //AttentionValue::sti_t recentMaxSTI;

    /* STI has gone out of aceceptable range during this mindagent cycle */
    bool lobeSTIOutOfBounds;

    /**
     * Collect STI rent for atoms above attentional focus boundary
     * and pay wages based on stimulation.
     *
     * @param AtomSpace the MindAgent is working on.
     */
    void updateAtomSTI(AtomSpace* a, Handle h);

    /**
     * Collect LTI rent for atoms above attentional focus boundary
     * and pay wages based on stimulation
     *
     * @param AtomSpace the MindAgent is working on.
     */
    void updateAtomLTI(AtomSpace* a, Handle h);

    /**
     * Cap STI values to the maximum to prevent all important atoms.
     *
     * @param AtomSpace the MindAgent is working on.
     * @return Whether any atoms had an STI above the cap.
     */
    bool enforceSTICap(AtomSpace* a, Handle h);

    /**
     * Cap LTI values to the maximum to prevent all important atoms.
     *
     * @param AtomSpace the MindAgent is working on.
     * @return Whether any atoms had an STI above the cap.
     */
    bool enforceLTICap(AtomSpace* a, Handle h);


    void updateSTIRent(AtomSpace* a);

    /**
     * Check whether AtomSpace funds are within limits, and make changes
     * if not.
     *
     * @param AtomSpace to work in.
     */
    void checkAtomSpaceFunds(AtomSpace* a);

    /**
     * Decide whether first value is in the range specified by the 2
     * values in range.
     *
     * @param value to check
     * @param range to compare with
     * @return whether value is within range
     */
    bool inRange(long val, long range[2]) const;

    /**
     * If STI funds are outside of acceptable limits, then adjust
     * rent.
     *
     * @param AtomSpace to work in
     */
    void adjustSTIFunds(AtomSpace* a);

    /**
     * If LTI funds are outside of acceptable limits, then adjust
     * rent.
     *
     * @param AtomSpace to work in
     */
    void adjustLTIFunds(AtomSpace* a);

    /**
     * When adjusting funds, use the mean value to stochastically
     * determine how much rent to charge an atom. (This is because sti/lti
     * are integers and the tax amount will likely be < 1 in any reasonable
     * sized opencog instance.
     *
     * Internally samples from a Poisson distribution.
     *
     * @param The tax that would be charged if sti/lti were a float.
     * @return an integer amount of tax to charge
     */
    int getTaxAmount(double mean);

    opencog::RandGen* rng;
    /**
     * Get Random number generator associated with MindAgent.
     */
    opencog::RandGen* getRandGen();

    /**
     * Update the attentional focus size variables
     * needed for tuning attention dynamics.
     *
     * @param the AtomSpace to work on
     */
    void updateAttentionalFocusSizes(AtomSpace* a);

    /* Has init been run to give iterative variables sensible start points */
    bool initialEstimateMade;

    /**
     * Initialise iterative variables with suitable starting values.
     *
     * @param server is a pointer to the cogserver
     */
    void init(CogServer *server);

    /**
     * Update the total stimulus variables.
     *
     * @param the AtomSpace to work on
     */
    void updateTotalStimulus(AtomSpace* a);

    HandleEntry* getHandlesToUpdate(AtomSpace* a);

    /* Debug */
    bool verbose;

public:

    ImportanceUpdatingAgent();

    virtual ~ImportanceUpdatingAgent();

    virtual void run(CogServer *server);

    virtual string toString();

    Logger* getLogger();
    void setLogger(Logger* l);

    /**
     * Set whether to randomly stimulate atoms.
     *
     * @param flag
     */
    void setNoiseFlag(bool newVal);

    /**
     * Set whether link atoms should be updated.
     *
     * @param flag
     */
    void setUpdateLinks(bool flag) {
        updateLinks = flag;
    }

    /* The target lobe STI and LTI values are not only initial values, but
     * also values that are returned to if the values exit their
     * acceptable ranges */
    long targetLobeSTI;
    long targetLobeLTI;
    long acceptableLobeSTIRange[2];
    long acceptableLobeLTIRange[2];

    void setUpdateLinksFlag(bool f);
    bool getUpdateLinksFlag();

    // Moved to Atomspace
    // AttentionValue::sti_t getRecentMaxSTI();

}; // class

}  // namespace

#endif // IMPORTANCEUPDATINGAGENT_H
