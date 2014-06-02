/*
 * opencog/dynamics/attention/SimpleSimpleImportanceDiffusionAgent.h
 *
 * Copyright (C) 2014 Cosmo Harrigan
 * All Rights Reserved
 *
 * Written by Cosmo Harrigan
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

#ifndef _OPENCOG_SIMPLE_IMPORTANCE_DIFFUSION_AGENT_H
#define _OPENCOG_SIMPLE_IMPORTANCE_DIFFUSION_AGENT_H

#include <string>
#include <math.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/server/Agent.h>
#include <opencog/util/Logger.h>
#include <opencog/util/RandGen.h>
#include "SpreadDecider.h"

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

class CogServer;

/** Spreads short term importance along HebbianLinks using a diffusion approach.
 *
 * Spreads along:
 * \arg SymmetricHebbianLinks
 * \arg InverseHebbianLinks
 * \arg AsymmetricHebbianLink
 * \arg SymmetricInverseHebbianLink
 *
 * @todo Optionally spread long term importance?
 */
class SimpleImportanceDiffusionAgent : public Agent
{

private:
    AtomSpace* a;

    //! Total amount spread during recent runs.
    opencog::recent_val<long> amountSpread;

    //! Maximum percentage of importance to spread
    float maxSpreadPercentage;

    //! Value that normalised STI has to be above before being spread
    //! Is a normalised value from -1 to 1. 0 == AF
    float diffusionThreshold;

    //! Whether to spread STI across all types of Links and not just HebbianLinks.
    //! If there are multiple links between the same two Atoms, then it will add up their strengths.
    bool allLinksSpread;

    //! Spread importance along Hebbian links.
    //! @todo split into sub functions instead of one giant beast.
    void spreadImportance();

    SpreadDecider* spreadDecider;

    //! For checking that STI is conserved
    int totalSTI;

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
        static const ClassInfo _ci("opencog::SimpleImportanceDiffusionAgent");
        return _ci;
    }

    enum { HYPERBOLIC, STEP };
    void setSpreadDecider(int type, float shape = 30);

    SimpleImportanceDiffusionAgent(CogServer&);
    virtual ~SimpleImportanceDiffusionAgent();
    virtual void run();

    /** Return the agent's logger object
     *
     * @return A logger object.
     */
    Logger* getLogger();

    /** Set the maximum percentage of importance that can be spread.
     * @param p the maximum percentage of importance that can be spread.
     */
    void setMaxSpreadPercentage(float p);

    /** Get the maximum percentage of importance that can be spread.
     * @return the maximum percentage of importance that can be spread.
     */
    float getMaxSpreadPercentage() const;

    void setDiffusionThreshold(float p);
    float getDiffusionThreshold() const;
}; // class

typedef std::shared_ptr<SimpleImportanceDiffusionAgent> SimpleImportanceDiffusionAgentPtr;

/** @}*/
} // namespace

#endif // _OPENCOG_SIMPLE_IMPORTANCE_DIFFUSION_AGENT_H
