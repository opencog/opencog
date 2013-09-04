/*
 * opencog/dynamics/attention/ForgettingAgent.h
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

#ifndef _OPENCOG_FORGETTING_AGENT_H
#define _OPENCOG_FORGETTING_AGENT_H

#include <string>

#include <math.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/server/Agent.h>
#include <opencog/util/Logger.h>

namespace opencog
{

class CogServer;

/** The ForgettingAgent, carries out the forgetting process in OpenCog Prime. 
 * 
 * It does based on the LTI of Atoms. Low LTI indicates that an atom has not been
 * of any use for a long time, and additionally, isn't near any other important
 * atoms. The latter condition is because the ImportanceSpreadingAgent would
 * otherwise increase the STI of the atom, by moving STI from nearby important
 * atoms, and increase the likelihood of the atom in question being:
 *
 * 1. used in mind processes, and thus
 * 2. rewarded with stimulus which later gets exchanged for STI. 
 *
 * The ForgettingAgent also takes into account the VLTI of an atom. This is a
 * boolean value that indicates whether the atom in fact is allowed to be
 * forgotten. This allows a mechanism for ensuring that highly important
 * information will never be forgotten, even if it's used only very very rarely.
 *
 * Forgetting can be tuned via two parameters:
 *
 * 1. the maximum LTI value that can be forgotten, and
 * 2. the percentage of the AtomSpace to forget (typically this would be very low!) 
 *
 * These work in concert to limit how much and what atoms are forgotten. If only
 * one parameter is set, then the other has free reign. I.e. a certain percentage
 * of the AtomSpace will always be forgotten regardless of their LTI, or, any atom
 * that drops below the maximum forgetting LTI will be forgotten. 
 */
class ForgettingAgent : public Agent
{

private:
    AtomSpace* a;
    Logger *log; //!< Logger object for Agent

    /** Set the agent's logger object
     *
     * Note, this will be deleted when this agent is.
     *
     * @param l The logger to associate with the agent.
     */
    void setLogger(Logger* l);

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::ForgettingAgent");
        return _ci;
    }

    //! Maximum LTI of an atom that can be forgot.
    AttentionValue::lti_t forgetThreshold;
    //! Percentage of AtomSpace to forget.
    float forgetPercentage;

    ForgettingAgent(CogServer&);
    virtual ~ForgettingAgent();
    virtual void run();

    void forget(float p);

    /** Return the agent's logger object
     *
     * @return A logger object.
     */
    Logger* getLogger();

}; // class

/**
 * Comparison operator for using qsort on a list of Handles.
 * Returns them with ascending LTI and if equal in LTI,
 * then sorted ascending by TruthValue.
 */
struct ForgettingLTIThenTVAscendingSort {

    AtomSpace* a;

    ForgettingLTIThenTVAscendingSort(AtomSpace* _a): a(_a) {};

    bool operator()(const Handle& h1, const Handle& h2) {
        AttentionValue::lti_t lti1, lti2;
        float tv1, tv2;

        lti1 = a->getAV(h1).getLTI();
        lti2 = a->getAV(h2).getLTI();

        tv1 = fabs(a->getMean(h1));
        tv2 = fabs(a->getMean(h2));

        if (lti1 != lti2) return lti1 < lti2;

        else return tv1 < tv2;
    }

};

} // namespace

#endif // _OPENCOG_FORGETTING_AGENT_H
