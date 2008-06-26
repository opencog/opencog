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

#ifndef _IMPORTANCE_SPREADING_AGENT_H
#define _IMPORTANCE_SPREADING_AGENT_H

#include <string>
#include <math.h>

#include <Logger.h>

#include <AtomSpace.h>
#include <MindAgent.h>
#include <AttentionValue.h>

namespace opencog
{

const int MA_DEFAULT_SPREAD_THRESHOLD = 0;
const float MA_DEFAULT_SPREAD_MULTIPLIER = 10.0f;

class CogServer;

/** Spreads short term importance along HebbianLinks.
 *
 * Currently spread along Symmetric and Inverse HebbianLinks.
 *
 * @todo Spread along asymmetric hebbian links too.
 * @todo Optionally spread long term importance.
 */
class ImportanceSpreadingAgent : public MindAgent
{

private:
    AtomSpace* a;

	//! Minimal amount of STI necessary for an atom to have before it spreads
	//! STI.
    AttentionValue::sti_t spreadThreshold;

	//! How much to multiply the HebbianLink TruthValue to convert to STI.
    float importanceSpreadingMultiplier;

	//! The 
    AttentionValue::sti_t stealingLimit;

	/** Spread importance for an atom.
	 *
	 * @param h The handle for the atom to spread importance for.
	 */
    void spreadAtomImportance(Handle h);

	//! Total amount spread during recent runs.
    opencog::recent_val<long> amountSpread;

    //! Spread importance along Hebbian links.
    void spreadImportance();

	//! Sum total difference for an atom
	int sumTotalDifference(Handle source, HandleEntry* links);

	//! Sum difference for one link
	int sumDifference(Handle source, Handle link);

	//! Calculate the difference for an inverse link
	float calcInverseDifference(AttentionValue::sti_t s, AttentionValue::sti_t t, \
			float weight);

	//! Calculate the difference for a normal Hebbian link
	float calcDifference(AttentionValue::sti_t s, AttentionValue::sti_t t, \
			float weight);

public:

    ImportanceSpreadingAgent();
    virtual ~ImportanceSpreadingAgent();
    virtual void run(CogServer *server);

	/** Set minimal amount of STI necessary for an atom to have before it
	 * spreads STI.
	 *
	 * @param t the threshold.
	 */
    void setSpreadThreshold(AttentionValue::sti_t t) {
        spreadThreshold = t;
    };

	/** Set the multiplier for converting the HebbianLink TruthValue to STI.
	 *
	 * If multiplier is zero, then \b all STI above the threshold is evenly
	 * distributed.
	 *
	 * @param m the multiplier.
	 */
    void setImportanceSpreadingMultiplier(float m) {
        importanceSpreadingMultiplier = m;
    };

}; // class

struct ImportanceSpreadSTISort {
    bool operator()(const Handle& h1, const Handle& h2) {
        return TLB::getAtom(h1)->getAttentionValue().getSTI() > TLB::getAtom(h2)->getAttentionValue().getSTI();
    }
};

struct ImportanceSpreadLTIAndTVAscendingSort {
    bool operator()(const Handle& h1, const Handle& h2) {
        AttentionValue::lti_t lti1, lti2;
        float tv1, tv2;

        tv1 = fabs(TLB::getAtom(h1)->getTruthValue().getMean());
        tv2 = fabs(TLB::getAtom(h2)->getTruthValue().getMean());

        lti1 = TLB::getAtom(h1)->getAttentionValue().getLTI();
        lti2 = TLB::getAtom(h2)->getAttentionValue().getLTI();

        if (lti1 < 0)
            tv1 = lti1 * (1.0f - tv1);
        else
            tv1 = lti1 * tv1;

        if (lti2 < 0)
            tv2 = lti2 * (1.0f - tv2);
        else
            tv2 = lti2 * tv2;


        return tv1 < tv2;
    }

};

struct ImportanceSpreadLTIThenTVAscendingSort {
    bool operator()(const Handle& h1, const Handle& h2) {
        AttentionValue::lti_t lti1, lti2;
        float tv1, tv2;

        lti1 = TLB::getAtom(h1)->getAttentionValue().getLTI();
        lti2 = TLB::getAtom(h2)->getAttentionValue().getLTI();

        tv1 = TLB::getAtom(h1)->getTruthValue().getMean();
        tv2 = TLB::getAtom(h2)->getTruthValue().getMean();

        if (lti1 != lti2) return lti1 < lti2;

        else return tv1 < tv2;
    }

};

}; // namespace
#endif // _IMPORTANCE_SPREADING_AGENT_H
