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

#ifndef _FORGETTING_AGENT_H
#define _FORGETTING_AGENT_H

#include <string>
#include <math.h>

#include <Logger.h>
#include <AtomSpace.h>
#include <MindAgent.h>
#include <AttentionValue.h>

namespace opencog {

class CogServer;

/**
 * Forgetting agent, removes atoms with low LTI.
 */
class ForgettingAgent : public MindAgent
{

    private:
	AtomSpace* a;

    public:
	// Maximum LTI of a link that can be forgot.
	AttentionValue::lti_t forgetThreshold;
	// percentage to forget
	float forgetPercentage;

	ForgettingAgent();
	virtual ~ForgettingAgent();
	virtual void run(CogServer *server);

	void forget(float p);

}; // class

struct ForgettingLTIThenTVAscendingSort
{
    bool operator()(const Handle& h1, const Handle& h2)
    {
        AttentionValue::lti_t lti1, lti2;
        float tv1, tv2;

        lti1 = TLB::getAtom(h1)->getAttentionValue().getLTI();
        lti2 = TLB::getAtom(h2)->getAttentionValue().getLTI();

        tv1 = fabs(TLB::getAtom(h1)->getTruthValue().getMean());
        tv2 = fabs(TLB::getAtom(h2)->getTruthValue().getMean());

        if (lti1 != lti2) return lti1 < lti2;

        else return tv1 < tv2;
    }

};


}; // namespace
#endif // _FORGETTING_AGENT_H

