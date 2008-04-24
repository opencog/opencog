/**
 * ForgettingAgent.h
 *
 * Forgetting agent, removes atoms with low LTI.
 *
 * Author: Joel Pitt 
 * Creation: Wed Apr 24 15:32:00 GMT+12 2008
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

class ForgettingAgent : public MindAgent
{

    private:
	AtomSpace* a;

    public:
	// Maximum LTI of a link that can be forgot.
	AttentionValue::lti_t forgettingThreshold;
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

