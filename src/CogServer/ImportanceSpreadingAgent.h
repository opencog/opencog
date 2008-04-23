/**
 * HebbianLearningAgent.h
 *
 * Simple Hebbian Learning, currently only updates existing Hebbian Links.
 *
 * Author: Joel Pitt 
 * Creation: Wed Apr 9 16:32:00 GMT+12 2008
 */

#ifndef _IMPORTANCE_SPREADING_AGENT_H
#define _IMPORTANCE_SPREADING_AGENT_H

#include <string>
#include <math.h>

#include <Logger.h>

#include <AtomSpace.h>
#include <MindAgent.h>
#include <AttentionValue.h>

#define MA_DEFAULT_SPREAD_THRESHOLD 4
#define MA_DEFAULT_SPREAD_MULTIPLIER 10.0f
#define MA_DEFAULT_SPREAD_STIM 1

namespace opencog {

class CogServer;

class ImportanceSpreadingAgent : public MindAgent
{

    private:
	AtomSpace* a;

	AttentionValue::sti_t spreadThreshold;
	float importanceSpreadingMultiplier;

	void spreadAtomImportance(Handle h);


    public:

	ImportanceSpreadingAgent();
	virtual ~ImportanceSpreadingAgent();
	virtual void run(CogServer *server);
	void setSpreadThreshold(AttentionValue::sti_t t) {spreadThreshold = t;};
	void setImportanceSpreadingMultiplier(float m) {
	    importanceSpreadingMultiplier = m;};

	/**
	 *
	 */
	void spreadImportance();

}; // class

struct ImportanceSpreadSTISort
{
     bool operator()(const Handle& h1, const Handle& h2)
     {
          return TLB::getAtom(h1)->getAttentionValue().getSTI() > TLB::getAtom(h2)->getAttentionValue().getSTI();
     }
};

struct ImportanceSpreadLTIAndTVAscendingSort
{
    bool operator()(const Handle& h1, const Handle& h2)
    {
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

struct ImportanceSpreadLTIThenTVAscendingSort
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
#endif // _IMPORTANCE_SPREADING_AGENT_H

