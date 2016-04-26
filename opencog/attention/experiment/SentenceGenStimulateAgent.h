/*
 * SentenceGenStimulateAgent.h
 *
 *  Created on: 10 Nov 2015
 *      Author: misgana
 */
#include <opencog/guile/SchemeEval.h>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/truthvalue/AttentionValue.h>

#ifndef _SENTENCEGENAGENT_H_
#define _SENTENCEGENAGENT_H_

namespace opencog {
namespace ECANExperiment {

/**
 * Generate Random sentence and stimulates them on some cognitive cycle interval.
 */
class SentenceGenStimulateAgent: public Agent {
private:
    UnorderedHandleSet _hword_wordInstance_nodes;
    SchemeEval* _scm_eval;
    AtomSpace& _as;

    AttentionValue::sti_t STIAtomWage;
    AttentionValue::lti_t LTIAtomWage;

    AttentionValue::sti_t targetSTI;
    AttentionValue::lti_t targetLTI;

    AttentionValue::sti_t stiFundsBuffer;
    AttentionValue::lti_t ltiFundsBuffer;

public:
    virtual ~ SentenceGenStimulateAgent();
    SentenceGenStimulateAgent(CogServer& cs);

    virtual const ClassInfo& classinfo() const;
    static const ClassInfo& info();
    void generate_stimuate_sentence(void);
    virtual void run();

    void localStimulateAtom(Handle h,float stimulus);
    AttentionValue::sti_t calculate_STI_Wage();
    AttentionValue::lti_t calculate_LTI_Wage();
    void hebbianUpdatingUpdate(Handle h);
    float targetConjunction(HandleSeq handles);
};

}
}
#endif /* _SENTENCEGENAGENT_H_ */
