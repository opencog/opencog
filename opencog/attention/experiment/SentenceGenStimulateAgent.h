/*
 * SentenceGenStimulateAgent.h
 *
 *  Created on: 10 Nov 2015
 *      Author: misgana
 */
#include <opencog/guile/SchemeEval.h>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/CogServer.h>

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

public:
    virtual ~ SentenceGenStimulateAgent();
    SentenceGenStimulateAgent(CogServer& cs);

    virtual const ClassInfo& classinfo() const;
    static const ClassInfo& info();
    void generate_stimuate_sentence(void);
    virtual void run();
};

}
}
#endif /* _SENTENCEGENAGENT_H_ */
