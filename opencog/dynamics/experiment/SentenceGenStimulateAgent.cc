/*
 * SentenceGenAgent.cc
 *
 *  Created on: 10 Nov 2015
 *      Author: misgana
 */
#include <opencog/atomutils/AtomUtils.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/nlp/types/atom_types.h>

#include "SentenceGenStimulateAgent.h"

using namespace opencog;
using namespace opencog::ECANExperiment;

SentenceGenStimulateAgent::~SentenceGenStimulateAgent()
{
    delete _scm_eval;
}
SentenceGenStimulateAgent::SentenceGenStimulateAgent(CogServer& cs) :
        Agent(cs), _as(cs.getAtomSpace())
{
    _scm_eval = new SchemeEval(&_as);
    _scm_eval->eval("(load-r2l-rulebase)");
}

const ClassInfo& SentenceGenStimulateAgent::classinfo() const
{
    return info();
}

const ClassInfo& SentenceGenStimulateAgent::info()
{
    static const ClassInfo _ci("opencog::SentenceGenStimulateAgent");
    return _ci;
}

void SentenceGenStimulateAgent::insertStimulate(void)
{
    for (const auto& sentence : generated_sentences) {
        std::string sentence_atom = _scm_eval->eval(
                "(nlp-parse " + sentence + ")");
        HandleSeq hs;
        _as.get_handles_by_name(std::back_inserter(hs), sentence_atom);
        Handle hparseLink = _as.get_incoming(hs.back()).back();
        UnorderedHandleSet hseq = get_outgoing_nodes(hparseLink, std::vector<Type> {
                WORD_NODE,WORD_INSTANCE_NODE});
        HandleSeq hsc;
        hsc.insert(hsc.end(),hseq.begin(),hseq.end());
        stimulateAtom(hs, 20);
        _hword_wordInstance_nodes.insert(hseq.begin(), hseq.end());
    }
}

void SentenceGenStimulateAgent::run()
{
    for (const Handle& h : _hword_wordInstance_nodes) {
        stimulateAtom(h, 20);
    }
}
