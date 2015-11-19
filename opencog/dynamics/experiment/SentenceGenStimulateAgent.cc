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
        HandleSeq word_and_wordinst;

        //Get the word instance nodes.
        std::string wordinst_list =
                "(ListLink (parse-get-words (car (sentence-get-parses (car (nlp-parse \"" + sentence
                + "\"))))))";
        Handle hwordinst_list = _scm_eval->eval_h(wordinst_list);
        HandleSeq hstemp = _as.get_outgoing(hwordinst_list);

        word_and_wordinst.insert(word_and_wordinst.end(), hstemp.begin(), hstemp.end());

        //Get the word nodes.
        std::string wordnode_list =
                "(ListLink (append-map (lambda (x) (cog-chase-link 'ReferenceLink 'WordNode x)) "
                "(parse-get-words (car (sentence-get-parses (car (nlp-parse \""
                + sentence + "\")))))))";
        Handle hwordnode_list = _scm_eval->eval_h(wordnode_list);
        hstemp = _as.get_outgoing(hwordnode_list);

        word_and_wordinst.insert(word_and_wordinst.end(), hstemp.begin(), hstemp.end());

        //Stimulate. TODO read stimulus value from config file
        stimulateAtom(word_and_wordinst, 20);

        _hword_wordInstance_nodes.insert(word_and_wordinst.begin(),
                                         word_and_wordinst.end());
    }
}

void SentenceGenStimulateAgent::run()
{
    static bool nlp_parse_called = false;

    if (not nlp_parse_called) {
        insertStimulate();
        nlp_parse_called = true;

    } else {
        for (const Handle& h : _hword_wordInstance_nodes)
            stimulateAtom(h, 20);
    }
}
