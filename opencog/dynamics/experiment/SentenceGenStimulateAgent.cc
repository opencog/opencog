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

void SentenceGenStimulateAgent::run(void) {
    static int ssize= sent_size;


    if(ssize > 0){
        generate_stimuate_sentence();
        ssize--;
    }
}

void SentenceGenStimulateAgent::generate_stimuate_sentence(
        void)
{
    std::vector<std::string> sentences;
    int sw_end = special_words.size() - 1;
    int nsw_end = nspecial_words.size() - 1;
    int i = 0;

    HandleSeq hwords;
    HandleSeq hword_instances;

    //Two random special words from each half
    int sw1 = rand() % (sw_end / 2);
    hwords.push_back(
            _scm_eval->eval_h("(WordNode\"" + special_words[sw1] + "\")"));
    hword_instances.push_back(
            _scm_eval->eval_h(
                    "(WordInstanceNode\"" + special_words[sw1] + "@" + std::to_string(++i)
                    + "\")"));

    int sw2 = rand() % (sw_end / 2) + sw_end / 2;
    hwords.push_back(
            _scm_eval->eval_h("(WordNode\"" + special_words[sw2] + "\")"));
    hword_instances.push_back(
            _scm_eval->eval_h(
                    "(WordInstanceNode\"" + special_words[sw2] + "@" + std::to_string(++i)
                    + "\")"));

    //Store special word nodes
    hspecial_word_nodes.insert(hwords[0]);
    hspecial_word_nodes.insert(hwords[1]);

    //Four Random non-special words chosen from each quarters
    int rw1 = rand() % (nsw_end / 4);
    hwords.push_back(
            _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw1] + "\")"));
    hword_instances.push_back(
            _scm_eval->eval_h(
                    "(WordInstanceNode\"" + nspecial_words[rw1] + "@" + std::to_string(++i)
                    + "\")"));

    int rw2 = rand() % (nsw_end / 4) + nsw_end / 4;
    hwords.push_back(
            _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw2] + "\")"));
    hword_instances.push_back(
            _scm_eval->eval_h(
                    "(WordInstanceNode\"" + nspecial_words[rw2] + "@" + std::to_string(++i)
                    + "\")"));

    int rw3 = rand() % (nsw_end / 4) + nsw_end / 2;
    hwords.push_back(
            _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw3] + "\")"));
    hword_instances.push_back(
            _scm_eval->eval_h(
                    "(WordInstanceNode\"" + nspecial_words[rw3] + "@" + std::to_string(++i)
                    + "\")"));

    int rw4 = rand() % (nsw_end / 4) + nsw_end * 3 / 4;
    hwords.push_back(
            _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw4] + "\")"));
    hword_instances.push_back(
            _scm_eval->eval_h(
                    "(WordInstanceNode\"" + nspecial_words[rw4] + "@" + std::to_string(++i)
                    + "\")"));

    //Should the non special word nodes be removed from the selection list?

    //Stimulate atoms. TODO change stimulus values.
    stimulateAtom(hwords, 20);
    stimulateAtom(hword_instances, 20);

    //Push sentence nodes
    sent_wordnodes.push_back(hwords);
    wordinstancenodes.push_back(hword_instances);

}
