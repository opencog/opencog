/*
 * SentenceGenAgent.cc
 *
 *  Created on: 10 Nov 2015
 *      Author: misgana
 */
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>

#include <opencog/atomutils/AtomUtils.h>
#include <opencog/attention/experiment/ExperimentSetupModule.h>
#include <opencog/attention/experiment/SentenceGenStimulateAgent.h>
#include <opencog/attention/atom_types.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/nlp/types/atom_types.h>


using namespace opencog;
using namespace opencog::ECANExperiment;

SentenceGenStimulateAgent::~SentenceGenStimulateAgent()
{
    //delete _scm_eval;
}
SentenceGenStimulateAgent::SentenceGenStimulateAgent(CogServer& cs) :
        Agent(cs), _as(cs.getAtomSpace())
{
    _scm_eval = new SchemeEval(&_as);
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

void SentenceGenStimulateAgent::run(void)
{
    generate_stimulate_sentence();

    if(_cogserver.getCycleCount() % 10 == 0){
        //Print counts
        printf("WORD_NODE = %d \n",_as.get_num_atoms_of_type(WORD_NODE));
        printf("WORD_INSTANCE_NODE = %d \n",_as.get_num_atoms_of_type(WORD_INSTANCE_NODE));
        printf("ASYMMETRIC_HEBBIAN_LINK = %d \n",_as.get_num_atoms_of_type(ASYMMETRIC_HEBBIAN_LINK));
    }
}

#define StringSeq std::vector<std::string>

void SentenceGenStimulateAgent::generate_stimulate_sentence()
{
    HandleSeq hwords;
    HandleSeq hword_instances;
    StringSeq selected_words;

    auto evalWord = [this](std::string word) -> Handle {
        return _scm_eval->eval_h("(ConceptNode \"" + word + "\")");
    };

    auto select = [](int num,StringSeq &data,StringSeq &out) -> void {
        int rnd;
        for (int i = 0; i < num;++i){
            rnd = rand() % (data.size() / num) + (data.size() / num) * i;
            out.push_back(data[rnd]);
        }
    };

    if (_cogserver.getCycleCount() % special_word_occurence_period == 0) {
        if (_cogserver.getCycleCount() % 25 == 0)
            current_group = (current_group + 1) % swords.size();
        select(2,swords[current_group],selected_words);
        select(4,words,selected_words);
    } else {
        select(6,words,selected_words);
    }

    for (std::string word : selected_words) {
        hwords.push_back(evalWord(word));
        hword_instances.push_back(evalWord(word + "@" + std::to_string(rand())));
    }

    for (Handle h : hwords)
        stimulateAtom(h,2);
    for (Handle h : hword_instances)
        stimulateAtom(h,0.5);
    printf("stifunds: %ld \n",_as.get_STI_funds());
}
