/*
 * SentenceGenAgent.cc
 *
 *  Created on: 10 Nov 2015
 *      Author: misgana
 */
#include <sstream>
#include <fstream>
#include <chrono>
#include <thread>
#include <boost/filesystem.hpp>

#include <opencog/neighbors/GetPredicates.h>
#include <opencog/attention/atom_types.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/nlp/types/atom_types.h>

#include "ExperimentSetupModule.h"
#include "SentenceGenStimulateAgent.h"

using namespace opencog;
using namespace opencog::ECANExperiment;

using namespace std;
using namespace chrono;

SentenceGenStimulateAgent::~SentenceGenStimulateAgent()
{
    //delete _scm_eval;
}
SentenceGenStimulateAgent::SentenceGenStimulateAgent(CogServer& cs) :
        Agent(cs)
{
    _scm_eval = new SchemeEval(_as);
    current_group = 0;
    startcount = _cogserver.getCycleCount();
    stime = std::time(nullptr);
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
        printf("WORD_NODE = %d \n",_as->get_num_atoms_of_type(WORD_NODE));
        printf("WORD_INSTANCE_NODE = %d \n",_as->get_num_atoms_of_type(WORD_INSTANCE_NODE));
        printf("ASYMMETRIC_HEBBIAN_LINK = %d \n",_as->get_num_atoms_of_type(ASYMMETRIC_HEBBIAN_LINK));
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
            rnd = rand() % data.size();
            out.push_back(data[rnd]);
        }
    };

    if (time(nullptr) > stime + 5) {
        current_group = (current_group + 1) % swords.size();
        if (current_group == 0)
            this_thread::sleep_for(seconds(5));
        stime = time(nullptr);
    }

    if (_cogserver.getCycleCount() % special_word_occurence_period == 0) { // and
    //(_cogserver.getCycleCount() - startcount) > 5 ) {
      select(4,swords[current_group],selected_words);
      select(2,words,selected_words);
    } else {
        select(6,words,selected_words);
    }

    for (std::string word : selected_words) {
        hwords.push_back(evalWord(word));
        hword_instances.push_back(evalWord(word + "@" + std::to_string(rand())));
    }

    for (Handle h : hwords)
        _as->stimulate(h,2);
    for (Handle h : hword_instances)
        _as->stimulate(h,0.5);
    this_thread::sleep_for(milliseconds(400));

    printf("stifunds: %ld \n",_as->get_STI_funds());
}
