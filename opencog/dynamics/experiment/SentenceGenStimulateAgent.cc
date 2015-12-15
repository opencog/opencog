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
#include <opencog/dynamics/attention/atom_types.h>
#include <opencog/server/Factory.h>
#include <opencog/nlp/types/atom_types.h>

#include "ExperimentSetupModule.h"
#include "SentenceGenStimulateAgent.h"

using namespace opencog;
using namespace opencog::ECANExperiment;

namespace opencog{namespace ECANExperiment{
//extern std::vector<std::string> generated_sentences;
extern std::vector<HandleSeq> sent_wordnodes;
extern std::vector<HandleSeq> wordinstancenodes;

extern UnorderedHandleSet hspecial_word_nodes;

extern std::vector<std::string> special_words;
extern std::vector<std::string> nspecial_words;
extern int sent_size;
extern int special_word_occurence_period;
}
}

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
    static int ssize = sent_size;

    if (ssize > 0) {
        generate_stimuate_sentence();
        ssize--;
    }

    if(_cogserver.getCycleCount() % 50 == 0){
        std::string dir_path = std::string(PROJECT_SOURCE_DIR)
                + "/opencog/dynamics/experiment/visualization/" + "cycle"
                + std::to_string(_cogserver.getCycleCount());

        boost::filesystem::path dir(dir_path);

        if (boost::filesystem::create_directory(dir)) {
            std::string file_name = dir_path + "/dump";

            //dump hebbian strength of between special word nodes
            ExperimentSetupModule::dump_ecan_data("heb", file_name);
            std::cout << "[INFO ]" << file_name << " written." << std::endl;

            std::ofstream outf(dir_path + "/atom_count.txt",
                               std::ofstream::out | std::ofstream::trunc);

            //Print counts
            outf <<"WORD_NODE = "<< _as.get_num_atoms_of_type(WORD_NODE);
            outf <<"WORD_INSTANCE_NODE = "<< _as.get_num_atoms_of_type(WORD_INSTANCE_NODE);
            outf <<"ASYMMETRIC_HEBBIAN_LINK = "<< _as.get_num_atoms_of_type(ASYMMETRIC_HEBBIAN_LINK);

            outf.flush();
            outf.close();
        }

    }
}

void SentenceGenStimulateAgent::generate_stimuate_sentence(void)
{
    static int cycle = 1;
    std::vector<std::string> sentences;
    int sw_end = special_words.size() - 1;
    int nsw_end = nspecial_words.size() - 1;
    static int i = 0;

    HandleSeq hwords;
    HandleSeq hword_instances;

    if (cycle % special_word_occurence_period == 0) {
        //Two random special words from each half
        int sw1 = rand() % (sw_end / 2);
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + special_words[sw1] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + special_words[sw1] + "@"
                        + std::to_string(++i) + "\")"));

        int sw2 = rand() % (sw_end / 2) + sw_end / 2;
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + special_words[sw2] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + special_words[sw2] + "@"
                        + std::to_string(++i) + "\")"));

        //Store special word nodes
        hspecial_word_nodes.insert(hwords[0]);
        hspecial_word_nodes.insert(hwords[1]);

        //Four Random non-special words chosen from each quarters
        int rw1 = rand() % (nsw_end / 4);
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw1] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + nspecial_words[rw1] + "@"
                        + std::to_string(++i) + "\")"));

        int rw2 = rand() % (nsw_end / 4) + nsw_end / 4;
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw2] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + nspecial_words[rw2] + "@"
                        + std::to_string(++i) + "\")"));

        int rw3 = rand() % (nsw_end / 4) + nsw_end / 2;
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw3] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + nspecial_words[rw3] + "@"
                        + std::to_string(++i) + "\")"));

        int rw4 = rand() % (nsw_end / 4) + nsw_end * 3 / 4;
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw4] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + nspecial_words[rw4] + "@"
                        + std::to_string(++i) + "\")"));
    } else {
        //Six Random non-special words chosen from each quarters
        int rw1 = rand() % (nsw_end / 6);
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw1] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + nspecial_words[rw1] + "@"
                        + std::to_string(++i) + "\")"));

        int rw2 = rand() % (nsw_end / 6) + nsw_end / 6;
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw2] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + nspecial_words[rw2] + "@"
                        + std::to_string(++i) + "\")"));

        int rw3 = rand() % (nsw_end / 6) + nsw_end / 3;
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw3] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + nspecial_words[rw3] + "@"
                        + std::to_string(++i) + "\")"));

        int rw4 = rand() % (nsw_end / 6) + nsw_end  / 2;
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw4] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + nspecial_words[rw4] + "@"
                        + std::to_string(++i) + "\")"));

        int rw5 = rand() % (nsw_end / 6) + nsw_end * 2 / 3;
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw5] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + nspecial_words[rw5] + "@"
                        + std::to_string(++i) + "\")"));

        int rw6 = rand() % (nsw_end / 6) + nsw_end * 5/6 ;
        hwords.push_back(
                _scm_eval->eval_h("(WordNode\"" + nspecial_words[rw6] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(WordInstanceNode\"" + nspecial_words[rw6] + "@"
                        + std::to_string(++i) + "\")"));
    }
    //Should the non special word nodes be removed from the selection list?

    //Stimulate atoms. TODO change stimulus values.
    stimulateAtom(hwords, 20);
    stimulateAtom(hword_instances, 20);

    cycle++;

    //Push sentence nodes
    sent_wordnodes.push_back(hwords);
    wordinstancenodes.push_back(hword_instances);
}
