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
#include <opencog/truthvalue/SimpleTruthValue.h>

#include <opencog/util/Config.h>

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
  //STIAtomWage = config().get_int("ECAN_STARTING_ATOM_STI_RENT");
  //LTIAtomWage = config().get_int("ECAN_STARTING_ATOM_LTI_RENT");

  //targetSTI = config().get_int("STARTING_STI_FUNDS");
  //stiFundsBuffer = config().get_int("STI_FUNDS_BUFFER");
  //targetLTI = config().get_int("STARTING_LTI_FUNDS");
  //ltiFundsBuffer = config().get_int("LTI_FUNDS_BUFFER");


    STIAtomWage = 100;
    LTIAtomWage = 100;

    targetSTI = 10000;
    stiFundsBuffer = 10000;
    targetLTI = 10000;
    ltiFundsBuffer = 10000;

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
                + "/opencog/attention/experiment/visualization/";
                //"cycle" + std::to_string(_cogserver.getCycleCount());

        std::string file_name = dir_path + "dump";

        //dump hebbian strength of between special word nodes
        ExperimentSetupModule::dump_ecan_data("heb", file_name);
        ExperimentSetupModule::dump_ecan_data("av", file_name);
        std::cout << "[INFO ]" << file_name << " written." << std::endl;

      //std::ofstream outf(dir_path + "/atom_count.txt",
      //                   std::ofstream::out | std::ofstream::trunc);

      ////Print counts
      //outf <<"WORD_NODE = "<< _as.get_num_atoms_of_type(WORD_NODE);
      //outf <<"WORD_INSTANCE_NODE = "<< _as.get_num_atoms_of_type(WORD_INSTANCE_NODE);
      //outf <<"ASYMMETRIC_HEBBIAN_LINK = "<< _as.get_num_atoms_of_type(ASYMMETRIC_HEBBIAN_LINK);

      //outf.flush();
      //outf.close();
    }
}

void SentenceGenStimulateAgent::generate_stimuate_sentence(void)
{
    static int cycle = 1;
    std::vector<std::string> sentences;
    int sw_end = special_words.size();
    int nsw_end = nspecial_words.size();
    static int i = 0;

    HandleSeq hwords;
    HandleSeq hword_instances;

    if (cycle % special_word_occurence_period == 0 && cycle > 10) {
        //Two random special words from each half
        int sw1 = rand() % (sw_end / 2);
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + special_words[sw1] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + special_words[sw1] + "@"
                        + std::to_string(++i) + "\")"));

        int sw2 = rand() % (sw_end / 2) + sw_end / 2;
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + special_words[sw2] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + special_words[sw2] + "@"
                        + std::to_string(++i) + "\")"));

        //Store special word nodes
        hspecial_word_nodes.insert(hwords[0]);
        hspecial_word_nodes.insert(hwords[1]);

        //Four Random non-special words chosen from each quarters
        int rw1 = rand() % (nsw_end / 4);
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + nspecial_words[rw1] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + nspecial_words[rw1] + "@"
                        + std::to_string(++i) + "\")"));

        int rw2 = rand() % (nsw_end / 4) + nsw_end / 4;
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + nspecial_words[rw2] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + nspecial_words[rw2] + "@"
                        + std::to_string(++i) + "\")"));

        int rw3 = rand() % (nsw_end / 4) + nsw_end / 2;
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + nspecial_words[rw3] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + nspecial_words[rw3] + "@"
                        + std::to_string(++i) + "\")"));

        int rw4 = rand() % (nsw_end / 4) + nsw_end * 3 / 4;
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + nspecial_words[rw4] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + nspecial_words[rw4] + "@"
                        + std::to_string(++i) + "\")"));
    } else {
        //Six Random non-special words chosen from each quarters
        int rw1 = rand() % (nsw_end / 6);
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + nspecial_words[rw1] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + nspecial_words[rw1] + "@"
                        + std::to_string(++i) + "\")"));

        int rw2 = rand() % (nsw_end / 6) + nsw_end / 6;
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + nspecial_words[rw2] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + nspecial_words[rw2] + "@"
                        + std::to_string(++i) + "\")"));

        int rw3 = rand() % (nsw_end / 6) + nsw_end / 3;
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + nspecial_words[rw3] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + nspecial_words[rw3] + "@"
                        + std::to_string(++i) + "\")"));

        int rw4 = rand() % (nsw_end / 6) + nsw_end  / 2;
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + nspecial_words[rw4] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + nspecial_words[rw4] + "@"
                        + std::to_string(++i) + "\")"));

        int rw5 = rand() % (nsw_end / 6) + nsw_end * 2 / 3;
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + nspecial_words[rw5] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + nspecial_words[rw5] + "@"
                        + std::to_string(++i) + "\")"));

        int rw6 = rand() % (nsw_end / 6) + nsw_end * 5/6 ;
        hwords.push_back(
                _scm_eval->eval_h("(ConceptNode\"" + nspecial_words[rw6] + "\")"));
        hword_instances.push_back(
                _scm_eval->eval_h(
                        "(ConceptNode\"" + nspecial_words[rw6] + "@"
                        + std::to_string(++i) + "\")"));
    }
    //Should the non special word nodes be removed from the selection list?

    //Stimulate atoms. TODO change stimulus values.
    for (Handle h : hwords)
        localStimulateAtom(h,2);
    for (Handle h : hword_instances)
        localStimulateAtom(h,0.5);
    //stimulateAtom(hwords, 20);
    //stimulateAtom(hword_instances, 20);
    fprintf(stdout,"stifunds: %d \n",_as.get_STI_funds());

    cycle++;

    //Push sentence nodes
    sent_wordnodes.push_back(hwords);
    wordinstancenodes.push_back(hword_instances);
}


void SentenceGenStimulateAgent::hebbianUpdatingUpdate(Handle source)
{
    float tcDecayRate = 0.3f;
    float tc, old_tc, new_tc;

    IncomingSet links = source->getIncomingSetByType(ASYMMETRIC_HEBBIAN_LINK);

    for (LinkPtr h : links) {
        if (source != h->getOutgoingAtom(0))
            continue;
        HandleSeq outgoing = h->getOutgoingSet();
        new_tc = targetConjunction(outgoing);

        // old link strength decays
        TruthValuePtr oldtv  = h->getTruthValue();
        old_tc = oldtv->getMean();
        tc = tcDecayRate * new_tc + (1.0f - tcDecayRate) * old_tc;

        //update truth value accordingly
        TruthValuePtr newtv(SimpleTruthValue::createTV(tc, 1));
        h->merge(newtv);
    }
    //h->setTruthValue(SimpleTruthValue::createTV(tc, 1));
}

float SentenceGenStimulateAgent::targetConjunction(HandleSeq handles)
{
    if (handles.size() != 2) {
        throw RuntimeException(
                TRACE_INFO,
                "Size of outgoing set of a hebbian link must be 2.");
    }
    //XXX: Should this be normalised to 0->1 Range
    auto normsti_i = _as.get_normalised_STI(handles[0],true,true);
    auto normsti_j = _as.get_normalised_STI(handles[1],true,true);
    float conj = std::max(-1.0f,std::min(1.0f,normsti_i * normsti_j));
    conj = conj + 1 / 2;

    return conj;
}
