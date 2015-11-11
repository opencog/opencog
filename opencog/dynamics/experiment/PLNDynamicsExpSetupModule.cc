/*
 * PLNDynamicsExpSetupModule.cc
 *
 *  Created on: Apr 19, 2015
 *      Author: misgana
 */

#include <sstream>
#include <string>
#include <fstream>

#include <opencog/atomspace/AtomSpace.h>

#include <opencog/dynamics/attention/ForgettingAgent.h>
#include <opencog/dynamics/attention/ImportanceUpdatingAgent.h>
#include <opencog/dynamics/attention/HebbianUpdatingAgent.h>
#include <opencog/dynamics/attention/SimpleImportanceDiffusionAgent.h>
#include <opencog/dynamics/experiment/ArtificialStimulatorAgent.h>
#include <opencog/dynamics/experiment/PLNDynamicsExpSetupModule.h>
#include <opencog/dynamics/experiment/SurprisingnessEvaluatorAgent.h>

#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>

#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

using namespace opencog;
using namespace opencog::ECANExperiment;

DECLARE_MODULE(PLNDynamicsExpSetupModule);

PLNDynamicsExpSetupModule::PLNDynamicsExpSetupModule(CogServer& cs) :
        Module(cs), _cs(cs)
{
    _log = new Logger(); //new Logger("ecanexpe.log");
    _log->fine("uuid,cycle,sti_old,sti_new,lti,vlti,wage,rent,agent_name");
    _as = &_cs.getAtomSpace();

    _AVChangedSignalConnection = _as->AVChangedSignal(
            boost::bind(&PLNDynamicsExpSetupModule::AVChangedCBListener, this,
                        _1, _2, _3));
}

PLNDynamicsExpSetupModule::~PLNDynamicsExpSetupModule()
{
    unregisterAgentRequests();
}

void PLNDynamicsExpSetupModule::AVChangedCBListener(
        const Handle& h, const AttentionValuePtr& av_old,
        const AttentionValuePtr& av_new)
{
    ECANValue ecanval(av_new->getSTI(), av_new->getLTI(), _cs.getCycleCount());
    _data[h] = ecanval;
}

void PLNDynamicsExpSetupModule::registerAgentRequests()
{
    do_ecan_load_register();
    do_ecan_start_register();
    do_ecan_pause_register();
    do_stimulate_register();
}

void PLNDynamicsExpSetupModule::unregisterAgentRequests()
{
    do_ecan_load_unregister();
    do_ecan_start_unregister();
    do_ecan_pause_unregister();
    do_stimulate_unregister();
}
void PLNDynamicsExpSetupModule::init(void)
{
    //Load params
    _as->set_attentional_focus_boundary(
            (stim_t) config().get_int("AF_BOUNDARY"));

    registerAgentRequests();
}

std::string PLNDynamicsExpSetupModule::do_ecan_load(Request *req,
                                                    std::list<std::string> args)
{
    //These mind agents have already been made registered by the attention module.So no need to register them.
    _forgetting_agentptr = _cs.createAgent(ForgettingAgent::info().id, false);
    _hebbianupdating_agentptr = _cs.createAgent(HebbianUpdatingAgent::info().id,
    false);
    _importanceupdating_agentptr = _cs.createAgent(
            ImportanceUpdatingAgent::info().id, false);
    _simpleimportancediffusion_agentptr = _cs.createAgent(
            SimpleImportanceDiffusionAgent::info().id, false);

    //Register experiment specific agents. Add more if you have here.

    Factory<SurprisingnessEvaluatorAgent, Agent> surprisingnessAgentFactory;
    bool status = _cs.registerAgent(SurprisingnessEvaluatorAgent::info().id,
                                    &surprisingnessAgentFactory);
    if (status) {
        _surprisingness_agentptr = _cs.createAgent(
                SurprisingnessEvaluatorAgent::info().id, false);
    }

    Factory<ArtificialStimulatorAgent, Agent> artificialStimulatorAgentFactory;
    status = _cs.registerAgent(ArtificialStimulatorAgent::info().id,
                               &artificialStimulatorAgentFactory);
    if (status) {
        _artificialstimulatoragentptr = _cs.createAgent(
                ArtificialStimulatorAgent::info().id, false);
    }

    return "Loaded the following agents:\n" + ECAN_EXP_AGENTS;
}

std::string PLNDynamicsExpSetupModule::do_ecan_start(
        Request *req, std::list<std::string> args)
{
    _cs.startAgent(_forgetting_agentptr);
    _cs.startAgent(_hebbianupdating_agentptr);
    _cs.startAgent(_importanceupdating_agentptr);
    _cs.startAgent(_simpleimportancediffusion_agentptr);

    _cs.startAgent(_artificialstimulatoragentptr);

    return "The following agents were started:\n" + ECAN_EXP_AGENTS;
}

std::string PLNDynamicsExpSetupModule::do_ecan_pause(
        Request *req, std::list<std::string> args)
{
    _cs.stopAgent(_forgetting_agentptr);
    _cs.stopAgent(_hebbianupdating_agentptr);
    _cs.stopAgent(_importanceupdating_agentptr);
    _cs.stopAgent(_simpleimportancediffusion_agentptr);

    _cs.stopAgent(_artificialstimulatoragentptr);

    return "The following agents were stopped:\n" + ECAN_EXP_AGENTS;
}

std::string PLNDynamicsExpSetupModule::do_stimulate(Request *req,
                                                    std::list<std::string> args)
{
    for (std::string& pairs : args) {
        auto pos = pairs.find_first_of(",");

        std::string uuid = pairs.substr(0, pos - 1);
        Handle h((UUID) std::strtoul(uuid.c_str(), nullptr, 10));

        std::string stimulus = pairs.substr(pos + 1);
        stim_t stim = std::stoi(stimulus);

        _artificialstimulatoragentptr->stimulateAtom(h, stim);
    }

    return "stimulus provided.\n";
}

std::string PLNDynamicsExpSetupModule::do_dump_data(Request *req,
                                                    std::list<std::string> args)
{
    std::string file_name = args.back();
    // cycle,uuid,sti,lti
    std::stringstream sstream;

    for (const auto& p : _data) {
        sstream << std::to_string(p.second._cycle) << ","
        << std::to_string(p.first.value()) << ","
        << std::to_string(p.second._sti) << "," << std::to_string(p.second._lti)
        << "\n";
    }

    std::ofstream outf(file_name);
    outf << sstream.str();
    outf.flush();
    outf.close();

    return "Time series data dumped in to " + file_name;

}

std::string PLNDynamicsExpSetupModule::load_word_dict(
        Request *req, std::list<std::string> args)
{
    auto get_wordvec = [](const std::string& str) {
        std::vector<std::string> words;
        auto init = str.begin();
        for (auto i = str.begin(); i != str.end(); ++i) {
            if (*i == ',') {
                std::string word = str.substr(
                        init - str.begin(), i - 1 - str.begin());
                words.push_back(word);
                init = i + 1;
            }
        }

        return words;
    };

    std::string file_name = args.back();
    config().load(file_name.c_str());
    std::string special_wstr = config().get("SPECIAL_WORDS");
    std::string nspecial_wstr = config().get("NON_SPECIAL_WORDS");

    std::vector<std::string> special_words = get_wordvec(special_wstr);
    std::vector<std::string> nspecial_words = get_wordvec(nspecial_wstr);

    std::vector<std::string> sentences = generate_sentence(
            nspecial_words, special_words, config().get_int("SENTENCE_SIZE"));

    generated_sentences = sentences;

}

std::vector<std::string> PLNDynamicsExpSetupModule::generate_sentence(
        const std::vector<std::string>& non_special_words,
        const std::vector<std::string>& special_words, int sent_size)
{
    std::vector<std::string> sentences;
    int send = special_words.size() - 1;
    int nsend = non_special_words.size() - 1;

    for (; sent_size > 0; sent_size--) {
        //Two random special words from each half
        int sw1 = rand() % (send / 2);
        int sw2 = rand() % (send / 2) + send / 2;

        //Four Random non-special words chosen from each quarters
        int rw1 = rand() % (nsend / 4);
        int rw2 = rand() % (nsend / 4) + nsend / 4;
        int rw3 = rand() % (nsend / 4) + nsend / 2;
        int rw4 = rand() % (nsend / 4) + nsend * 3 / 4;

        //Don't care about order
        std::string sentence = special_words[sw1] + non_special_words[rw1]
                               + non_special_words[rw2] + special_words[sw2]
                               + non_special_words[rw3]
                               + non_special_words[rw4];

        sentences.push_back(sentence);
    }

    return sentences;
}

