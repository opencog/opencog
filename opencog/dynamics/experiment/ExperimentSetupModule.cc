/*
 * ExperimentSetupModule.cc
 *
 *  Created on: Apr 19, 2015
 *      Author: misgana
 */

#include <sstream>
#include <string>
#include <fstream>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemeEval.h>

#include <opencog/dynamics/attention/ForgettingAgent.h>
#include <opencog/dynamics/attention/ImportanceUpdatingAgent.h>
#include <opencog/dynamics/attention/HebbianUpdatingAgent.h>
#include <opencog/dynamics/attention/SimpleImportanceDiffusionAgent.h>
#include <opencog/dynamics/experiment/ArtificialStimulatorAgent.h>
#include <opencog/dynamics/experiment/ExperimentSetupModule.h>

#include <opencog/server/CogServer.h>
#include <opencog/server/Module.h>

#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

using namespace opencog;
using namespace opencog::ECANExperiment;

DECLARE_MODULE(ExperimentSetupModule);

ExperimentSetupModule::ExperimentSetupModule(CogServer& cs) :
        Module(cs), _cs(cs)
{
    _log = new Logger(); //new Logger("ecanexpe.log");
    _log->fine("uuid,cycle,sti_old,sti_new,lti,vlti,wage,rent,agent_name");
    _as = &_cs.getAtomSpace();
    _scmeval = new SchemeEval(_as);

    _AVChangedSignalConnection = _as->AVChangedSignal(
            boost::bind(&ExperimentSetupModule::AVChangedCBListener, this, _1,
                        _2, _3));
}

ExperimentSetupModule::~ExperimentSetupModule()
{
    unregisterAgentRequests();
    delete(_scmeval);
}

void ExperimentSetupModule::AVChangedCBListener(const Handle& h,
                                                const AttentionValuePtr& av_old,
                                                const AttentionValuePtr& av_new)
{
    ECANValue ecanval(av_new->getSTI(), av_new->getLTI(),av_new->getVLTI(), _cs.getCycleCount());
    _data[h].push_back(ecanval);
}

void ExperimentSetupModule::registerAgentRequests()
{
    do_ecan_load_register();
    do_ecan_start_register();
    do_ecan_pause_register();
    do_stimulate_register();
    do_load_word_dict_register();
    do_dump_data_register();
    do_start_nlp_stimulate_register();
    do_pause_nlp_stimulate_register();
}

void ExperimentSetupModule::unregisterAgentRequests()
{
    do_ecan_load_unregister();
    do_ecan_start_unregister();
    do_ecan_pause_unregister();
    do_stimulate_unregister();
    do_load_word_dict_unregister();
    do_dump_data_unregister();
    do_start_nlp_stimulate_unregister();
    do_pause_nlp_stimulate_unregister();
}
void ExperimentSetupModule::init(void)
{
    //Load params
    //_as->set_attentional_focus_boundary(
    //        (stim_t) config().get_int("AF_BOUNDARY"));

    registerAgentRequests();
}

std::string ExperimentSetupModule::do_ecan_load(Request *req,
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
    bool status = _cs.registerAgent(ArtificialStimulatorAgent::info().id,
                                    &artificialStimulatorAgentFactory);
    if (status) {
        _artificialstimulatoragentptr = _cs.createAgent(
                ArtificialStimulatorAgent::info().id, false);
    }

    return "Loaded the following agents:\n" + ECAN_EXP_AGENTS;
}

std::string ExperimentSetupModule::do_ecan_start(Request *req,
                                                 std::list<std::string> args)
{
    _cs.startAgent(_forgetting_agentptr);
    _cs.startAgent(_hebbianupdating_agentptr);
    _cs.startAgent(_importanceupdating_agentptr);
    _cs.startAgent(_simpleimportancediffusion_agentptr);
    _cs.startAgent(_artificialstimulatoragentptr);

    return "The following agents were started:\n" + ECAN_EXP_AGENTS;
}

std::string ExperimentSetupModule::do_ecan_pause(Request *req,
                                                 std::list<std::string> args)
{
    _cs.stopAgent(_forgetting_agentptr);
    _cs.stopAgent(_hebbianupdating_agentptr);
    _cs.stopAgent(_importanceupdating_agentptr);
    _cs.stopAgent(_simpleimportancediffusion_agentptr);

    _cs.stopAgent(_artificialstimulatoragentptr);

    return "The following agents were stopped:\n" + ECAN_EXP_AGENTS;
}

std::string ExperimentSetupModule::do_start_nlp_stimulate(
        Request *req, std::list<std::string> args)
{
    bool status = _cs.registerAgent(SentenceGenStimulateAgent::info().id,
                                    &sentenceGenStimulateFactory);
    if (status) {
        _sentencegenstim_agentptr = _cs.createAgent(
                SentenceGenStimulateAgent::info().id, false);
        _cs.startAgent(_sentencegenstim_agentptr);
        return "The following agents were started:\nopencog::SentenceGenStimulateAgent\n";
    }

    return "Unable to start the agent.\n";
}

std::string ExperimentSetupModule::do_pause_nlp_stimulate(
        Request *req, std::list<std::string> args)
{
    _cs.stopAgent(_sentencegenstim_agentptr);

    return "The following agents were stopped:\nopencog::SentenceGenStimulateAgent\n";
}

std::string ExperimentSetupModule::do_stimulate(Request *req,
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

std::string ExperimentSetupModule::do_dump_data(Request *req,
                                                std::list<std::string> args)
{
    std::string file_name = args.back();
    // cycle,uuid,sti,lti

    auto swprint = [this](const UnorderedHandleSet & uhs) {
        std::stringstream sstream;
        for (const auto & p : _data) {
            if(uhs.find(p.first) != uhs.end()) {
                for (const ECANValue& ev : p.second) {
                    sstream << std::to_string(p.first.value()) << ","
                    << std::to_string(ev._sti) << ","
                    << std::to_string(ev._lti) << ","
                    << std::to_string(ev._vlti) << ","
                    << std::to_string(ev._cycle) << "\n";
                }
            }
        }
        return sstream.str();
    };

    auto nswprint = [this](const UnorderedHandleSet & uhs) {
        std::stringstream sstream;
        for (const auto & p : _data) {
            if(uhs.find(p.first) == uhs.end()) {
                for (const ECANValue& ev : p.second) {
                    sstream << std::to_string(p.first.value()) << ","
                    << std::to_string(ev._sti) << ","
                    << std::to_string(ev._lti) << ","
                    << std::to_string(ev._vlti) << ","
                    << std::to_string(ev._cycle) << "\n";
                }
            }
        }
        return sstream.str();
    };

    std::ofstream outf(file_name+"-sw.data",
                       std::ofstream::out | std::ofstream::trunc);
    //Print ecan  values of special word nodes
    outf << swprint(hspecial_word_nodes);
    outf.flush();
    outf.close();

    std::ofstream outf2(file_name+"-nsw.data",
                         std::ofstream::out | std::ofstream::trunc);
    outf2 << nswprint(hspecial_word_nodes);
    outf2.flush();
    outf2.close();

    return "Time series data dumped in to " + file_name + ".\n";

}

std::string ExperimentSetupModule::do_load_word_dict(
        Request *req, std::list<std::string> args)
{
    auto tokenize = [](const std::string& str,const char& delim) {
        std::vector<std::string> words;
        auto init = str.begin();
        for (auto i = str.begin(); i != str.end(); ++i) {
            if (*i == delim) {
                std::string word = str.substr(
                        init - str.begin(), std::distance(init,i));
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

    special_words = tokenize(special_wstr, ',');
    nspecial_words = tokenize(nspecial_wstr, ',');
    sent_size = config().get_int("SENTENCE_SIZE");

    return "Loading successful.\n";
}


