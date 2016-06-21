/*
 * ExperimentSetupModule.cc
 *
 *  Created on: Apr 19, 2015
 *      Author: misgana
 */

#include <sstream>
#include <string>
#include <fstream>

#include <stdio.h>
#include <chrono>
#include <thread>

#include <boost/algorithm/string/predicate.hpp>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemeEval.h>

#include <opencog/attention/ForgettingAgent.h>
//#include <opencog/attention/StochasticImportanceDiffusionAgent.h>
//#include <opencog/attention/StochasticImportanceUpdatingAgent.h>
//#include <opencog/attention/MinMaxSTIUpdatingAgent.h>
#include <opencog/attention/atom_types.h>
#include <opencog/attention/experiment/SmokesDBFCAgent.h>

#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/Module.h>

#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

#include "ArtificialStimulatorAgent.h"
#include "ExperimentSetupModule.h"

using namespace opencog;
using namespace opencog::ECANExperiment;
using namespace std::chrono;

DECLARE_MODULE(ExperimentSetupModule);

std::map<Handle, std::vector<AValues>> ExperimentSetupModule::_av_data;
std::map<Handle, std::vector<TValues>> ExperimentSetupModule::_hebtv_data;
std::map<Handle, std::vector<TValues>>  ExperimentSetupModule::_hascancer_tv_data;

std::string ExperimentSetupModule::file_name;

ExperimentSetupModule::ExperimentSetupModule(CogServer& cs) :
        Module(cs), _cs(cs)
{
    _log = new Logger(); //new Logger("ecanexpe.log");
    _log->fine("uuid,cycle,sti_old,sti_new,lti,vlti,wage,rent,agent_name");
    _as = &_cs.getAtomSpace();
    am = (AttentionModule*)_cs.getModule(AttentionModule::id_function_name());

    _scmeval = new SchemeEval(_as);
    _scmeval->eval("(add-to-load-path \"/usr/local/share/opencog/scm\")");
    _scmeval->eval("(use-modules  (opencog)");

    _AVChangedSignalConnection = _as->AVChangedSignal(
            boost::bind(&ExperimentSetupModule::AVChangedCBListener, this, _1,
                        _2, _3));
    _AVChangedSignalConnection = _as->TVChangedSignal(
            boost::bind(&ExperimentSetupModule::TVChangedCBListener, this, _1,
                        _2, _3));

    _AtomAddedSignalConnection = _as->addAtomSignal(
            boost::bind(&ExperimentSetupModule::AtomAddedCBListener, this, _1));

    file_name = std::string(PROJECT_SOURCE_DIR)
                + "/opencog/attention/experiment/visualization/dump";

}

ExperimentSetupModule::~ExperimentSetupModule()
{
    unregisterAgentRequests();
    //delete (_scmeval);
}

void ExperimentSetupModule::AVChangedCBListener(const Handle& h,
                                                const AttentionValuePtr& av_old,
                                                const AttentionValuePtr& av_new)
{
    NodePtr node;
    std::string name;

    if (h->isNode()) {
        node = NodeCast(h);
        name = node->getName();
        if (name.find("@") == std::string::npos)
        {
            std::ofstream outav(file_name + "-av.data", std::ofstream::app);
            outav << name << ","
                 << av_new->getSTI() << ","
                 << av_new->getLTI() << ","
                 << av_new->getVLTI() << ","
                 << system_clock::now().time_since_epoch().count() << ","
                 << current_group << "\n";
            outav.close();
        }
    }
}

void ExperimentSetupModule::TVChangedCBListener(const Handle& h,
                                                const TruthValuePtr& tv_old,
                                                const TruthValuePtr& tv_new)
{
    const Type t = h->getType();
    // For the experiment that aims at checking the sanity of the ECAN system.
    if (t == ASYMMETRIC_HEBBIAN_LINK) {
        HandleSeq outg = LinkCast(h)->getOutgoingSet();
        assert(outg.size() == 2);

        if (!outg[0]->isNode() or !outg[1]->isNode())
            return;
        std::string nn0 = NodeCast(outg[0])->getName();
        std::string nn1 = NodeCast(outg[1])->getName();

        if (boost::starts_with(nn0, "group") and boost::starts_with(nn1, "group")
            and !boost::contains(nn0,"@") and !boost::contains(nn1,"@"))
        {
            std::ofstream outheb(file_name + "-hebtv.data", std::ofstream::app);
            outheb << h.value() << ","
                   << nn0 << ","
                   << nn1 << ","
                   << tv_new->getMean() << ","
                   << tv_new->getConfidence() << ","
                   << system_clock::now().time_since_epoch().count() << "\n";
            outheb.close();
        }
    }
    // For the smokes FC experiment ( i.e Attention guided inference experiment with tuffy smokes database)
    else if (t == EVALUATION_LINK and is_cancer_reln(h)) {
        TValues hebtvv(tv_new->getMean(), tv_new->getConfidence(),
                       _cs.getCycleCount());
        _hascancer_tv_data[h].push_back(hebtvv);
    }

}

void ExperimentSetupModule::AtomAddedCBListener(const Handle& h)
{
    if (h->getType() == EVALUATION_LINK and is_cancer_reln(h)) {
        TruthValuePtr tv = h->getTruthValue();
        TValues tvs(tv->getMean(), tv->getConfidence(), _cs.getCycleCount());
        _hascancer_tv_data[h].push_back(tvs);
    }
}

void ExperimentSetupModule::registerAgentRequests()
{
    do_ecan_load_register();
    do_ecan_start_register();
    do_ecan_pause_register();
    do_stimulate_register();
    do_dump_data_register();
    do_start_nlp_stimulate_register();
    do_pause_nlp_stimulate_register();
    do_start_exp_register();
    do_stop_exp_register();
}

void ExperimentSetupModule::unregisterAgentRequests()
{
    do_ecan_load_unregister();
    do_ecan_start_unregister();
    do_ecan_pause_unregister();
    do_stimulate_unregister();
    do_dump_data_unregister();
    do_start_nlp_stimulate_unregister();
    do_pause_nlp_stimulate_unregister();
    do_stop_exp_unregister();
}
void ExperimentSetupModule::init(void)
{
    //Load params
    //_as->set_attentional_focus_boundary((stim_t) config().get_int("AF_BOUNDARY"));
    std::cout << "AF_BOUNDARY = " << _as->get_attentional_focus_boundary()
              << std::endl;
    registerAgentRequests();
}

std::string ExperimentSetupModule::do_start_exp(Request *req,
                                                std::list<std::string> args)
{
    int groups = std::stoi(args.front());
    args.pop_front();
    int groupsize = std::stoi(args.front());
    args.pop_front();
    int wordcount =  std::stoi(args.front());
    args.pop_front();
    int assize =  std::stoi(args.front());
    genWords(groups,groupsize,wordcount);

    std::ofstream outf(file_name + "-av.data", std::ofstream::trunc);
    outf << groups << ","
         << groupsize << ","
         << wordcount << "\n";
    outf.flush();
    outf.close();

    remove((file_name + "-hebtv.data").c_str());

    ForgettingAgent::maxSize = assize;

    //do_ecan_load(req,args);
    //do_ecan_start(req,args);
    do_start_nlp_stimulate(req,args);
    return "Started Experiment\n";
}


std::string ExperimentSetupModule::do_stop_exp(Request *req,
                                                std::list<std::string> args) {

    do_pause_nlp_stimulate(req,args);

    ForgettingAgent::maxSize = 0;

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    printf("STI in Bank: %ld \n",_as->get_STI_funds());

    return "Stoped Experiment \n";
}

std::string ExperimentSetupModule::do_ecan_load(Request *req,
                                                std::list<std::string> args)
{
    //am->createAgents();

    //Register experiment specific agents. Add more if you have here.
    if (_cs.registerAgent(ArtificialStimulatorAgent::info().id,
                          &artificialStimulatorAgentFactory)) {
        _artificialstimulatoragentptr = _cs.createAgent(
                ArtificialStimulatorAgent::info().id, false);
    }

    if (_cs.registerAgent(SmokesDBFCAgent::info().id, &smokesFCAgnetFactory)) {
      _smokes_fc_agentptr = _cs.createAgent(SmokesDBFCAgent::info().id,false);
    }

    return "Loaded the following agents:\n" + ECAN_EXP_AGENTS;
}

std::string ExperimentSetupModule::do_ecan_start(Request *req,
                                                 std::list<std::string> args)
{
    //am->do_start_ecan(req,args);

    _cs.startAgent(_artificialstimulatoragentptr);
    _cs.startAgent(_smokes_fc_agentptr);

    return "The following agents were started:\n" + ECAN_EXP_AGENTS;
}

std::string ExperimentSetupModule::do_ecan_pause(Request *req,
                                                 std::list<std::string> args)
{
    am->pause_ecan();

    //_cs.stopAgent(_artificialstimulatoragentptr);
    //_cs.stopAgent(_smokes_fc_agentptr);

    return "The following agents were stopped:\n" + ECAN_EXP_AGENTS;
}

std::string ExperimentSetupModule::do_start_nlp_stimulate(
        Request *req, std::list<std::string> args)
{
    bool status = _cs.registerAgent(SentenceGenStimulateAgent::info().id,
                                    &sentenceGenStimulateFactory);
    if (status)
    {


        _sentencegenstim_agentptr = _cs.createAgent(
                SentenceGenStimulateAgent::info().id, false);
        _cs.startAgent(_sentencegenstim_agentptr);

        return "The following agents were started:\nopencog::SentenceGenStimulateAgent\n";
    }

    return "Unable to start the agent.\n";
}

void ExperimentSetupModule::genWords(int groups,int groupsize,int nonspecial)
{
    swords.assign(groups,std::vector<std::string>(groupsize));

    int i = 0;
    int j = 0;

    for (auto &sv : swords) {
        for (std::string &word : sv) {
            word =  "group" + std::to_string(i) + "word" + std::to_string(j++);
        }
        i++;
        j=0;
    }

    words.assign(nonspecial,"");
    i = 0;

    for (std::string &word : words)
        word = "nonspecial" + std::to_string(i++);
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
    ExperimentSetupModule::dump_ecan_data();

    return "Data is dumped.";
}

std::string ExperimentSetupModule::dump_ecan_data()
{
    auto tvprint = [=](const std::map<Handle, std::vector<TValues>>& tvs) {
        std::stringstream sstream;
        for (const auto & p : tvs) {
            for (const TValues& hebtv : p.second) {
                sstream << std::to_string(p.first.value()) << ","
                << hebtv._strength << ","
                << hebtv._confidence << ","
                << hebtv._cycle<< "\n";
            }
        }
        return sstream.str();
    };

    std::ofstream outf(file_name + "-hascancer.data",
                       std::ofstream::out | std::ofstream::trunc);
    //Print ecan  values of special word nodes
    outf << tvprint(_hascancer_tv_data);
    outf.flush();
    outf.close();
    return "HasCancer TV dumped in to " + file_name + ".\n";
}
