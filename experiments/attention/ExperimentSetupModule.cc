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

#include <opencog/attention/atom_types.h>

#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/Module.h>

#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

#include "ExperimentSetupModule.h"

using namespace opencog;
using namespace opencog::ECANExperiment;
using namespace std::chrono;

DECLARE_MODULE(ExperimentSetupModule);

std::string ExperimentSetupModule::file_name;

std::vector<std::vector<std::string>> opencog::ECANExperiment::swords;
std::vector<std::string> opencog::ECANExperiment::words;

int opencog::ECANExperiment::current_group = 0;

int opencog::ECANExperiment::special_word_occurence_period = 1;

ExperimentSetupModule::ExperimentSetupModule(CogServer& cs) :
        Module(cs), _cs(cs)
{
    _log = new Logger(); //new Logger("ecanexpe.log");
    _log->fine("uuid,cycle,sti_old,sti_new,lti,vlti,wage,rent,agent_name");
    _as = &_cs.getAtomSpace();

    _AVChangedSignalConnection = _as->AVChangedSignal(
            boost::bind(&ExperimentSetupModule::AVChangedCBListener, this, _1,
                        _2, _3));
    _AVChangedSignalConnection = _as->TVChangedSignal(
            boost::bind(&ExperimentSetupModule::TVChangedCBListener, this, _1,
                        _2, _3));

    file_name = std::string(PROJECT_SOURCE_DIR)
                + "/experiments/attention/dump";

}

ExperimentSetupModule::~ExperimentSetupModule()
{
    unregisterAgentRequests();
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
                 << current_group << ","
                 << _as->get_attentional_focus_boundary() << "\n";
            outav.close();
        }
    }
}

void ExperimentSetupModule::TVChangedCBListener(const Handle& h,
                                                const TruthValuePtr& tv_old,
                                                const TruthValuePtr& tv_new)
{
    if (h->getType() == ASYMMETRIC_HEBBIAN_LINK) {
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
}

void ExperimentSetupModule::registerAgentRequests()
{
    do_start_exp_register();
    do_stop_exp_register();
}

void ExperimentSetupModule::unregisterAgentRequests()
{
    do_start_exp_unregister();
    do_stop_exp_unregister();
}
void ExperimentSetupModule::init(void)
{
    registerAgentRequests();

    _cs.registerAgent(SentenceGenStimulateAgent::info().id,
                                    &sentenceGenStimulateFactory);

    _sentencegenstim_agentptr = _cs.createAgent(
                SentenceGenStimulateAgent::info().id, false);
}

std::string ExperimentSetupModule::do_start_exp(Request *req,
                                                std::list<std::string> args)
{
    int groups;
    int groupsize;
    int wordcount;
    std::string output;
    if (args.size() != 3){
        groups = 10;
        groupsize = 10;
        wordcount = 100;
        output = output + "Using Default Configuration \n";
    } else {
        groups = std::stoi(args.front());
        args.pop_front();
        groupsize = std::stoi(args.front());
        args.pop_front();
        wordcount =  std::stoi(args.front());
    }
    genWords(groups,groupsize,wordcount);

    std::ofstream outf(file_name + "-av.data", std::ofstream::trunc);
    outf << groups << ","
         << groupsize << ","
         << wordcount << "\n";
    outf.flush();
    outf.close();

    remove((file_name + "-hebtv.data").c_str());

    _cs.startAgent(_sentencegenstim_agentptr);

    output = output + "Started opencog::SentenceGenStimulateAgent\n";

    return output;
}

std::string ExperimentSetupModule::do_stop_exp(Request *req,
                                                std::list<std::string> args) {

    _cs.stopAgent(_sentencegenstim_agentptr);

    return "Stoped Experiment \n";
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
