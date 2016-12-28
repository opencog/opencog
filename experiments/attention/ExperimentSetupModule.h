/*
 * PLNDynamicsExpSetupModule.h
 *
 *  Created on: Apr 19, 2015
 *      Author: misgana
 */

#ifndef PLNDYNAMICSEXPSETUPMODULE_H_
#define PLNDYNAMICSEXPSETUPMODULE_H_

#include "SentenceGenStimulateAgent.h"
#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/attention/AttentionModule.h>

namespace opencog {

class AtomSpace;
class CogServer;
class SchemeEval;
class Module;
class Logger;

namespace ECANExperiment {

extern std::vector<std::vector<std::string>> swords;
extern std::vector<std::string> words;

extern int special_word_occurence_period;

extern int current_group;

class ExperimentSetupModule: public Module {
private:
    AgentPtr _sentencegenstim_agentptr;

    Factory<SentenceGenStimulateAgent, Agent> sentenceGenStimulateFactory;

    AtomSpace * _as;
    CogServer& _cs;
    Logger * _log;

    boost::signals2::connection _AVChangedSignalConnection,_TVChangedSignalConnection,_AtomAddedSignalConnection;

    void AVChangedCBListener(const Handle& h, const AttentionValuePtr& av_old,
                             const AttentionValuePtr& av_new);

    void TVChangedCBListener(const Handle& h, const TruthValuePtr& av_old,
                             const TruthValuePtr& tv_new);

    //Load word dict.
    DECLARE_CMD_REQUEST(ExperimentSetupModule, "start-exp",do_start_exp,
            "Loads word dict for experimentation. The word dict file should have two sections\n"
            "SPEICAL_WORDS = [comma separated words]\n"
            "NON_SPECIAL_WORDS = [comma separated words] size\n",
            "Usage: load-word-dict [FILE_NAME]\n"
            "Dump time serise ECAN data\n",
            false, true)

    DECLARE_CMD_REQUEST(ExperimentSetupModule, "stop-exp",do_stop_exp,
            "Loads word dict for experimentation. The word dict file should have two sections\n"
            "SPEICAL_WORDS = [comma separated words]\n"
            "NON_SPECIAL_WORDS = [comma separated words] size\n",
            "Usage: load-word-dict [FILE_NAME]\n"
            "Dump time serise ECAN data\n",
            false, true)


    void registerAgentRequests();
    void unregisterAgentRequests();

    void genWords(int groups,int groupsize,int nonspecial);

public:
    ExperimentSetupModule(CogServer&);
    virtual ~ExperimentSetupModule();
    static inline const char* id();
    virtual void init(void);

    std::string dump_ecan_data();
    static std::string file_name;

  //std::ofstream outav;
  //std::ofstream outheb;

};
}

} /* namespace opencog */

#endif /* EXPERIMENTSETUPMODULE_H_ */
