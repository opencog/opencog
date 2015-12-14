/*
 * PLNDynamicsExpSetupModule.h
 *
 *  Created on: Apr 19, 2015
 *      Author: misgana
 */

#ifndef PLNDYNAMICSEXPSETUPMODULE_H_
#define PLNDYNAMICSEXPSETUPMODULE_H_

#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>

#include "ArtificialStimulatorAgent.h"
#include "SentenceGenStimulateAgent.h"

namespace opencog {

class AtomSpace;
class CogServer;
class SchemeEval;
class Module;
class Logger;

namespace ECANExperiment {

static std::string ECAN_EXP_AGENTS = "opencog::SimpleHebbianUpdatingAgent\n"
                                     "opencog::ImportanceUpdatingAgent\n"
                                     "opencog::SimpleImportanceDiffusionAgent\n"
                                     "opencog::ForgettingAgent\n"
                                     "opencog::ArtificilaStimulatorAgent\n";
                                     //"opencog::SurprisingnessEvaluatorAgent\n";

class ExperimentSetupModule: public Module {
private:
    struct AValues {
        AttentionValue::sti_t _sti;
        AttentionValue::lti_t _lti;
        AttentionValue::vlti_t _vlti;
        long _cycle;

        AValues() :
                _sti(0), _lti(0),_vlti(0), _cycle(0)
        {
        }

        AValues(AttentionValue::sti_t sti, AttentionValue::lti_t lti,
                  AttentionValue::vlti_t vlti, long cycle) :
                _sti(sti), _lti(lti), _vlti(vlti), _cycle(cycle)
        {
        }
    };

    struct HebTValues {
        strength_t _strength;
        confidence_t _confidence;
        long _cycle;

        HebTValues() :
                _strength(0.0), _confidence(0.0), _cycle(0)
        {
        }

        HebTValues(strength_t strength, confidence_t confidence, long cycle) :
                _strength(strength), _confidence(confidence), _cycle(cycle)
        {
        }
    };

    AgentPtr _forgetting_agentptr;
    AgentPtr _hebbianupdating_agentptr;
    AgentPtr _importanceupdating_agentptr;
    AgentPtr _simpleimportancediffusion_agentptr;

    AgentPtr _sentencegenstim_agentptr;
    AgentPtr _artificialstimulatoragentptr;

    Factory<ArtificialStimulatorAgent, Agent> artificialStimulatorAgentFactory;
    Factory<SentenceGenStimulateAgent, Agent> sentenceGenStimulateFactory;


    AtomSpace * _as;
    CogServer& _cs;
    Logger * _log;
    SchemeEval * _scmeval;

    static std::map<Handle, std::vector<AValues>> _av_data;
    static std::map<Handle, std::vector<HebTValues>> _hebtv_data;

    boost::signals2::connection _AVChangedSignalConnection,_TVChangedSignalConnection;

    void AVChangedCBListener(const Handle& h, const AttentionValuePtr& av_old,
                             const AttentionValuePtr& av_new);

    void TVChangedCBListener(const Handle& h, const TruthValuePtr& av_old,
                             const TruthValuePtr& tv_new);

    //Start stop ECAN agents commands
    DECLARE_CMD_REQUEST(ExperimentSetupModule, "ecan-load", do_ecan_load,
            "Loads all agents of ECAN and experiment\n",
            "Usage: ecan-load\n"
            "Loads the following ECAN agents\n"+ECAN_EXP_AGENTS
            ,
            false, true)

    DECLARE_CMD_REQUEST(ExperimentSetupModule, "ecan-start", do_ecan_start,
            "Starts main ECAN agents\n",
            "Usage: ecan-start\n"
            "Starts the following ECAN agents\n"+ECAN_EXP_AGENTS,
            false, true)

    DECLARE_CMD_REQUEST(ExperimentSetupModule, "ecan-pause", do_ecan_pause,
            "Pause main ECAN agents\n",
            "Usage: ecan-pause\n"
            "Stops the following ECAN agents\n"+ECAN_EXP_AGENTS,
            true, true)

    DECLARE_CMD_REQUEST(ExperimentSetupModule, "start-word-stimulator",do_start_nlp_stimulate,
            "Starts NLP atoms stimulator agent\n",
            "Usage: start-nlp-stimulator\n"
            "Starts the following ECAN agents\nopencog::SentenceGenStimulateAgent",
            false, true)

    DECLARE_CMD_REQUEST(ExperimentSetupModule, "pause-word-stimulator",do_pause_nlp_stimulate,
            "Pauses NLP atoms stimulator agent\n",
            "Usage: pause-nlp-stimulator\n"
            "Starts the following ECAN agents\nopencog::SentenceGenStimulateAgent",
            false, true)


    //For manually stimulating some atoms by referring them via their UUID
    DECLARE_CMD_REQUEST(ExperimentSetupModule, "stimulate-atoms", do_stimulate,
            "Stimulate an atom or set of atoms\n",
            "Usage: stimulate-atoms UUID,STIMULUS \n"
            "Stimulates atoms with certain stimulus value\n"
            "arguments are a pair of uuid and stimulus vaue\n"
            "separated by comma and each pair separated by space\n",
            false, true)

    //Dump all STI,LTI changes recorded till now to a file.
    DECLARE_CMD_REQUEST(ExperimentSetupModule, "dump", do_dump_data,
            "Dumps ECAN time series data to file\n",
            "Usage: dump [av|heb] [FILE_NAME]\n"
            "Dump time serise ECAN data\n",
            false, true)

    //Load word dict.
    DECLARE_CMD_REQUEST(ExperimentSetupModule, "load-word-dict",do_load_word_dict,
            "Loads word dict for experimentation. The word dict file should have two sections\n"
            "SPEICAL_WORDS = [comma separated words]\n"
            "NON_SPECIAL_WORDS = [comma separated words] size\n",
            "Usage: load-word-dict [FILE_NAME]\n"
            "Dump time serise ECAN data\n",
            false, true)

    void registerAgentRequests();
    void unregisterAgentRequests();

public:
    ExperimentSetupModule(CogServer&);
    virtual ~ExperimentSetupModule();
    static inline const char* id();
    virtual void init(void);

    static std::string dump_ecan_data(std::string what_to_dump, std::string file_name);
};
}

} /* namespace opencog */

#endif /* EXPERIMENTSETUPMODULE_H_ */
