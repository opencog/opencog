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

namespace opencog {

class AtomSpace;
class CogServer;
class Module;
class Logger;

namespace ECANExperiment {

static std::string ECAN_EXP_AGENTS = "opencog::HebbianUpdatingAgent\n"
                                     "opencog::ImportanceUpdatingAgent\n"
                                     "opencog::SimpleImportanceDiffusionAgent\n"
                                     "opencog::ForgettingAgent\n"
                                     "opencog::ArtificilaStimulatorAgent\n"
                                     "opencog::SurprisingnessEvaluatorAgent\n";

class PLNDynamicsExpSetupModule: public Module {
private:
    struct ECANValue {
        AttentionValue::sti_t _sti;
        AttentionValue::lti_t _lti;
        long  _cycle;

        ECANValue():_sti(0), _lti(0), _cycle(0){}

        ECANValue(AttentionValue::sti_t sti, AttentionValue::lti_t lti,
                  long cycle) :
                _sti(sti), _lti(lti), _cycle(cycle)
        {
        }
    };

    AgentPtr _forgetting_agentptr;
    AgentPtr _hebbianupdating_agentptr;
    AgentPtr _importanceupdating_agentptr;
    AgentPtr _simpleimportancediffusion_agentptr;

    AgentPtr _surprisingness_agentptr;
    AgentPtr _artificialstimulatoragentptr;

    AtomSpace * _as;
    CogServer& _cs;
    Logger * _log;

    std::map<Handle, ECANValue> _data;
    boost::signals2::connection _AVChangedSignalConnection;

    void AVChangedCBListener(const Handle& h, const AttentionValuePtr& av_old,
                             const AttentionValuePtr& av_new);


    //Start stop ECAN agents commands
    DECLARE_CMD_REQUEST(PLNDynamicsExpSetupModule, "ecan-load", do_ecan_load,
            "Loads all agents of ECAN and experiment\n",
            "Usage: ecan_load\n"
            "Loads the following ECAN agents\n"+ECAN_EXP_AGENTS
            ,
            false, true)

    DECLARE_CMD_REQUEST(PLNDynamicsExpSetupModule, "ecan-start", do_ecan_start,
            "Starts main ECAN agents\n",
            "Usage: ecan_start\n"
            "Starts the following ECAN agents\n"+ECAN_EXP_AGENTS
            ,
            false, true)

    DECLARE_CMD_REQUEST(PLNDynamicsExpSetupModule, "ecan-pause", do_ecan_pause,
            "Pause main ECAN agents\n",
            "Usage: ecan_pause\n"
            "Stops the following ECAN agents\n"+ECAN_EXP_AGENTS
            ,
            true, true)

    //For manually stimulating some atoms by referring them via their UUID
    DECLARE_CMD_REQUEST(PLNDynamicsExpSetupModule, "stimulate-atoms", do_stimulate,
            "Stimulate an atom or set of atoms\n",
            "Usage: stimulate_atoms UUID,STIMULUS \n"
            "Stimulates atoms with certain stimulus value\n"
            "arguments are a pair of uuid and stimulus vaue\n"
            "separated by comma and each pair separated by space\n",
            false, true)

    //Dump all STI,LTI changes recorded till now to a file.
    DECLARE_CMD_REQUEST(PLNDynamicsExpSetupModule, "dump-tseries", do_dump_data,
            "Dumps ECAN time series data to file\n",
            "Usage: dump-tseries [FILE_NAME]\n"
            "Dump time serise ECAN data\n",
            false, true)

    void registerAgentRequests();
    void unregisterAgentRequests();

public:
    PLNDynamicsExpSetupModule(CogServer&);
    virtual ~PLNDynamicsExpSetupModule();
    static inline const char* id();
    virtual void init(void);
};
}

} /* namespace opencog */

#endif /* EXPERIMENTSETUPMODULE_H_ */
