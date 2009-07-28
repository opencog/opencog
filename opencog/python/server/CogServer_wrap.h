#ifndef _OPENCOG_COGSERVER_WRAP_H
#define _OPENCOG_COGSERVER_WRAP_H

#include <boost/python/wrapper.hpp>

#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>

using namespace boost::python;
using namespace opencog;

/** Exposes the CogServer class. */
void init_CogServer_py();

/** A class wrapper of the CogServer class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct CogServerWrap : CogServer, wrapper<CogServer>
{
    // Non-pure virtual functions.

    void runAgent(Agent *agent);
    void default_runAgent(Agent *agent);
    /*void serverLoop(void);
    void default_serverLoop(void);
    void runLoopStep(void);
    void default_runLoopStep(void);
    bool customLoopRun(void);
    bool default_customLoopRun(void);
    long getCycleCount(void);
    long default_getCycleCount(void);
    void stop(void);
    void default_stop(void);
    void enableNetworkServer(void);
    void default_enableNetworkServer(void);
    void disableNetworkServer(void);
    void default_disableNetworkServer(void);
    NetworkServer& networkServer(void);
    NetworkServer& default_networkServer(void);
    SystemActivityTable& systemActivityTable(void);
    SystemActivityTable& default_systemActivityTable(void);
    bool loadModule(const std::string& filename);
    bool default_loadModule(const std::string& filename);
    bool unloadModule(const std::string& id);
    bool default_unloadModule(const std::string& id);
    ModuleData getModuleData(const std::string& id);
    ModuleData default_getModuleData(const std::string& id);
    Module* getModule(const std::string& id);
    Module* default_getModule(const std::string& id);
    void loadModules();
    void default_loadModules();
    void loadSCMModules(const char* [] = NULL);
    void default_loadSCMModules(const char* [] = NULL);*/
    bool registerAgent(const std::string& id, AbstractFactory<Agent> const* factory);
    bool default_registerAgent(const std::string& id, AbstractFactory<Agent> const* factory);
    bool unregisterAgent(const std::string& id);
    bool default_unregisterAgent(const std::string& id);
    /*std::list<const char*> agentIds(void) const;
    std::list<const char*> default_agentIds(void) const;
    const AgentSeq &runningAgents(void);
    const AgentSeq &default_runningAgents(void);
    Agent* createAgent(const std::string& id, const bool start = false);
    Agent* default_createAgent(const std::string& id, const bool start = false);
    void startAgent(Agent* a);
    void default_startAgent(Agent* a);
    void stopAgent(Agent* a);
    void default_stopAgent(Agent* a);
    void destroyAgent(Agent* a);
    void default_destroyAgent(Agent* a);
    void destroyAllAgents(const std::string& id);
    void default_destroyAllAgents(const std::string& id);
    void startAgentLoop(void);
    void default_startAgentLoop(void);
    void stopAgentLoop(void);
    void default_stopAgentLoop(void);*/
    bool registerRequest(const std::string& id, AbstractFactory<Request> const* factory);
    bool default_registerRequest(const std::string& id, AbstractFactory<Request> const* factory);
    bool unregisterRequest(const std::string& id);
    bool default_unregisterRequest(const std::string& id);
    std::list<const char*> requestIds(void) const;
    std::list<const char*> default_requestIds(void) const;
    Request* createRequest(const std::string& id);
    Request* default_createRequest(const std::string& id);
    const RequestClassInfo& requestInfo(const std::string& id) const;
    const RequestClassInfo& default_requestInfo(const std::string& id) const;
    /*void pushRequest(Request* request);
    void default_pushRequest(Request* request);
    Request* popRequest(void);
    Request* default_popRequest(void);*/
    int getRequestQueueSize(void);
    int default_getRequestQueueSize(void);
};

#endif // _OPENCOG_COGSERVER_WRAP_H
