/*
 * opencog/server/CogServer.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
 *            Gustavo Gama <gama@vettalabs.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_COGSERVER_H
#define _OPENCOG_COGSERVER_H

#include <map>
#include <queue>
#include <vector>
#include <tr1/memory>

#include <pthread.h>

#include <opencog/server/Agent.h>
#include <opencog/server/BaseServer.h>
#include <opencog/server/CogServerRequest.h>
#include <opencog/server/Module.h>
#include <opencog/server/NetworkServer.h>
#include <opencog/server/Registry.h>
#include <opencog/server/RequestProcessor.h>

namespace opencog
{

class CogServer : public BaseServer, public Registry<Agent>, public Registry<Request>
{

// forward declarations
class CogServerRequest;

protected:

    // define a map with the list of loaded modules
    //typedef std::pair<Module*, Module::UnloadFunction*> ModuleData;
    typedef struct {
        Module*                 module;
        std::string             id;
        std::string             filename;
        Module::LoadFunction*   loadFunction;
        Module::UnloadFunction* unloadFunction;
        void*                   handle;
    } ModuleData;
    typedef std::map<const std::string, ModuleData> ModuleMap;

    ModuleMap modules;
    std::map< const std::string, Request* > requests;
    std::vector<Agent*> agents;

    long cycleCount;
    bool running;

    void processAgents();
    void processRequests();

    pthread_mutex_t messageQueueLock;
    std::queue< Request* > requestQueue;

    NetworkServer _networkServer;

public:

    CogServer(void);
    static CogServer* createInstance(void);

    virtual ~CogServer(void);

    virtual void serverLoop(void);
    virtual long getCycleCount(void);
    virtual void stop(void);

    virtual void enableNetworkServer(void);
    virtual void disableNetworkServer(void);
    virtual NetworkServer& networkServer(void);

    // load/unload modules
    virtual bool loadModule(const std::string& filename);
    virtual bool unloadModule(const std::string& id);
    virtual ModuleData getModuleData(const std::string& id);
    virtual Module* getModule(const std::string& id);

    // register/unregister/instantiate agents
    virtual bool registerAgent(const std::string& id, AbstractFactory<Agent> const* factory);
    virtual bool unregisterAgent(const std::string& id);
    virtual std::list<const char*> agentIds(void) const;
    virtual Agent* createAgent(const std::string& id, const bool start = false);
    virtual void startAgent(Agent*);
    virtual void stopAgent(Agent*);
    virtual void destroyAgent(Agent*);
    virtual void destroyAllAgents(const std::string& typeinfo_name);

    // register/unregister/instantiate requests
    virtual bool registerRequest(const std::string& name, AbstractFactory<Request> const* factory);
    virtual bool unregisterRequest(const std::string& name);
    virtual Request* createRequest(const std::string& name);
    virtual const RequestClassInfo& requestInfo(const std::string& name) const;
    virtual std::list<const char*> requestIds(void) const;
    virtual Request* popRequest(void);
    virtual void pushRequest(Request* request);
    virtual int getRequestQueueSize(void);

    // used for debug purposes in unit tests
    virtual void unitTestServerLoop(int limitNumberOfCycles);

}; // class

}  // namespace

#endif // _OPENCOG_COGSERVER_H
