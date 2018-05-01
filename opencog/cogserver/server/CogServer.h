/*
 * opencog/cogserver/server/CogServer.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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
#include <memory>
#include <mutex>
#include <vector>
#include <thread>

#include <opencog/util/concurrent_queue.h>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/AgentRunnerBase.h>
#include <opencog/cogserver/server/AgentRunnerThread.h>
#include <opencog/cogserver/server/BaseServer.h>
#include <opencog/cogserver/server/Module.h>
#include <opencog/cogserver/server/NetworkServer.h>
#include <opencog/cogserver/server/SystemActivityTable.h>
#include <opencog/cogserver/server/Request.h>
#include <opencog/cogserver/server/Registry.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

/**
 * This class implements a network and agent server. It provides shared
 * network access to a given AtomSpace, together with some basic
 * capabilities to open a database, load C++ modules and to run
 * time-sliced processing "agents" called MindAgents.
 *
 * The module system is somewhat deprecated: python and scheme users
 * will typically find it more convenient to use the module system
 * provided by those langauges.  Thus, there is no requirement that new
 * C++ code must be wrapped into a CogServer module.
 *
 * Future versions of the CogServer might remove the module subsystem.
 *
 * The MindAgent subsystem is also somewhat obsolescent. It implements
 * a crude single-threaded cooperative time-slicing framework.  Most
 * users will simply find it easier to just create a new thread to run
 * whatever task that needs to be performed.
 *
 * Future versions of the CogServer might remove the agent subsystem.
 * Alternately, it might be useful to design a band-new thread
 * management subsystem. Maybe. Unclear.
 *
 * The most useful thing that the cogserver currently provides is
 * shared multi-user network access to a command line, from which
 * the python and scheme read-evaluate-print-loop (REPL) shells
 * can be accessed. This allows users to perform management and 
 * development on an atomspace by attaching to it from a remote
 * location.  In particular, the atomspace(s) are not killed when
 * the last user disconnects; the server will stay running in this
 * dettached state, until user reconnect, or until it is shut down.
 *
 * The network server is implemented by devoting one thread to listening
 * for tcp/ip socket connections. Upon connection, a new thread is
 * forked to handle user requests and the run the python/scheme shells.
 * Command-line commands are implemented as "Requests", described below.
 * The atomspace is thread-safe, as well as the Request queue, and the
 * python/scheme REPL shells, so there should be no issues with
 * multi-user access.
 *
 * Some (many?) of the C++ modules might not be thread-safe. 
 * None of the existing agents are thread-safe.
 *
 * -----------------------------------------------------------------
 * Implementation details:
 * 
 * Module management is the part responsible for extending the server
 * through the use of dynamically loadable libraries (or modules).
 * Valid modules must extended the class defined in Module.h and be
 * compiled and linked as a shared library. Currently, only Unix DSOs
 * are supported; Win32 DLLs are not. The server API itself provides
 * methods to load, unload and retrieve  modules. The server provides
 * modules with two entry points: the constructor, which is typically
 * invoked by the module's load function; and the 'init' method, which
 * is called after the module has been instantiated and its meta-data 
 * has been filled.
 *
 * Cycles are handled by the server's main loop (method 'serverLoop').
 * Each cycle has a minimum duration controlled by the parameter
 * "SERVER_CYCLE_DURATION" (in seconds). At the start of every cycle,
 * the server processes the queued requests and then executes an
 * interaction of each scheduled agent. When all the agents are finished,
 * the server sleeps for the remaining time until the end of the cycle.
 *
 * Agent management is done through inheritance from the Registry<Agent>
 * class.  The agent registry API provides several methods to:
 *   1. register, unregister and list agents classes;
 *   2. create and destroy agent instances;
 *   3. start and stop agents.
 * We chose to wrap the register methods in the server class to avoid
 * conflicts with the other registry inheritance (Registry<Command>).
 *
 * Request management uses the same Registry base template as the agent
 * subsystem -- only, this time, using the Request base class. Thus, the
 * functionalities provided are very similar:
 *   1. register, unregister and list request classes;
 *   2. create requests instances;
 *   3. push/pop from requests queue.
 * Contrary to the agent management, the lifecycle of each Request is
 * controlled by the server itself (that why no "destroyRequest" is
 * provided), which destroys the instance right after its execution.
 * That is, requests are assumed to run to completion in a small amount
 * of time, so that they don't block later requests.
 */
class CogServer : public BaseServer, public Registry<Agent>, public Registry<Request>
{
    AtomSpace* _private_as;

protected:

    // Define a map with the list of loaded modules.
    typedef struct {
        Module*                 module;
        std::string             id;
        std::string             filename;
        Module::LoadFunction*   loadFunction;
        Module::UnloadFunction* unloadFunction;
        void*                   handle;
    } ModuleData;
    typedef std::map<const std::string, ModuleData> ModuleMap;
    typedef std::unique_ptr<AgentRunnerThread> AgentRunnerThreadPtr;

    // Containers used to store references to the modules, requests
    // and agents
    ModuleMap modules;
    std::map<const std::string, Request*> requests;

    long cycleCount;
    bool running;
    // Used to start and stop the Agents loop via shell commands
    bool agentsRunning;

    SimpleRunner agentScheduler;
    std::vector<AgentRunnerThreadPtr> agentThreads;
    std::map<std::string, AgentRunnerThread*> threadNameMap;

    std::mutex processRequestsMutex;
    concurrent_queue<Request*> requestQueue;

    NetworkServer* _networkServer;

    SystemActivityTable _systemActivityTable;

public:

    /** CogServer's constructor. Initializes the mutex, atomspace
     * and cycleCount  variables*/
    CogServer(AtomSpace* = nullptr);

    /** Factory method. Override's the base class factory method
     * and returns an  instance of CogServer instead. */
    static BaseServer* createInstance(AtomSpace* = nullptr);

    /** CogServer's destructor. Disables the network server and
     * unloads all modules. */
    virtual ~CogServer(void);

    /** Server's main loop. Executed while the 'running' flag is set
     *  to true. It processes the request queue, then the scheduled
     *  agents and finally sleeps for the remaining time until the end
     * of the cycle (if any). */
    virtual void serverLoop(void);

    /** Runs a single server loop step. Quasi-private method, made
     *  public to be used in unit tests and for debug purposes only. */
    virtual void runLoopStep(void);

    /** Customized server loop run. This method is called inside serverLoop
     *  (between processing request queue and scheduled agents) and can be
     *  overwritten by CogServer's subclasses in order to customize the
     *  server loop behavior.
     *
     *  This method controls the execution of server cycles by returning
     *  'true' if the server must run a cycle and 'false' if it must not.
     *  By default it does nothing and returns true.
     *
     *  If EXTERNAL_TICK_MODE config parameter is enabled, serverLoop
     *  will not go sleep at all. So, in this case, this method must be
     *  in charge of going sleep when the server is idle to prevent
     *  a busy-wait loop. */
    virtual bool customLoopRun(void);

    /** Returns the number of executed cycles so far */
    virtual long getCycleCount(void);

    /** Terminates the main loop. The loop will be exited
     *  after the current interaction is finished. */
    virtual void stop(void);

    /** Starts the network server and adds the default command line
     *  server socket on the port specified by the configuration
     *  parameter SERVER_PORT */
    virtual void enableNetworkServer(int port=17001);

    /** Stops the network server and closes all the open server sockets. */
    virtual void disableNetworkServer(void);

    /** Returns a reference to the system activity table instance */
    virtual SystemActivityTable& systemActivityTable(void);

    /**** Module API ****/
    /** Loads a dynamic library/module. Takes the filename of the
     *  library (.so or .dylib or .dll). On Linux/Unix, the filename may
     *  be absolute or relative to the server's RPATH path (which
     *  typically, should be "INSTALL_PREFIX/lib/opencog") */
    virtual bool loadModule(const std::string& filename);

    /** Unloads a dynamic library/module. Takes the module's id, as
     *  defined in the Module base class and overriden by the derived
     *  module classes. See the documentation in the Module.h file for
     *  more details. */
    virtual bool unloadModule(const std::string& id);

    /** Lists the modules that are currently loaded. */
    virtual std::string listModules();

    /** Retrieves the module's meta-data (id, filename, load/unload
     * function pointers, etc). Takes the module's id */
    virtual ModuleData getModuleData(const std::string& id);

    /** Retrieves the module's instance. Takes the module's id */
    virtual Module* getModule(const std::string& id);

    /** Load all modules specified in configuration file. If
        module_paths is empty then DEFAULT_MODULE_PATHS is used
        instead, which is why it is passed as copy instead of const
        ref. */
    virtual void loadModules(std::vector<std::string> module_paths =
                             std::vector<std::string>());

    /**** Agent Registry API ****/
    /** Register a new agent class/type. Takes the class' id and a
     *  derived factory for this particular agent type. (note: the
     *  caller owns the factory instance). */
    virtual bool registerAgent(const std::string& id, AbstractFactory<Agent> const* factory);

    /** Unregister an agent class/type. Takes the class' id. */
    virtual bool unregisterAgent(const std::string& id);

    /** Returns a list with the ids of all the registered agent classes. */
    virtual std::list<const char*> agentIds(void) const;

    /** Returns a list of all the currently running agent instances. */
    virtual AgentSeq runningAgents(void);

    /** Creates and returns a new instance of an agent of class 'id'.
     *  If 'start' is true, then the agent will be automatically added
     *  to the list of scheduled agents. */
    virtual AgentPtr createAgent(const std::string& id,
                                 const bool start = false);

    /// Same as above, but returns the correct type.
    template <typename T>
    std::shared_ptr<T> createAgent(const bool start = false) {
        return std::dynamic_pointer_cast<T>(createAgent(T::info().id, start));
    }

    /** Adds agent 'a' to the list of scheduled agents. */
    virtual void startAgent(AgentPtr a, bool dedicated_thread = false,
        const std::string &thread_name = "");

    /** Removes agent 'a' from the list of scheduled agents. */
    virtual void stopAgent(AgentPtr a);

    /** Destroys all agents from class 'id' */
    virtual void stopAllAgents(const std::string& id);

    /** Starts running agents as part of the serverLoop (enabled by default) */
    virtual void startAgentLoop(void);

    /** Stops running agents as part of the serverLoop */
    virtual void stopAgentLoop(void);

    /**** Request Registry API ****/
    /** Register a new request class/type. Takes the class id and a derived
     *  factory for this particular request type. (note: the caller owns the
     *  factory instance). */
    virtual bool registerRequest(const std::string& id, AbstractFactory<Request> const* factory);

    /** Unregister a request class/type. Takes the class' id. */
    virtual bool unregisterRequest(const std::string& id);

    /** Returns a list with the ids of all the registered request classes. */
    virtual std::list<const char*> requestIds(void) const;

    /** Creates and returns a new instance of a request of class 'id'. */
    virtual Request* createRequest(const std::string& id);

    /** Returns the class metadata from request class 'id'. */
    virtual const RequestClassInfo& requestInfo(const std::string& id) const;

    /**
     * Adds request to the end of the requests queue.
     * Caution: after this push, the request might be executed and deleted
     * in a different thread, and so it must NOT be referenced after the push!
     */
    void pushRequest(Request* request) { requestQueue.push(request); }

    /** Removes and returns the first request from the requests queue. */
    Request* popRequest(void) { return requestQueue.pop(); }

    /** Returns the requests queue size. */
    int getRequestQueueSize(void) { return requestQueue.size(); }

    /** Force drain of all outstanding requests */
    void processRequests(void);

    /** Return the logger */
    Logger &logger(void);

}; // class

// Handy dandy utility
inline CogServer& cogserver(AtomSpace* as = nullptr)
{
    return dynamic_cast<CogServer&>(server(CogServer::createInstance, as));
}

/** @}*/
}  // namespace

#endif // _OPENCOG_COGSERVER_H
