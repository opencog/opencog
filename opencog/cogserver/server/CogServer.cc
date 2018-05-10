/*
 * opencog/cogserver/server/CogServer.cc
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

#include <time.h>
#ifdef WIN32
#include <winsock2.h>
#else
#include <dlfcn.h>
#include <unistd.h>
#include <sys/time.h>
#endif

#include <boost/filesystem/operations.hpp>

#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/files.h>
#include <opencog/util/misc.h>
#include <opencog/util/platform.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/AttentionBank.h>

#ifdef HAVE_CYTHON
#include <opencog/cython/PythonEval.h>
#endif

#include <opencog/guile/SchemeEval.h>

#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/ConsoleSocket.h>
#include <opencog/cogserver/server/NetworkServer.h>
#include <opencog/cogserver/server/SystemActivityTable.h>
#include <opencog/cogserver/server/Request.h>

#include "CogServer.h"
#include "BaseServer.h"

using namespace opencog;

namespace opencog {
struct equal_to_id :
    public std::binary_function<AgentPtr, const std::string&, bool>
{
    bool operator()(AgentPtr a, const std::string& cid) const {
        return (a->classinfo().id != cid);
    }
};
}

BaseServer* CogServer::createInstance(AtomSpace* as)
{
    return new CogServer(as);
}

CogServer::~CogServer()
{
    logger().debug("[CogServer] enter destructor");
    disableNetworkServer();

    std::vector<std::string> moduleKeys;

    for (ModuleMap::iterator it = modules.begin(); it != modules.end(); ++it)
        moduleKeys.push_back(it->first);

    // unload all modules
    for (std::vector<std::string>::iterator k = moduleKeys.begin(); k != moduleKeys.end(); ++k) {
        // retest the key because it might have been removed already
        ModuleMap::iterator it = modules.find(*k);
        if (it != modules.end()) {
            logger().debug("[CogServer] removing module \"%s\"", it->first.c_str());
            ModuleData mdata = it->second;

            // cache filename and id to erase the entries from the modules map
            std::string filename = mdata.filename;
            std::string id = mdata.id;

            // invoke the module's unload function
            (*mdata.unloadFunction)(mdata.module);

            // erase the map entries (one with the filename as key, and
            // one with the module id as key)
            modules.erase(filename);
            modules.erase(id);
        }
    }

    // Shut down the system activity table.
    _systemActivityTable.halt();
    if (_private_as) delete _private_as;

    logger().debug("[CogServer] exit destructor");
}

CogServer::CogServer(AtomSpace* as) :
    BaseServer(as),
    cycleCount(1), running(false), _networkServer(nullptr)
{
    if (nullptr == as) {
        _atomSpace = new AtomSpace();
        _attentionBank = &attentionbank(_atomSpace);
        _private_as = _atomSpace;
    }
    else {
        _atomSpace = as;
        _private_as = nullptr;
    }

#ifdef HAVE_GUILE
    // Tell scheme which atomspace to use.
    SchemeEval::init_scheme();
    SchemeEval::set_scheme_as(_atomSpace);
#endif // HAVE_GUILE
#ifdef HAVE_CYTHON
    // Initialize Python.
    global_python_initialize();

    // Tell the python evaluator to create its singleton instance
    // with our atomspace.
    PythonEval::create_singleton_instance(_atomSpace);
#endif // HAVE_CYTHON

    _systemActivityTable.init(this);

    agentsRunning = true;
}

void CogServer::enableNetworkServer(int port)
{
    if (_networkServer) return;
    _networkServer = new NetworkServer(config().get_int("SERVER_PORT", port));
}

void CogServer::disableNetworkServer()
{
    if (_networkServer) {
        delete _networkServer;
        _networkServer = nullptr;
    }
}

SystemActivityTable& CogServer::systemActivityTable()
{
    return _systemActivityTable;
}

void CogServer::serverLoop()
{
    struct timeval timer_start, timer_end, elapsed_time;
    time_t cycle_duration = config().get_int("SERVER_CYCLE_DURATION", 100) * 1000;
//    bool externalTickMode = config().get_bool("EXTERNAL_TICK_MODE");

    logger().info("Starting CogServer loop.");

    gettimeofday(&timer_start, NULL);
    for (running = true; running;)
    {
        runLoopStep();

        gettimeofday(&timer_end, NULL);
        timersub(&timer_end, &timer_start, &elapsed_time);

        // sleep long enough so that the next cycle will only start
        // after config["SERVER_CYCLE_DURATION"] milliseconds
        long delta = cycle_duration;
        delta -= 1000.0*elapsed_time.tv_sec + elapsed_time.tv_usec/1000.0;
        if (delta > 0)
            usleep((unsigned int) delta);
        timer_start = timer_end;
    }

    // Perform a clean shutdown. Drain the request queue.
    while (0 < getRequestQueueSize())
    {
        processRequests();
    }

    // No way to process requests. Stop accepting network connections.
    disableNetworkServer();
    _systemActivityTable.halt();
}

void CogServer::runLoopStep(void)
{
    struct timeval timer_start, timer_end, elapsed_time, requests_time;
    // this refers to the current cycle, so that logging reports correctly
    // regardless of whether cycle is incremented.
    long currentCycle = this->cycleCount;

    // Process requests
    if (0 < getRequestQueueSize())
    {
        gettimeofday(&timer_start, NULL);
        processRequests();
        gettimeofday(&timer_end, NULL);
        timersub(&timer_end, &timer_start, &requests_time);

        logger().fine("[CogServer::runLoopStep cycle = %d] Time to process requests: %f",
                   currentCycle,
                   requests_time.tv_usec/1000000.0
                  );
    }

    // Process mind agents
    if (customLoopRun() and agentsRunning and 0 < agentScheduler.get_agents().size())
    {
        agentScheduler.process_agents();

        gettimeofday(&timer_end, NULL);
        timersub(&timer_end, &timer_start, &elapsed_time);
        logger().fine("[CogServer::runLoopStep cycle = %d] Time to process MindAgents: %f",
               currentCycle,
               elapsed_time.tv_usec/1000000.0, currentCycle
              );
    }

    cycleCount++;
    if (cycleCount < 0) cycleCount = 0;
}

bool CogServer::customLoopRun(void)
{
    return true;
}

void CogServer::processRequests(void)
{
    std::lock_guard<std::mutex> lock(processRequestsMutex);
    while (0 < getRequestQueueSize()) {
        Request* request = popRequest();
        request->execute();
        delete request;
    }
}

bool CogServer::registerAgent(const std::string& id, AbstractFactory<Agent> const* factory)
{
    return Registry<Agent>::register_(id, factory);
}

bool CogServer::unregisterAgent(const std::string& id)
{
    logger().debug("[CogServer] unregister agent \"%s\"", id.c_str());
    stopAllAgents(id);
    return Registry<Agent>::unregister(id);
}

std::list<const char*> CogServer::agentIds() const
{
    return Registry<Agent>::all();
}

AgentSeq CogServer::runningAgents(void)
{
    AgentSeq agents = agentScheduler.get_agents();
    for (auto &runner: agentThreads) {
        auto t = runner->get_agents();
        agents.insert(agents.end(), t.begin(), t.end());
    }
    return agents;
}

AgentPtr CogServer::createAgent(const std::string& id, const bool start)
{
    AgentPtr a(Registry<Agent>::create(*this, id));
    if (a && start) startAgent(a);
    return a;
}

void CogServer::startAgent(AgentPtr agent, bool dedicated_thread,
    const std::string &thread_name)
{
    if (dedicated_thread) {
        AgentRunnerThread *runner;
        auto runner_ptr = threadNameMap.find(thread_name);
        if (runner_ptr != threadNameMap.end())
            runner = runner_ptr->second;
        else {
            agentThreads.emplace_back(new AgentRunnerThread);
            runner = agentThreads.back().get();
            if (!thread_name.empty())
            {
                runner->set_name(thread_name);
                threadNameMap[thread_name] = runner;
            }
        }
        runner->add_agent(agent);
        if (agentsRunning)
            runner->start();
    }
    else
        agentScheduler.add_agent(agent);
}

void CogServer::stopAgent(AgentPtr agent)
{
    agentScheduler.remove_agent(agent);
    for (auto &runner: agentThreads)
        runner->remove_agent(agent);
    logger().debug("[CogServer] stopped agent \"%s\"", agent->to_string().c_str());
}

void CogServer::stopAllAgents(const std::string& id)
{
    agentScheduler.remove_all_agents(id);
    for (auto &runner: agentThreads)
        runner->remove_all_agents(id);
//    // remove statistical record of their activities
//    for (size_t n = 0; n < to_delete.size(); n++)
//        _systemActivityTable.clearActivity(to_delete[n]);
}

void CogServer::startAgentLoop(void)
{
    agentsRunning = true;
    for (auto &runner: agentThreads)
        runner->start();
}

void CogServer::stopAgentLoop(void)
{
    agentsRunning = false;
    for (auto &runner: agentThreads)
        runner->stop();
}

bool CogServer::registerRequest(const std::string& name, AbstractFactory<Request> const* factory)
{
    return Registry<Request>::register_(name, factory);
}

bool CogServer::unregisterRequest(const std::string& name)
{
    return Registry<Request>::unregister(name);
}

Request* CogServer::createRequest(const std::string& name)
{
    return Registry<Request>::create(*this, name);
}

const RequestClassInfo& CogServer::requestInfo(const std::string& name) const
{
    return static_cast<const RequestClassInfo&>(Registry<Request>::classinfo(name));
}

std::list<const char*> CogServer::requestIds() const
{
    return Registry<Request>::all();
}

long CogServer::getCycleCount()
{
    return cycleCount;
}

void CogServer::stop()
{
    running = false;
}

bool CogServer::loadModule(const std::string& filename)
{
// TODO FIXME I guess this needs to be different for windows.
#define PATH_SEP '/'
    // The module file identifier does NOT include the file path!
    std::string fi = filename;
    size_t path_sep = fi.rfind(PATH_SEP);
    if (path_sep != std::string::npos)
        fi.erase(0, path_sep+1);
    if (modules.find(fi) !=  modules.end()) {
        logger().info("Module \"%s\" is already loaded.", fi.c_str());
        return true;
    }

    // reset error
    dlerror();

    logger().info("Loading module \"%s\"", filename.c_str());
#ifdef __APPLE__
    // Tell dyld to search runpath
    std::string withRPath("@rpath/");
    withRPath += filename;
    // Check to see if so extension is specified, replace with .dylib if it is.
    if (withRPath.substr(withRPath.size()-3,3) == ".so") {
        withRPath.replace(withRPath.size()-3,3,".dylib");
    }
    void *dynLibrary = dlopen(withRPath.c_str(), RTLD_LAZY | RTLD_GLOBAL);
#else
    void *dynLibrary = dlopen(filename.c_str(), RTLD_LAZY | RTLD_GLOBAL);
#endif
    const char* dlsymError = dlerror();
    if ((dynLibrary == NULL) || (dlsymError)) {
        // This is almost surely due to a user configuration error.
        // User errors are always logged as warnings.
        logger().warn("Unable to load module \"%s\": %s", filename.c_str(), dlsymError);
        return false;
    }

    // reset error
    dlerror();

    // search for id function
    Module::IdFunction* id_func =
	    (Module::IdFunction*) dlsym(dynLibrary, Module::id_function_name());
    dlsymError = dlerror();
    if (dlsymError) {
        logger().error("Unable to find symbol \"opencog_module_id\": %s (module %s)", dlsymError, filename.c_str());
        return false;
    }

    // get and check module id
    const char *module_id = (*id_func)();
    if (module_id == NULL) {
        logger().warn("Invalid module id (module \"%s\")", filename.c_str());
        return false;
    }

    // search for 'load' & 'unload' symbols
    Module::LoadFunction* load_func =
	    (Module::LoadFunction*) dlsym(dynLibrary, Module::load_function_name());
    dlsymError = dlerror();
    if (dlsymError) {
        logger().error("Unable to find symbol \"opencog_module_load\": %s", dlsymError);
        return false;
    }

    Module::UnloadFunction* unload_func =
	    (Module::UnloadFunction*) dlsym(dynLibrary, Module::unload_function_name());
    dlsymError = dlerror();
    if (dlsymError) {
        logger().error("Unable to find symbol \"opencog_module_unload\": %s", dlsymError);
        return false;
    }

    // load and init module
    Module* module = (Module*) (*load_func)(*this);

    // store two entries in the module map:
    //    1: filename => <struct module data>
    //    2: moduleid => <struct module data>
    // we rely on the assumption that no module id will match the filename of
    // another module (and vice-versa). This is probably reasonable since most
    // module filenames should have a .dll or .so suffix, and module ids should
    // (by convention) be prefixed with its class namespace (i.e., "opencog::")
    std::string i = module_id;
    std::string f = filename;
    // The filename does NOT include the file path!
    path_sep = f.rfind(PATH_SEP);
    if (path_sep != std::string::npos)
        f.erase(0, path_sep+1);
    ModuleData mdata = {module, i, f, load_func, unload_func, dynLibrary};
    modules[i] = mdata;
    modules[f] = mdata;

    // after registration, call the module's init() method
    module->init();

    return true;
}

std::string CogServer::listModules()
{
    // Prepare a stream to collect the module information
    std::ostringstream oss;

    // Prepare iterators to process the ModuleMap
    ModuleMap::iterator startIterator = modules.begin();
    ModuleMap::iterator endIterator = modules.end();

    // Loop through the ModuleMap
    for(; startIterator != endIterator; ++startIterator)
    {
        // Get the module_id from the item
        std::string module_id = startIterator->first;
        ModuleData moduleData = startIterator->second;

        // Only list the names, not the filenames.
        if (module_id.find(".so", 0) != std::string::npos)
        {
            // Add the module_id to our stream
            oss
            // << "ModuleID: " << module_id
            << "Filename: " << moduleData.filename
            << ", ID: " << moduleData.id
            // << ", Load function: " << moduleData.loadFunction
            // << ", Module: " << moduleData.module
            // << ", Unload function: " << moduleData.unloadFunction
            << std::endl;
        }

    }

    // Return the contents of the stream
    return oss.str();
}

bool CogServer::unloadModule(const std::string& moduleId)
{
    // The module file identifier does NOT include the file path!
    std::string f = moduleId;
    size_t path_sep = f.rfind(PATH_SEP);
    if (path_sep != std::string::npos)
        f.erase(0, path_sep+1);
    logger().info("[CogServer] unloadModule(%s)", f.c_str());
    ModuleMap::const_iterator it = modules.find(f);
    if (it == modules.end()) {
        logger().info("[CogServer::unloadModule] module \"%s\" is not loaded.", f.c_str());
        return false;
    }
    ModuleData mdata = it->second;

    // cache filename, id and handle
    std::string filename = mdata.filename;
    std::string id       = mdata.id;
    void*       handle   = mdata.handle;

    // invoke the module's unload function
    (*mdata.unloadFunction)(mdata.module);

    // erase the map entries (one with the filename as key, and one with the module
    // id as key
    modules.erase(filename);
    modules.erase(id);

    // unload dynamically loadable library
    logger().info("Unloading module \"%s\"", filename.c_str());

    dlerror(); // reset error
    if (dlclose(handle) != 0) {
        const char* dlsymError = dlerror();
        if (dlsymError) {
            logger().warn("Unable to unload module \"%s\": %s", filename.c_str(), dlsymError);
            return false;
        }
    }

    return true;
}

CogServer::ModuleData CogServer::getModuleData(const std::string& moduleId)
{
    // The module file identifier does NOT include the file path!
    std::string f = moduleId;
    size_t path_sep = f.rfind(PATH_SEP);
    if (path_sep != std::string::npos)
        f.erase(0, path_sep+1);

    ModuleMap::const_iterator it = modules.find(f);
    if (it == modules.end()) {
        logger().info("[CogServer::getModuleData] module \"%s\" was not found.", f.c_str());
        ModuleData nulldata = {NULL, "", "", NULL, NULL, NULL};
        return nulldata;
    }
    return it->second;
}

Module* CogServer::getModule(const std::string& moduleId)
{
    return getModuleData(moduleId).module;
}

void CogServer::loadModules(std::vector<std::string> module_paths)
{
    if (module_paths.empty())
    {
        // Give priority search oorder to the build directories
        module_paths.push_back("./opencog/cogserver/server/");
        module_paths.push_back("./opencog/cogserver/shell/");

        // If not found at above locations, search the install paths
        for (auto p : get_module_paths())
        {
            module_paths.push_back(p);
            module_paths.push_back(p + "/opencog");
            module_paths.push_back(p + "/opencog/modules");
        }
    }

    // Load modules specified in the config file
    std::string modlist;
    if (config().has("MODULES"))
        modlist = config().get("MODULES");
    else
        // Defaults: search the build dirs first, then the install dirs.
        modlist =
            "libbuiltinreqs.so, "
            "libscheme-shell.so, "
            "libpy-shell.so";
    std::vector<std::string> modules;
    tokenize(modlist, std::back_inserter(modules), ", ");
    bool load_failure = false;
    for (const std::string& module : modules) {
        bool rc = false;
        if (not module_paths.empty()) {
            for (const std::string& module_path : module_paths) {
                boost::filesystem::path modulePath(module_path);
                modulePath /= module;
                if (boost::filesystem::exists(modulePath)) {
                    rc = loadModule(modulePath.string());
                    if (rc) break;
                }
            }
        } else {
            rc = loadModule(module);
        }
        if (!rc)
        {
            logger().warn("Failed to load cogserver module %s", module.c_str());
				load_failure = true;
        }
    }
    if (load_failure) {
        for (auto p : module_paths)
            logger().warn("Searched for module at %s", p.c_str());
    }
}

Logger &CogServer::logger()
{
    return ::logger();
}
