/*
 * opencog/server/CogServer.cc
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/cython/PythonEval.h>
#include <opencog/guile/load-file.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/server/Agent.h>
#include <opencog/server/ConsoleSocket.h>
#include <opencog/server/NetworkServer.h>
#include <opencog/server/SystemActivityTable.h>
#include <opencog/server/Request.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/files.h>
#include <opencog/util/misc.h>
#include <opencog/util/platform.h>

#ifdef HAVE_SQL_STORAGE
#include <opencog/persist/sql/PersistModule.h>
#endif /* HAVE_SQL_STORAGE */

#include "CogServer.h"
#include "BaseServer.h"

using namespace opencog;

namespace opencog {
struct equal_to_id : public std::binary_function<AgentPtr, const std::string&, bool>
{
    bool operator()(AgentPtr a, const std::string& cid) const {
        return (a->classinfo().id != cid);
    }
};
}

BaseServer* CogServer::createInstance()
{
    return new CogServer();
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

            // erase the map entries (one with the filename as key, and one with the module)
            // id as key
            modules.erase(filename);
            modules.erase(id);
        }
    }

    logger().debug("[CogServer] exit destructor");
}

CogServer::CogServer() : cycleCount(1)
{
    if (atomSpace) delete atomSpace;  // global static, declared in BaseServer.
    atomSpace = new AtomSpace();
#ifdef HAVE_GUILE
    // Tell scheme which atomspace to use.
    SchemeEval* se = new SchemeEval(atomSpace);
    delete se;
#endif // HAVE_GUILE
#ifdef HAVE_CYTHON
    // Tell python which atomspace to use.
    PythonEval::instance(atomSpace);
#endif // HAVE_CYTHON

    _systemActivityTable.init(this);

    agentsRunning = true;
}

NetworkServer& CogServer::networkServer()
{
    return _networkServer;
}

void CogServer::enableNetworkServer()
{
    // WARN: By using boost::asio, at least one listener must be added to
    // the NetworkServer before starting its thread. Other Listeners may
    // be added later, though.
    _networkServer.addListener<ConsoleSocket>(config().get_int("SERVER_PORT"));
    _networkServer.start();

}

void CogServer::disableNetworkServer()
{
    _networkServer.stop();
}

SystemActivityTable& CogServer::systemActivityTable()
{
    return _systemActivityTable;
}

void CogServer::serverLoop()
{
    struct timeval timer_start, timer_end;
    time_t elapsed_time;
    time_t cycle_duration = config().get_int("SERVER_CYCLE_DURATION") * 1000;
//    bool externalTickMode = config().get_bool("EXTERNAL_TICK_MODE");

    logger().info("Starting CogServer loop.");

    gettimeofday(&timer_start, NULL);
    for (running = true; running;) {
        // Because cycleCount may or may not get incremented
        long currentCycle = this->cycleCount;
        runLoopStep();

        gettimeofday(&timer_end, NULL);
        elapsed_time = ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
                       (timer_end.tv_usec - timer_start.tv_usec);

//        if (!externalTickMode) {
            // sleep long enough so that the next cycle will only start
            // after config["SERVER_CYCLE_DURATION"] milliseconds
            if ((cycle_duration - elapsed_time) > 0)
                usleep((unsigned int) (cycle_duration - elapsed_time));
//        }

        if (currentCycle != this->cycleCount) {
            logger().fine("[CogServer::serverLoop] Cycle %d completed in %f seconds.",
                            currentCycle,
                           elapsed_time/1000000.0
                          );
        }

        timer_start = timer_end;

    }
}

void CogServer::runLoopStep(void)
{
    struct timeval timer_start, timer_end;
    time_t elapsed_time;
    time_t requests_time;
    // this refers to the current cycle, so that logging reports correctly
    // regardless of whether cycle is incremented.
    long currentCycle = this->cycleCount;

    // Process requests
    gettimeofday(&timer_start, NULL);

    if (getRequestQueueSize() != 0) {
        processRequests();
        gettimeofday(&timer_end, NULL);
        requests_time = ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
                   (timer_end.tv_usec - timer_start.tv_usec);

        logger().fine("[CogServer::runLoopStep cycle = %d] Time to process requests: %f",
                   currentCycle,
                   requests_time/1000000.0
                  );
    } else {
        gettimeofday(&timer_end, NULL);
    }

    // Run custom loop
    timer_start = timer_end;
    bool runCycle = customLoopRun();

    gettimeofday(&timer_end, NULL);
    elapsed_time = ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
                   (timer_end.tv_usec - timer_start.tv_usec);

    logger().fine("[CogServer::runLoopStep cycle = %d] Time to run customRunLoop: %f",
                   currentCycle,
                   elapsed_time/1000000.0
                  );

    // Process mind agents
    timer_start = timer_end;
    if (runCycle) {
        if (agentsRunning) {
            processAgents();
        }

        cycleCount++;
        if (cycleCount < 0) cycleCount = 0;

        gettimeofday(&timer_end, NULL);
        elapsed_time = ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
                       (timer_end.tv_usec - timer_start.tv_usec);
        logger().fine("[CogServer::runLoopStep cycle = %d] Time to process MindAgents: %f",
                   currentCycle,
                   elapsed_time/1000000.0, currentCycle
                  );
    } else {
        // Skipping MindAgents, and not incremented cycle counter.
        logger().fine("[CogServer::runLoopStep cycle = %d] customRunLoop returned false.", currentCycle);
    }

}

bool CogServer::customLoopRun(void)
{
    return true;
}

void CogServer::processRequests(void)
{
    std::unique_lock<std::mutex> lock(processRequestsMutex);
    while (0 < getRequestQueueSize()) {
        Request* request = popRequest();
        request->execute();
        delete request;
    }
}

void CogServer::runAgent(AgentPtr agent)
{
    struct timeval timer_start, timer_end;
    struct timeval elapsed_time;
    size_t mem_start, mem_end;
    size_t atoms_start, atoms_end;
    size_t mem_used, atoms_used;
    time_t time_used;

    gettimeofday(&timer_start, NULL);
    mem_start = getMemUsage();
    atoms_start = atomSpace->getSize();

    logger().debug("[CogServer] begin to run mind agent: %s, [cycle = %d]",
                   agent->classinfo().id.c_str(),  this->cycleCount);

    agent->resetUtilizedHandleSets();
    agent->run();

    gettimeofday(&timer_end, NULL);
    mem_end = getMemUsage();
    atoms_end = atomSpace->getSize();

    time_used =  timer_end.tv_sec*1000000 + timer_end.tv_usec -
                (timer_start.tv_sec*1000000  + timer_start.tv_usec);

    elapsed_time.tv_sec = time_used/1000000;
    elapsed_time.tv_usec = time_used - elapsed_time.tv_sec*1000000;

    if (mem_start > mem_end)
        mem_used = 0;
    else
        mem_used = mem_end - mem_start;
    if (atoms_start > atoms_end)
        atoms_used = 0;
    else
        atoms_used = atoms_end - atoms_start;

    logger().debug("[CogServer] running mind agent: %s, elapsed time (sec): %f, memory used: %d, atom used: %d [cycle = %d]",
                   agent->classinfo().id.c_str(), 1.0*time_used/1000000, mem_used, atoms_used, this->cycleCount
                  );

    _systemActivityTable.logActivity(agent, elapsed_time, mem_used,
                                            atoms_used);
}

void CogServer::processAgents(void)
{
    std::unique_lock<std::mutex> lock(agentsMutex);
    AgentSeq::const_iterator it;
    for (it = agents.begin(); it != agents.end(); ++it) {
        AgentPtr agent = *it;
        if ((cycleCount % agent->frequency()) == 0)
            runAgent(agent);
    }
}

bool CogServer::registerAgent(const std::string& id, AbstractFactory<Agent> const* factory)
{
    return Registry<Agent>::register_(id, factory);
}

bool CogServer::unregisterAgent(const std::string& id)
{
    logger().debug("[CogServer] unregister agent \"%s\"", id.c_str());
    destroyAllAgents(id);
    return Registry<Agent>::unregister(id);
}

std::list<const char*> CogServer::agentIds() const
{
    return Registry<Agent>::all();
}

AgentPtr CogServer::createAgent(const std::string& id, const bool start)
{
    AgentPtr a(Registry<Agent>::create(*this, id));
    if (a && start) startAgent(a);
    return a;
}

void CogServer::startAgent(AgentPtr agent)
{
    std::unique_lock<std::mutex> lock(agentsMutex);
    agents.push_back(agent);
}

void CogServer::stopAgent(AgentPtr agent)
{
    std::unique_lock<std::mutex> lock(agentsMutex);
    AgentSeq::iterator ai = std::find(agents.begin(), agents.end(), agent);
    if (ai != agents.end())
        agents.erase(ai);
    lock.unlock();
    logger().debug("[CogServer] stopped agent \"%s\"", agent->to_string().c_str());
}

void CogServer::destroyAgent(AgentPtr agent)
{
    stopAgent(agent);
    logger().debug("[CogServer] deleting agent \"%s\"", agent->to_string().c_str());
}

void CogServer::destroyAllAgents(const std::string& id)
{
    // TODO: This will need to be changed for MindAgents that are not
    // constrained to only running every N cognitive cycles. I.e. when
    // they have their own thread they'll have to be stopped and their threads
    // "joined"
    std::unique_lock<std::mutex> lock(agentsMutex);
    // place agents with classinfo().id == id at the end of the container
    AgentSeq::iterator last =
        std::partition(agents.begin(), agents.end(),
                       boost::bind(equal_to_id(), _1, id));

    // save the agents that should be deleted on a temporary container
    AgentSeq to_delete(last, agents.end());

    // remove those agents from the main container
    agents.erase(last, agents.end());

    // remove statistical record of their activities
    for (size_t n = 0; n < to_delete.size(); n++)
        _systemActivityTable.clearActivity(to_delete[n]);

    // delete the selected agents; NOTE: we must ensure that this is executed
    // after the 'agents.erase' call above, because the agent's destructor might
    // include a recursive call to destroyAllAgents
    // std::for_each(to_delete.begin(), to_delete.end(), safe_deleter<Agent>());
}

void CogServer::startAgentLoop(void)
{
    agentsRunning = true;
}

void CogServer::stopAgentLoop(void)
{
    agentsRunning = false;
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
    Module::IdFunction* id_func = (Module::IdFunction*) dlsym(dynLibrary, Module::id_function_name());
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
    Module::LoadFunction* load_func = (Module::LoadFunction*) dlsym(dynLibrary, Module::load_function_name());
    dlsymError = dlerror();
    if (dlsymError) {
        logger().error("Unable to find symbol \"opencog_module_load\": %s", dlsymError);
        return false;
    }

    Module::UnloadFunction* unload_func = (Module::UnloadFunction*) dlsym(dynLibrary, Module::unload_function_name());
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

void CogServer::loadModules(const char* module_paths[])
{
    if (NULL == module_paths) {
        module_paths = DEFAULT_MODULE_PATHS;
    }

    // Load modules specified in the config file
    std::vector<std::string> modules;
    tokenize(config()["MODULES"], std::back_inserter(modules), ", ");
    std::vector<std::string>::const_iterator it;
    for (it = modules.begin(); it != modules.end(); ++it) {
        bool rc = false;
        const char * mod = (*it).c_str();
        if ( module_paths != NULL ) {
            for (int i = 0; module_paths[i] != NULL; ++i) {
                boost::filesystem::path modulePath(module_paths[i]);
                modulePath /= *it;
                if (boost::filesystem::exists(modulePath)) {
                    mod = modulePath.string().c_str();
                    rc = loadModule(mod);
                    if (rc) break;
                }
            }
        } else {
            rc = loadModule(mod);
        }
        if (!rc)
        {
           logger().warn("Failed to load %s", mod);
        }
    }
}

void CogServer::loadSCMModules(const char* config_paths[])
{
#ifdef HAVE_GUILE
    if (NULL == config_paths) {
        config_paths = DEFAULT_MODULE_PATHS;
    }

    load_scm_files_from_config(*atomSpace, config_paths);
#else /* HAVE_GUILE */
    logger().warn(
        "Server compiled without SCM support");
#endif /* HAVE_GUILE */
}

void CogServer::openDatabase(void)
{
    // No-op if the user has not configured a storage backend
    if (!config().has("STORAGE")) {
        logger().warn("No database persistant storage configured! "
                      "Use the STORAGE config keyword to define.");
        return;
    }

#ifdef HAVE_SQL_STORAGE
    const std::string &dbname = config()["STORAGE"];
    const std::string &username = config()["STORAGE_USERNAME"];
    const std::string &passwd = config()["STORAGE_PASSWD"];

    std::list<std::string> args;
    args.push_back(dbname);
    args.push_back(username);
    args.push_back(passwd);

    // Do this all very politely, by loading the required module,
    // and then calling methods on it, as needed.
    loadModule("libpersist.so");

    Module *mod = getModule("opencog::PersistModule");
    if (NULL == mod)
    {
        logger().warn("Failed to pre-load database, because persist module not found!\n");
        return;
    }
    PersistModule *pm = dynamic_cast<PersistModule *>(mod);
    const std::string &resp = pm->do_open(NULL, args);

    logger().info("Preload %s as user %s msg: %s",
        dbname.c_str(), username.c_str(), resp.c_str());

#else /* HAVE_SQL_STORAGE */
    logger().warn(
        "Server compiled without database support");
#endif /* HAVE_SQL_STORAGE */
}

Logger &CogServer::logger()
{
    return ::logger();
}
