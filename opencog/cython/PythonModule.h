// Enables support for loading MindAgents and CogServer requests written in
// Python
//
#ifndef _OPENCOG_PYTHON_MODULE_H
#define _OPENCOG_PYTHON_MODULE_H

#include <Python.h>

#include <string>

#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/server/Request.h>
#include <opencog/server/CogServer.h>

#include "PyMindAgent.h"

using std::string;

namespace opencog
{

class CogServer;

class PythonAgentFactory : public AbstractFactory<Agent>
{
    // Store the name of the python module and class so that we can instantiate
    // them
    string pySrcModuleName;
    string pyClassName;
public:
    explicit PythonAgentFactory(string& module, string& clazz) : AbstractFactory<Agent>() {
        pySrcModuleName = module;
        pyClassName = clazz;
    }
    virtual ~PythonAgentFactory() {}
    virtual Agent* create() const { return new PyMindAgent(pySrcModuleName,pyClassName); }
    virtual const ClassInfo& info() const { return PyMindAgent::info(); }
}; 

class PythonModule : public Module
{
    DECLARE_CMD_REQUEST(PythonModule, "loadpy", do_load_py, 
       "Load requests and MindAgents from a python module", 
       "Usage: load_py module_name\n\n"
       "Loads and registers all request classes and all MindAgents with the CogServer", 
       false);

private:
    static inline AbstractFactory<Agent>& factory() {
        static Factory<PyMindAgent, Agent> _factory;
        return _factory;
    }

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::PythonModule");
        return _ci;
    }
    static inline const char* id();

    PythonModule();
    ~PythonModule();
    void init();

    std::string name;

    PyObject* load_module(string& filename);
}; // class

} // namespace opencog

#endif // _OPENCOG_SINGLE_AGENT_MODULE_H

