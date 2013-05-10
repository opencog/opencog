/*
 * @file opencog/cython/PythonEval.cc
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date   2011-09-20
 *
 * Reference: 
 *   http://www.linuxjournal.com/article/3641?page=0,2
 *   http://www.codeproject.com/KB/cpp/embedpython_1.aspx
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

#include <boost/filesystem/operations.hpp>

#include <opencog/util/Config.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/foreach.h>
#include <opencog/util/Logger.h>
#include <opencog/util/misc.h>

#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/Request.h>

#include "PythonEval.h"
#include "agent_finder_types.h"
#include "agent_finder_api.h"


using std::string;
using std::vector;

using namespace opencog;

//#define DPRINTF printf
#define DPRINTF(...)

PythonEval * PythonEval::singletonInstance = 0;

static const char* DEFAULT_PYTHON_MODULE_PATHS[] = 
{
    PROJECT_BINARY_DIR"/opencog/cython", // bindings
    PROJECT_SOURCE_DIR"/opencog/python", // opencog modules written in python
    PROJECT_SOURCE_DIR"/tests/cython",   // for testing
    DATADIR"/python",                    // install directory
#ifndef WIN32
    "/usr/local/share/opencog/python",
    "/usr/share/opencog/python",
#endif // !WIN32
    NULL
};

void PythonEval::init(void)
{
    logger().info("PythonEval::%s Initialising python evaluator.", __FUNCTION__);
    Py_SetProgramName((char*)"OpenCog");

    //Start up Python (this init method skips registering signal handlers)
    if(!Py_IsInitialized())
        Py_InitializeEx(0);
    if(!PyEval_ThreadsInitialized())
        PyEval_InitThreads();

    // Save a pointer to the main PyThreadState object
    this->mainThreadState = PyThreadState_Get();

    // Get a reference to the PyInterpreterState
    this->mainInterpreterState = this->mainThreadState->interp;

    pyModule = PyModule_New("openCogModule");
    PyModule_AddStringConstant(pyModule, "__file__", "");

    pyGlobal = PyDict_New();

    //Add our module directories to the Python interprator's path
    const char** config_paths = DEFAULT_PYTHON_MODULE_PATHS;

    PyObject* sysPath = PySys_GetObject((char*)"path");

    // Default paths for python modules
    for (int i = 0; config_paths[i] != NULL; ++i) {
        boost::filesystem::path modulePath(config_paths[i]);
        if (boost::filesystem::exists(modulePath))
            PyList_Append(sysPath, PyString_FromString(modulePath.string().c_str()));
    }

    // Add custom paths for python modules from the config file if available
    if (config().has("PYTHON_EXTENSION_DIRS")) {
        std::vector<std::string> pythonpaths;
        // For debugging current path
        tokenize(config()["PYTHON_EXTENSION_DIRS"], std::back_inserter(pythonpaths), ", ");
        for (std::vector<std::string>::const_iterator it = pythonpaths.begin();
             it != pythonpaths.end(); ++it) {
            boost::filesystem::path modulePath(*it);
            if (boost::filesystem::exists(modulePath)) {
                PyList_Append(sysPath, PyString_FromString(modulePath.string().c_str()));
            } else {
                logger().error("PythonEval::%s Could not find custom python extension directory: %s ",
                               __FUNCTION__,
                               (*it).c_str()
                              );
            }
        }
    }

    // Initialise the agent_finder module which helps with the Python side of
    // things
    if (import_agent_finder() == -1) {
        PyErr_Print();
        throw RuntimeException(TRACE_INFO,"PythonEval::init Failed to load helper python module");
    }

    // Import pattern_match_functions which contains user defined functions
    PyObject* pList = PyList_New(0);
    PyObject* pyLocal = PyModule_GetDict(pyModule);
    OC_ASSERT(pyLocal != NULL);
    pmfModule = PyImport_ImportModuleLevel((char*)"pattern_match_functions", pyGlobal, pyLocal, pList, 0);
    PyModule_AddObject(pyModule, "pattern_match_functions", pmfModule);
    Py_DECREF(pList);

    // For debugging the python path:
    logger().debug("Python sys.path is: " + get_path_as_string());
}

PyObject * PythonEval::getPyAtomspace(AtomSpace * atomspace) {
    PyObject * pAtomSpace; 

    if (atomspace)
        pAtomSpace = py_atomspace(atomspace);
    else
        pAtomSpace = py_atomspace(this->atomspace); 

    if (pAtomSpace != NULL)
        logger().debug("PythonEval::%s Get atomspace wrapped with python object",
                       __FUNCTION__
                      ); 
    else {
        if (PyErr_Occurred())
            PyErr_Print();

        logger().error("PythonEval::%s Failed to get atomspace wrapped with python object", 
                       __FUNCTION__
                      ); 
    }

    return pAtomSpace; 
}

void PythonEval::printDict(PyObject* obj) {  
    if (!PyDict_Check(obj))  
        return;  

    PyObject *k, *keys;  
    keys = PyDict_Keys(obj);  
    for (int i = 0; i < PyList_GET_SIZE(keys); i++) {  
        k = PyList_GET_ITEM(keys, i);  
        char* c_name = PyString_AsString(k);  
        printf("%s/n", c_name);  
    }  
}

PythonEval::~PythonEval()
{
    logger().info("PythonEval::%s destructor", __FUNCTION__);
    Py_Finalize();

    delete pyModule;
    delete pyGlobal;
}


// Use a singleton instance to avoid initializing python interpreter
// twice.
PythonEval& PythonEval::instance(AtomSpace * atomspace)
{
    if (!singletonInstance)
    {
        if (!atomspace) {
            // Create our own local AtomSpace to send calls to
            // the
            // event loop (otherwise the getType cache breaks)
            atomspace = new AtomSpace(cogserver().getAtomSpace());
        }
        singletonInstance = new PythonEval(atomspace);
    }
    else if (atomspace and singletonInstance->atomspace->atomSpaceAsync !=
               atomspace->atomSpaceAsync)
    {
        // Someone is trying to initialize the Python
        // interpreter
        // on a different AtomSpace. because of the singleton
        // design
        // there is no easy way to support this...
        throw RuntimeException(TRACE_INFO, "Trying to re-initialize"
              " python interpreter with different AtomSpaceAsync ptr!");
   }
   return *singletonInstance;
}

PyObject* PythonEval::call_func(const std::string name, const int arg)
{
    PyObject *pFunc, *pArgs, *pInt, *pValue = NULL;

    pFunc = PyObject_GetAttrString(pyModule, name.c_str());

    OC_ASSERT(pFunc != NULL);
    OC_ASSERT(PyCallable_Check(pFunc));

    pArgs = PyTuple_New(1);
    pInt = PyInt_FromLong(arg);
    OC_ASSERT(pInt != NULL);
    PyTuple_SetItem(pArgs, 0, pInt);

    pValue = PyObject_CallObject(pFunc, pArgs);

    Py_DECREF(pArgs);
    Py_DECREF(pInt);
    Py_DECREF(pFunc);

    return pValue;
}

void PythonEval::apply(std::string script)
{
    PyObject *pyLocal = PyModule_GetDict(pyModule);
    OC_ASSERT(pyLocal != NULL);

    PyRun_String(script.c_str(), Py_file_input, pyGlobal, pyLocal);
}

