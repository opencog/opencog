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

#include "PythonEval.h"
#include "agent_finder_api.h"

// for backward compatibility as from boost 1.46 filesystem 3 is the default
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem/operations.hpp>

#include <boost/foreach.hpp>
#ifndef foreach
#define foreach  BOOST_FOREACH
#endif

#include <opencog/util/Config.h>
#include <opencog/util/misc.h>

using std::vector;
using std::string;

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

    // Start up Python (this init method skips registering signal handlers)
    Py_InitializeEx(0);
    PyEval_InitThreads();

    // Save a pointer to the main PyThreadState object
    this->mainThreadState = PyThreadState_Get();

    // Get a reference to the PyInterpreterState
    this->mainInterpreterState = this->mainThreadState->interp;

    // Add our module directories to the Python interprator's path
    const char** config_paths = DEFAULT_PYTHON_MODULE_PATHS;
    PyRun_SimpleString("import sys\n");
    PyRun_SimpleString("paths=[]\n");

    // Add custom paths for python modules from the config file if available
    if (config().has("PYTHON_EXTENSION_DIRS")) {
        std::vector<std::string> pythonpaths;
        // For debugging current path
        //boost::filesystem::path getcwd = boost::filesystem::current_path();
        //std::cout << getcwd << std::endl;
        tokenize(config()["PYTHON_EXTENSION_DIRS"], std::back_inserter(pythonpaths), ", ");
        for (std::vector<std::string>::const_iterator it = pythonpaths.begin();
             it != pythonpaths.end(); ++it) {
            boost::filesystem::path modulePath(*it);
            if (boost::filesystem::exists(modulePath)) {
                PyRun_SimpleString(("paths.append('" + modulePath.string() + "')\n").c_str());
            } else {
                logger().error("PythonEval::%s Could not find custom python extension directory: %s ", 
                               __FUNCTION__, 
                               (*it).c_str()
                              );
            }
        }
    }

    // Default paths for python modules
    for (int i = 0; config_paths[i] != NULL; ++i) {
        boost::filesystem::path modulePath(config_paths[i]);
        if (boost::filesystem::exists(modulePath)) {
            std::string x = modulePath.string();
            // remove the path if it already exists in sys.path
            PyRun_SimpleString(("if \"" + x + "\" in sys.path: sys.path.remove(\"" + x + "\")").c_str());
            // and then place it at the new position
            PyRun_SimpleString(("if \"" + x + "\" not in paths: paths.append('" + x + "')\n").c_str());
        }
    }
    PyRun_SimpleString("sys.path = paths + sys.path\n");
    //PyRun_SimpleString("import sys; print sys.path\n");

    // Initialise the agent_finder module which helps with the Python side of
    // things
    if (import_agent_finder() == -1) {
        PyErr_Print();
        throw RuntimeException(TRACE_INFO,"PythonEval::init Failed to load helper python module");
    }
    // For debugging the python path:
    logger().debug("Python sys.path is: " + get_path_as_string());
  
    PyEval_ReleaseThread(mainThreadState); 
//    PyEval_SaveThread();
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
    PyEval_RestoreThread(mainThreadState);
    logger().info("PythonEval::%s destructor", __FUNCTION__);
    Py_Finalize();
}

