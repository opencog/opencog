/*
 * @file opencog/cython/PythonEval.cc
 * @author Zhenhua Cai <czhedu@gmail.com>
 *         Ramin Barati <rekino@gmail.com>
 *         Keyvan Mir Mohammad Sadeghi <keyvan@opencog.org>
 *         Curtis Faith <curtis.m.faith@gmail.com>
 * @date 2011-09-20
 *
 * @todo When can we remove the singleton instance?
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
#include <opencog/util/Logger.h>
#include <opencog/util/misc.h>
#include <opencog/util/oc_assert.h>

#include <opencog/server/CogServer.h>

#include "PythonEval.h"

#include "opencog/atomspace_api.h"
#include "opencog/agent_finder_types.h"
#include "opencog/agent_finder_api.h"

using std::string;
using std::vector;

using namespace opencog;

//#define DPRINTF printf
#define DPRINTF(...)


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


PythonEval* PythonEval::singletonInstance = NULL;
AtomSpace* PythonEval::singletonAtomSpace = NULL;

const int NO_SIGNAL_HANDLERS = 0;

static bool already_initialized = false;

void opencog::global_python_initialize()
{
    logger().debug("[global_python_initialize] Start");

    // Throw an exception if this is called more than once.
    if (already_initialized) {
        throw opencog::RuntimeException(TRACE_INFO,
                "Python initializer global_python_init() called twice.");
    }

    // Remember this initialization.
    already_initialized = true;

    // Start up Python. (InitThreads grabs GIL implicitly)
    Py_InitializeEx(NO_SIGNAL_HANDLERS);
    PyEval_InitThreads();

    logger().debug("[global_python_initialize] Adding OpenCog sys.path "
            "directories");

    // Get starting "sys.path".
    PyRun_SimpleString(
                "import sys\n"
                "import StringIO\n"
                );
    PyObject* pySysPath = PySys_GetObject((char*)"path");

    // Add default OpenCog module directories to the Python interprator's path.
    const char** config_paths = DEFAULT_PYTHON_MODULE_PATHS;
    for (int i = 0; config_paths[i] != NULL; ++i) {
        boost::filesystem::path modulePath(config_paths[i]);
        if (boost::filesystem::exists(modulePath)) {
            const char* modulePathCString = modulePath.string().c_str();
            PyObject* pyModulePath = PyBytes_FromString(modulePathCString);
            PyList_Append(pySysPath, pyModulePath);
            Py_DECREF(pyModulePath);
        }
    }

    // Add custom paths for python modules from the config file.
    if (config().has("PYTHON_EXTENSION_DIRS")) {
        std::vector<string> pythonpaths;
        // For debugging current path
        tokenize(config()["PYTHON_EXTENSION_DIRS"], 
                std::back_inserter(pythonpaths), ", ");
        for (std::vector<string>::const_iterator it = pythonpaths.begin();
             it != pythonpaths.end(); ++it) {
            boost::filesystem::path modulePath(*it);
            if (boost::filesystem::exists(modulePath)) {
                const char* modulePathCString = modulePath.string().c_str();
                PyObject* pyModulePath = PyBytes_FromString(modulePathCString);
                PyList_Append(pySysPath, pyModulePath);
                Py_DECREF(pyModulePath);
            } else {
                logger().warn("PythonEval::%s Could not find custom python"
                        " extension directory: %s ",
                        __FUNCTION__, (*it).c_str() );
            }
        }
    }

    // NOTE: Can't use get_path_as_string() yet because it is defined in a
    // Cython api which we can't import unless the sys.path is correct. So
    // we'll write it out before the imports below to aid in debugging.
    if (logger().isDebugEnabled()) {
        logger().debug("Python 'sys.path' after OpenCog config adds is:");
        Py_ssize_t pathSize = PyList_Size(pySysPath);
        for (int pathIndex = 0; pathIndex < pathSize; pathIndex++) {
            PyObject* pySysPathLine = PyList_GetItem(pySysPath, pathIndex);
            const char* sysPathCString = PyString_AsString(pySysPathLine);
            logger().debug("    %2d > %s", pathIndex, sysPathCString);
            // NOTE: PyList_GetItem returns borrowed reference so don't do this:
            // Py_DECREF(pySysPathLine);
        } 
    }

    // Initialize the auto-generated Cython api. Do this AFTER the python
    // sys.path is updated so the imports can find the cython modules.
    import_opencog__atomspace();
    import_opencog__agent_finder();

    // Now we can use get_path_as_string() to get 'sys.path'
    logger().info("Python 'sys.path' after OpenCog config adds is: " +
            get_path_as_string());

    // NOTE: PySys_GetObject returns a borrowed reference so don't do this:
    // Py_DECREF(pySysPath);

    // Release the GIL. Otherwise the Python shell hangs on startup.
    PyEval_ReleaseLock();

    logger().debug("[global_python_initialize] Finish");
}

void opencog::global_python_finalize()
{
    logger().debug("[global_python_finalize] Start");

    // Cleanup Python. 
    Py_Finalize();

    // No longer initialized.
    already_initialized = false;

    logger().debug("[global_python_finalize] Finish");
}

PythonEval::PythonEval(AtomSpace* atomspace)
{
    // Check that this is the first and only PythonEval object.
    if (singletonInstance) {
        throw (RuntimeException(TRACE_INFO,
                "Can't create more than one PythonEval singleton instance!"));
    }

    // Remember our atomspace.
    _atomspace = atomspace;

    // Initialize Python objects and imports.
    this->initialize_python_objects_and_imports();

    // Add the preload functions
    if (config().has("PYTHON_PRELOAD_FUNCTIONS")) {
        string preloadDirectory = config()["PYTHON_PRELOAD_FUNCTIONS"];
        this->addModuleFromPath(preloadDirectory);
    }
}

PythonEval::~PythonEval()
{
    logger().info("PythonEval::%s destructor", __FUNCTION__);

    // Grab the GIL
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    // Decrement reference counts for instance Python object references.
    Py_DECREF(this->pyGlobal);
    Py_DECREF(this->pyLocal);

    // NOTE: The following come from Python C api calls that return borrowed
    // references. However, we have called Py_INCREF( x ) to promote them
    // to full references so we can and must decrement them here.
    Py_DECREF(this->pySysPath);
    Py_DECREF(this->pyRootModule);

    // Release the GIL. No Python API allowed beyond this point.
    PyGILState_Release(gstate);
}

/**
* Use a singleton instance to avoid initializing python interpreter twice.
*/
void PythonEval::create_singleton_instance(AtomSpace* atomspace)
{
    // Remember which atomspace is used for the singleton for
    // later sanity checks.
    singletonAtomSpace = atomspace;
    
    // Create the single instance of a PythonEval object.
    singletonInstance = new PythonEval(atomspace);
}

void PythonEval::delete_singleton_instance()
{
    // This should only be called once after having created one.
    if (!singletonInstance) {
        throw (RuntimeException(TRACE_INFO, 
                "Null singletonInstance in delete_singleton_instance()"));
    }

    // Delete the singleton PythonEval instance.
    delete singletonInstance;
    singletonInstance = NULL;

    // Clear the reference to the atomspace. We do not own it so we
    // won't delete it.
    singletonAtomSpace = NULL;
}

PythonEval& PythonEval::instance(AtomSpace* atomspace)
{
    // Make sure we have a singleton.
    if (!singletonInstance) {
        throw (RuntimeException(TRACE_INFO, 
                "Null singletonInstance! Did you create a CogServer?"));
    }

    // Make sure the atom space is the same as the one in the singleton.
    if (atomspace and singletonInstance->_atomspace != atomspace) {

        // Someone is trying to initialize the Python interpreter on a
        // different AtomSpace.  Because of the singleton design of the
        // the CosgServer+AtomSpace, there is no easy way to support this...
        throw RuntimeException(TRACE_INFO, "Trying to re-initialize"
                " python interpreter with different AtomSpace ptr!");
    }
    return *singletonInstance;
}

void PythonEval::initialize_python_objects_and_imports(void)
{
    // Grab the GIL
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
 
    // Get sys.path and keep the reference, used in this->addSysPath()
    // NOTE: We have to promote the reference here with Py_INCREF because
    // PySys_GetObject returns a borrowed reference and we don't want it to
    // go away behind the scenes.
    this->pySysPath = PySys_GetObject((char*)"path");
    Py_INCREF(this->pySysPath);

    // Get the __main__ module. NOTE: As above, PyImport_AddModule returns 
    // a borrowed reference so we must promote it with an increment.
    this->pyRootModule = PyImport_AddModule("__main__");
    Py_INCREF(this->pyRootModule);
    PyModule_AddStringConstant(this->pyRootModule, "__file__", "");

    // Define the user function executer
    // XXX FIXME ... this should probably be defined in some py file
    // that is loaded up by opencog.conf
    PyRun_SimpleString(
        "from opencog.atomspace import Handle, Atom\n"
        "import inspect\n"
        "def execute_user_defined_function(func, handle_uuid):\n"
        "    handle = Handle(handle_uuid)\n"
        "    args_list_link = ATOMSPACE[handle]\n"
        "    no_of_arguments_in_pattern = len(args_list_link.out)\n"
        "    no_of_arguments_in_user_fn = len(inspect.getargspec(func).args)\n"
        "    if no_of_arguments_in_pattern != no_of_arguments_in_user_fn:\n"
        "        raise Exception('Number of arguments in the function (' + "
        "str(no_of_arguments_in_user_fn) + ') does not match that of the "
        "corresponding pattern (' + str(no_of_arguments_in_pattern) + ').')\n"
        "    atom = func(*args_list_link.out)\n"
        "    if atom is None:\n"
        "        return\n"
        "    assert(type(atom) == Atom)\n"
        "    return atom.h.value()\n\n");

    // Add ATOMSPACE to __main__ module.
    PyObject* pyRootDictionary = PyModule_GetDict(this->pyRootModule);
    PyObject* pyAtomSpaceObject = this->getPyAtomspace();
    PyDict_SetItemString(pyRootDictionary, "ATOMSPACE", pyAtomSpaceObject);
    Py_DECREF(pyAtomSpaceObject);

    // PyModule_GetDict returns a borrowed reference, so don't do this:
    // Py_DECREF(pyRootDictionary);
    
    // These are needed for calling Python/C API functions, define 
    // them once here so we can reuse them.
    this->pyGlobal = PyDict_New();
    this->pyLocal = PyDict_New();

    // Release the GIL. No Python API allowed beyond this point.
    PyGILState_Release(gstate);

    logger().info("PythonEval::%s Finished initialising python evaluator.",
        __FUNCTION__);
}

PyObject* PythonEval::getPyAtomspace(AtomSpace* atomspace)
{
    PyObject * pyAtomSpace;

    if (atomspace)
        pyAtomSpace = py_atomspace(atomspace);
    else
        pyAtomSpace = py_atomspace(this->_atomspace);

    if (!pyAtomSpace) {
        if (PyErr_Occurred())
            PyErr_Print();

        logger().error("PythonEval::%s Failed to get atomspace "
                       "wrapped with python object", __FUNCTION__);
    }

    return pyAtomSpace;
}

void PythonEval::printDict(PyObject* obj)
{
    if (!PyDict_Check(obj))
        return;

    PyObject *pyKey, *pyKeys;

    // Get the keys from the dictionary and print them.
    pyKeys = PyDict_Keys(obj);
    for (int i = 0; i < PyList_GET_SIZE(pyKeys); i++) {

        // Get and print one key.
        pyKey = PyList_GET_ITEM(pyKeys, i);
        char* c_name = PyBytes_AsString(pyKey);
        printf("%s\n", c_name);
 
        // Cleanup the Python reference count for this key.
        Py_DECREF(pyKey);
    }

    // Cleanup the Python reference count for the keys list.
    Py_DECREF(pyKeys);
}

Handle PythonEval::apply(const std::string& func, Handle varargs)
{
    PyObject *pyError, *pyModule, *pyUserFunc, *pyExecuteUserFunc;
    PyObject *pyArgs, *pyUUID, *pyReturnValue = NULL;
    PyObject *pyDict;
    string moduleName;
    string funcName;
    bool errorCallingFunction;
    string errorString;
    UUID uuid = 0;

    // Get the correct module and extract the function name.
    int index = func.find_first_of('.');
    if (index < 0){
        pyModule = this->pyRootModule;
        funcName = func;
        moduleName = "__main__";
    } else {
        moduleName = func.substr(0,index);
        pyModule = this->modules[moduleName];
        funcName = func.substr(index+1);
    }

    // Grab the GIL.
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    // Get a reference to the user function.
    pyDict = PyModule_GetDict(pyModule);
    pyUserFunc = PyDict_GetItemString(pyDict, funcName.c_str());

    // PyModule_GetDict returns a borrowed reference, so don't do this:
    // Py_DECREF(pyDict);

    // If we can't find that function then throw an exception.
    if (!pyUserFunc) {
        PyGILState_Release(gstate);
        throw (RuntimeException(TRACE_INFO, "Python function '%s' not found!",
                funcName.c_str()));
    }
        
    // Promote the borrowed reference for pyUserFunc since it will
    // be passed to a Python C API function later that "steals" it.
    Py_INCREF(pyUserFunc);

    // Make sure the function is callable.
    if (!PyCallable_Check(pyUserFunc)) {
        Py_DECREF(pyUserFunc);
        PyGILState_Release(gstate);
        logger().error() << "Member " << func << " is not callable.";
        return Handle::UNDEFINED;
    }

    // Get a reference to our executer function. NOTE: Same as above,
    // we must get separate references for later decrement of the
    // Python reference counts and promotion of borrowed references.
    pyDict = PyModule_GetDict(this->pyRootModule);
    pyExecuteUserFunc = PyDict_GetItemString(pyDict,
            "execute_user_defined_function");
    OC_ASSERT(pyExecuteUserFunc != NULL);

    // PyModule_GetDict returns a borrowed reference, so don't do this:
    // Py_DECREF(pyDict);

    // Promote the pyExecuteUserFunc reference since it is only borrowed
    // and we don't want it to go away behind the scenes.
    Py_INCREF(pyExecuteUserFunc);

    // Create the argument list.
    pyArgs = PyTuple_New(2);
    pyUUID = PyLong_FromLong(varargs.value());
    OC_ASSERT(pyUUID != NULL);
    PyTuple_SetItem(pyArgs, 0, pyUserFunc);
    PyTuple_SetItem(pyArgs, 1, pyUUID);

    // Execute the user function and store its return value.
    pyReturnValue = PyObject_CallObject(pyExecuteUserFunc, pyArgs);

    // Check for errors.
    pyError = PyErr_Occurred();
    if (!pyError) {
        // Success - get the return value.
        errorCallingFunction = false;
        uuid = static_cast<unsigned long>(PyLong_AsLong(pyReturnValue));

    } else {
        // Error - save the message.
        errorCallingFunction = true;

        // Construct the error message.
        PyObject *pyErrorType, *pyErrorMessage, *pyTraceback;
        PyErr_Fetch(&pyErrorType, &pyErrorMessage, &pyTraceback);
        std::stringstream errorStringStream;
        errorStringStream << "Python error in " << func.c_str();
        if (pyErrorMessage) {
            char* errorMessage = PyString_AsString(pyErrorMessage);
            if (errorMessage) {
                errorStringStream << ": " << errorMessage << ".";
            }
            
            // NOTE: The traceback can be NULL even when the others aren't.
            Py_DECREF(pyErrorType);
            Py_DECREF(pyErrorMessage);
            if (pyTraceback)
                Py_DECREF(pyTraceback);
        }
        errorString = errorStringStream.str();

        // Cleanup the reference count for pyError
        Py_DECREF(pyError);
    }
    
    // Cleanup the reference counts for Python objects we no longer reference.
    // Since we promoted the borrowed pyExecuteUserFunc reference, we need to
    // decrement it here.
    Py_DECREF(pyExecuteUserFunc);
    Py_DECREF(pyArgs);

    // In case PyObject_CallObject returns an error, we need to check for NULL.
    if (pyReturnValue)
        Py_DECREF(pyReturnValue);

    // NOTE: PyTuple_SetItem "steals" the reference to the object that
    // is set for the list item. That means that the list assumes it
    // owns the references after the call. So don't do this:
    // Py_DECREF(pyUserFunc);
    // Py_DECREF(pyUUID);

    // Release the GIL. No Python API allowed beyond this point.
    PyGILState_Release(gstate);

    // Return UNDEFINED on error.
    if (errorCallingFunction) {
        logger().error() << errorString;
        throw (RuntimeException(TRACE_INFO, errorString.c_str()));
        return Handle::UNDEFINED;
    }

    return Handle(uuid);
}

std::string PythonEval::apply_script(const std::string& script)
{
    PyObject* pyError = NULL;
    PyObject *pyCatcher = NULL;
    PyObject *pyOutput = NULL;
    std::string result;
    bool errorRunningScript;
    std::string errorString;

    // Grab the GIL
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    PyRun_SimpleString("_opencog_output_stream = StringIO.StringIO()\n"
                       "_python_output_stream = sys.stdout\n"
                       "sys.stdout = _opencog_output_stream\n"
                       "sys.stderr = _opencog_output_stream\n");

    // Run the script.
    PyRun_SimpleString(script.c_str());

    // Check for errors in the script.
    pyError = PyErr_Occurred();
    if (!pyError) {
        // Script succeeded, get the output stream as a string
        // so we can return it.
        errorRunningScript = false;
        pyCatcher = PyObject_GetAttrString(this->pyRootModule,
                "_opencog_output_stream");
        pyOutput = PyObject_CallMethod(pyCatcher, (char*)"getvalue", NULL);
        result = PyBytes_AsString(pyOutput);

        // Cleanup reference counts for Python objects we no longer reference.
        Py_DECREF(pyCatcher);
        Py_DECREF(pyOutput);

    } else {
        // Remember the error and get the error string for the throw below.
        errorRunningScript = true;

        // Construct the error message.
        PyObject *pyErrorType, *pyErrorMessage, *pyTraceback;
        PyErr_Fetch(&pyErrorType, &pyErrorMessage, &pyTraceback);
        std::stringstream errorStringStream;
        errorStringStream << "Python error";
        if (pyErrorMessage) {
            char* errorMessage = PyString_AsString(pyErrorMessage);
            if (errorMessage) {
                errorStringStream << ": " << errorMessage << ".";
            }
            
            // NOTE: The traceback can be NULL even when the others aren't.
            Py_DECREF(pyErrorType);
            Py_DECREF(pyErrorMessage);
            if (pyTraceback)
                Py_DECREF(pyTraceback);
        }
        errorString = errorStringStream.str();

        // Cleanup the reference count for the Python objects.
        Py_DECREF(pyError);
     }

    // Close the output stream.
    PyRun_SimpleString("sys.stdout = _python_output_stream\n"
                       "sys.stderr = _python_output_stream\n"
                       "_opencog_output_stream.close()\n");
 
    // Release the GIL. No Python API allowed beyond this point.
    PyGILState_Release(gstate);

    // If there was an error throw an exception so the user knows the
    // script had a problem.
    if (errorRunningScript) {
        logger().error() << errorString;
        throw (RuntimeException(TRACE_INFO, errorString.c_str()));
    }

    // printf("Python says that: %s\n", result.c_str());
    return result;
}

void PythonEval::addSysPath(std::string path)
{
    PyList_Append(this->pySysPath, PyBytes_FromString(path.c_str()));
    //    PyRun_SimpleString(("sys.path += ['" + path + "']").c_str());
}


const int ABSOLUTE_IMPORTS_ONLY = 0;

/**
* Add all the .py files in the given directory as modules to __main__ and
* keep the references in a dictionary (this->modules)
*/
void PythonEval::add_module_directory(const boost::filesystem::path &directory)
{
    vector<boost::filesystem::path> files;
    vector<boost::filesystem::path> pyFiles;
    string fileName, moduleName;

    // Loop over the files in the directory looking for Python files.
    copy(boost::filesystem::directory_iterator(directory),
            boost::filesystem::directory_iterator(), back_inserter(files));
    for(vector<boost::filesystem::path>::const_iterator it(files.begin());
            it != files.end(); ++it) {
        if(it->extension() == boost::filesystem::path(".py"))
            pyFiles.push_back(*it);
    }

    // Add the directory we are adding to Python's sys.path
    this->addSysPath(directory.c_str());

    // The pyFromList variable corresponds to what would appear in an
    // import statement after the import:
    //
    // from <module> import <from list>
    //
    // When this list is empty, as below, this corresponds to an import of the
    // entire module as is done in the simple import statement:
    //
    // import <module>
    //
    PyObject* pyModule = NULL;
    PyObject* pyFromList = PyList_New(0);

    // Import each of the ".py" files as a Python module.
    for(vector<boost::filesystem::path>::const_iterator it(pyFiles.begin());
            it != pyFiles.end(); ++it) {

        // Get the module name from the Python file name by removing the ".py"
        fileName = it->filename().c_str();
        moduleName = fileName.substr(0, fileName.length()-3);

        logger().info("    importing Python module: " + moduleName);

        // Import the entire module into the current Python environment.
        pyModule = PyImport_ImportModuleLevel((char*) moduleName.c_str(),
                this->pyGlobal, this->pyLocal, pyFromList,
                ABSOLUTE_IMPORTS_ONLY);

        // If the import succeeded...
        if (pyModule) {
            PyObject* pyModuleDictionary = PyModule_GetDict(pyModule);
            
            // Add the ATOMSPACE object to this module
            PyObject* pyAtomSpaceObject = this->getPyAtomspace();
            PyDict_SetItemString(pyModuleDictionary,"ATOMSPACE",
                    pyAtomSpaceObject);
            Py_DECREF(pyAtomSpaceObject);

            // Add the module name to the root module.
            PyModule_AddObject(this->pyRootModule, moduleName.c_str(), pyModule);

            // Add the module to our modules list. So don't decrement the
            // Python reference in this function.
            this->modules[moduleName] = pyModule;

        // otherwise, handle the error.
        } else {
            if(PyErr_Occurred())
                PyErr_Print();
            logger().warn() << "Couldn't import " << moduleName << 
                    " module from directory " << directory.c_str();
        }
    }

    // Cleanup the reference count for the Python objects.
    Py_DECREF(pyFromList);
}

/**
* Add the .py file in the given path as a module to __main__ and keep the
* references in a dictionary (this->modules)
*/
void PythonEval::add_module_file(const boost::filesystem::path &p)
{
    this->addSysPath(p.parent_path().c_str());

    string name;
    PyObject* mod;
    PyObject* pyList = PyList_New(0);

    name = p.filename().c_str();
    name = name.substr(0, name.length()-3);
    mod = PyImport_ImportModuleLevel((char *)name.c_str(), this->pyGlobal,
            this->pyLocal, pyList, 0);

    PyDict_SetItem(PyModule_GetDict(mod), PyBytes_FromString("ATOMSPACE"),
            this->getPyAtomspace());
    if(mod){
        PyDict_SetItem(PyModule_GetDict(mod), PyBytes_FromString("ATOMSPACE"),
                this->getPyAtomspace());
        PyModule_AddObject(this->pyRootModule, name.c_str(), mod);
        this->modules[name] = mod;
    }
    else{
        if(PyErr_Occurred())
            PyErr_Print();
        logger().warn() << "Couldn't import " << name << " module";
    }
    Py_DECREF(pyList);
}

/**
* Get a path from the user and call the corresponding function for
* directories and files
*/
void PythonEval::addModuleFromPath(std::string pathString)
{
    boost::filesystem::path modulePath(pathString);

    logger().info("Adding Python module (or directory): " + modulePath.string());

    if(boost::filesystem::exists(modulePath)){
        if(boost::filesystem::is_directory(modulePath))
            this->add_module_directory(modulePath);
        else
            this->add_module_file(modulePath);
    }
    else{
        logger().error() << modulePath << " doesn't exists";
    }

}

void PythonEval::eval_expr(const std::string& partial_expr)
{
    _result = "";
    if (partial_expr == "\n")
    {
        _pending_input = false;
        _result = this->apply_script(_input_line);
        _input_line = "";
    }
    else
    {
        // If the line ends with a colon, its not a complete expression,
        // and we must wait for more input.
        // XXX FIXME TODO: the same might be true if there was an open
        // parenthesis, and no close parentheis.  This neds to be
        // scanned for and fixed.
        size_t size = partial_expr.size();
        size_t colun = partial_expr.find_last_of(':');
        if (size-2 == colun)
            _pending_input = true;

        _input_line += partial_expr;

        if (not _pending_input)
        {
            _result = this->apply_script(_input_line);
            _input_line = "";
        }
    }
}

std::string PythonEval::poll_result()
{
	std::string r = _result;
	_result.clear();
	return r;
}
