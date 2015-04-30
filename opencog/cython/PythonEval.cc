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
 * NOTE: The Python C API reference counting is very tricky and the
 * API inconsistently handles PyObject* object ownership.
 * DO NOT change the reference count calls below using Py_INCREF
 * and the corresponding Py_DECREF until you completely understand
 * the Python API concepts of "borrowed" and "stolen" references.
 * Make absolutely NO assumptions about the type of reference the API
 * will return. Instead, verify if it returns a "new" or "borrowed"
 * reference. When you pass PyObject* objects to the API, don't assume
 * you still need to decrement the reference count until you verify
 * that the exact API call you are making does not "steal" the
 * reference: SEE: https://docs.python.org/2/c-api/intro.html?highlight=steals#reference-count-details
 * Remember to look to verify the behavior of each and every Py_ API call.
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

const int NO_SIGNAL_HANDLERS = 0;
const char* NO_FUNCTION_NAME = NULL;
const int SIMPLE_STRING_SUCCESS = 0;
const int SIMPLE_STRING_FAILURE = -1;
const int MISSING_FUNC_CODE = -1;

// The Python functions can't take const flags.
#define NO_COMPILER_FLAGS NULL

static bool already_initialized = false;
static bool initialized_outside_opencog = false;

void opencog::global_python_initialize()
{
    // We don't really know the gstate yet but we'll set it here to avoid
    // compiler warnings below.
    PyGILState_STATE gstate = PyGILState_UNLOCKED;

    logger().debug("[global_python_initialize] Start");

    // Throw an exception if this is called more than once.
    if (already_initialized) {
        throw opencog::RuntimeException(TRACE_INFO,
                "Python initializer global_python_init() called twice.");
    }

    // Remember this initialization.
    already_initialized = true;
   
    // Start up Python. 
    if (Py_IsInitialized())  {
        // If we were already initialized then someone else did it.
        initialized_outside_opencog = true;

        // Just grab the GIL
        gstate = PyGILState_Ensure();

    } else {
        // We are doing the initialization.
        initialized_outside_opencog = false;

        // Initialize Python (InitThreads grabs GIL implicitly)
        Py_InitializeEx(NO_SIGNAL_HANDLERS);
        PyEval_InitThreads();        
    }

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

    // Release the GIL, otherwise the Python shell hangs on startup.
    if (initialized_outside_opencog)
        PyGILState_Release(gstate);
    else
        PyEval_ReleaseLock();

    logger().debug("[global_python_initialize] Finish");
}

void opencog::global_python_finalize()
{
    logger().debug("[global_python_finalize] Start");

    // Cleanup Python.
    if (!initialized_outside_opencog)
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
        this->add_modules_from_path(preloadDirectory);
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
 
    // Get sys.path and keep the reference, used in this->add_to_sys_path()
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

    // Add ATOMSPACE to __main__ module.
    PyObject* pyRootDictionary = PyModule_GetDict(this->pyRootModule);
    PyObject* pyAtomSpaceObject = this->atomspace_py_object();
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

PyObject* PythonEval::atomspace_py_object(AtomSpace* atomspace)
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

void PythonEval::print_dictionary(PyObject* obj)
{
    if (!PyDict_Check(obj))
        return;

    PyObject *pyKey, *pyKeys;

    // Get the keys from the dictionary and print them.
    pyKeys = PyDict_Keys(obj);
    for (int i = 0; i < PyList_Size(pyKeys); i++) {

        // Get and print one key.
        pyKey = PyList_GetItem(pyKeys, i);
        char* c_name = PyBytes_AsString(pyKey);
        printf("%s\n", c_name);
 
        // PyList_GetItem returns a borrowed reference, so don't do this:
        // Py_DECREF(pyKey);
    }

    // Cleanup the Python reference count for the keys list.
    Py_DECREF(pyKeys);
}

/**
 * Build the Python error message for the current error. 
 *
 * Only call this when PyErr_Occurred() returns a non-null PyObject*.
 */
void PythonEval::build_python_error_message(    const char* function_name,
                                                std::string& errorMessage)
{
    PyObject *pyErrorType, *pyError, *pyTraceback, *pyErrorString;
    std::stringstream errorStringStream;

    // Get the error from Python.
    PyErr_Fetch(&pyErrorType, &pyError, &pyTraceback);
    
    // Construct the error message string.
    errorStringStream << "Python error ";
    if (function_name != NO_FUNCTION_NAME)
        errorStringStream << "in " << function_name;
    if (pyError) {
        pyErrorString = PyObject_Str(pyError);
        char* pythonErrorString = PyString_AsString(pyErrorString);
        if (pythonErrorString) {
            errorStringStream << ": " << pythonErrorString << ".";
        } else {
            errorStringStream << ": Undefined Error";
        }
        
        // Cleanup the references. NOTE: The traceback can be NULL even
        // when the others aren't.
        Py_DECREF(pyErrorType);
        Py_DECREF(pyError);
        Py_DECREF(pyErrorString);
        if (pyTraceback)
            Py_DECREF(pyTraceback);

    } else {
        errorStringStream << ": Undefined Error";
    }
    errorMessage = errorStringStream.str();
}

/**
 * Execute the python string at the __main__ module context. 
 *
 * This replaces a call to PyRun_SimpleString which clears errors so
 * that a subsequent call to Py_Error() returns false. This version
 * does everything that PyRun_SimpleString does except it does not
 * call PyErr_Print() and PyErr_Clear().
 */
void PythonEval::execute_string(const char* command)
{
    PyObject *pyRootDictionary, *pyResult;
    pyRootDictionary = PyModule_GetDict(this->pyRootModule);
    pyResult = PyRun_StringFlags(command, Py_file_input, pyRootDictionary,
            pyRootDictionary, NO_COMPILER_FLAGS);
    if (pyResult)
        Py_DECREF(pyResult);
    Py_FlushLine();
}

int PythonEval::argument_count(PyObject* pyFunction)
{
    PyObject* pyFunctionCode;
    PyObject* pyArgumentCount;
    int argumentCount;

    // Get the 'function.func_code.co_argcount' Python internal attribute.
    pyFunctionCode = PyObject_GetAttrString(pyFunction, "func_code");
    if (pyFunctionCode) {
        pyArgumentCount = PyObject_GetAttrString(pyFunctionCode, "co_argcount");
        if (pyArgumentCount) {
            argumentCount = PyInt_AsLong(pyArgumentCount);
        }  else {
            Py_DECREF(pyFunctionCode);
            return MISSING_FUNC_CODE;
        }
    } else {
        return MISSING_FUNC_CODE;
    }

    // Cleanup the reference counts.
    Py_DECREF(pyFunctionCode);
    Py_DECREF(pyArgumentCount);

    return argumentCount;
}

/**
 * Get the Python module and stripped function name given the identifer of
 * the form 'module.function'.
 * 
 */
PyObject* PythonEval::module_for_function(  const std::string& moduleFunction,
                                            std::string& functionName)
{
    PyObject* pyModule;
    std::string moduleName;

    // Get the correct module and extract the function name.
    int index = moduleFunction.find_first_of('.');
    if (index < 0){
        pyModule = this->pyRootModule;
        functionName = moduleFunction;
        moduleName = "__main__";
    } else {
        moduleName = moduleFunction.substr(0, index);
        pyModule = this->modules[moduleName];
        functionName = moduleFunction.substr(index+1);
    }

    return pyModule;
}

/**
 * Call the user defined function with the arguments passed in the
 * ListLink handle 'arguments'.
 * 
 * On error throws an exception.
 */
PyObject* PythonEval::call_user_function(   const std::string& moduleFunction, 
                                            Handle arguments)
{
    PyObject *pyError, *pyModule, *pyUserFunc, *pyReturnValue = NULL;
    PyObject *pyDict;
    std::string functionName;
    std::string errorString;

    // Grab the GIL.
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    // Get the module and stripped function name.
    pyModule = this->module_for_function(moduleFunction, functionName);

    // If we can't find that module then throw an exception.
    if (!pyModule) {
        PyGILState_Release(gstate);
        logger().error("Python module for '%s' not found!", moduleFunction.c_str());
        throw (RuntimeException(TRACE_INFO, "Python module for '%s' not found!",
                moduleFunction.c_str()));
    }
        
    // Get a reference to the user function.
    pyDict = PyModule_GetDict(pyModule);
    pyUserFunc = PyDict_GetItemString(pyDict, functionName.c_str());

    // PyModule_GetDict returns a borrowed reference, so don't do this:
    // Py_DECREF(pyDict);

    // If we can't find that function then throw an exception.
    if (!pyUserFunc) {
        PyGILState_Release(gstate);
        logger().error("Python function '%s' not found!", moduleFunction.c_str());
        throw (RuntimeException(TRACE_INFO, "Python function '%s' not found!",
                moduleFunction.c_str()));
    }
        
    // Promote the borrowed reference for pyUserFunc since it will
    // be passed to a Python C API function later that "steals" it.
    Py_INCREF(pyUserFunc);

    // Make sure the function is callable.
    if (!PyCallable_Check(pyUserFunc)) {
        Py_DECREF(pyUserFunc);
        PyGILState_Release(gstate);
        logger().error("Python user function '%s' not callable!",
                moduleFunction.c_str());
        throw (RuntimeException(TRACE_INFO,
            "Python function '%s' not callable!", moduleFunction.c_str()));
    }

    // Get the expected argument count.
    int expectedArgumentCount = this->argument_count(pyUserFunc);
    if (expectedArgumentCount == MISSING_FUNC_CODE) {
        PyGILState_Release(gstate);
        throw (RuntimeException(TRACE_INFO,
            "Python function '%s' error missing 'func_code'!",
            moduleFunction.c_str()));
    }

    // Get the actual argument count, passed in the ListLink.
    if (arguments->getType() != LIST_LINK) {
        PyGILState_Release(gstate);
        throw RuntimeException(TRACE_INFO,
            "Expecting arguments to be a ListLink!");
    }
    LinkPtr linkArguments(LinkCast(arguments));
    int actualArgumentCount = linkArguments->getArity();

    // Now make sure the expected count matches the actual argument count.
    if (expectedArgumentCount != actualArgumentCount) {
        PyGILState_Release(gstate);
        throw (RuntimeException(TRACE_INFO,
            "Python function '%s' which expects '%d arguments,"
            " called with %d arguments!", moduleFunction.c_str(),
            expectedArgumentCount, actualArgumentCount
            ));
    }

    // Create the Python tuple for the function call with python
    // atoms for each of the atoms in the link arguments.
    PyObject* pyArguments = PyTuple_New(actualArgumentCount);
    PyObject* pyAtomSpace = this->atomspace_py_object();
    const HandleSeq& argumentHandles = linkArguments->getOutgoingSet();
    int tupleItem = 0;
    for (HandleSeq::const_iterator it = argumentHandles.begin();
        it != argumentHandles.end(); ++it) {

        // Place a Python atom object for this handle into the tuple.
        PyObject* pyAtom = py_atom(it->value(), pyAtomSpace);
        PyTuple_SetItem(pyArguments, tupleItem, pyAtom);

        // PyTuple_SetItem steals it's item so don't do this:
        // Py_DECREF(pyAtom)

        ++tupleItem;
    }
    Py_DECREF(pyAtomSpace);

    // Execute the user function and store its return value.
    pyReturnValue = PyObject_CallObject(pyUserFunc, pyArguments);
    
    // Cleanup the reference counts for Python objects we no longer reference.
    // Since we promoted the borrowed pyExecuteUserFunc reference, we need
    // to decrement it here. Do this before error checking below since we'll
    // need to decrement these references even if there is an error.
    Py_DECREF(pyUserFunc);
    Py_DECREF(pyArguments);

    // Check for errors.
    pyError = PyErr_Occurred();
    if (pyError) {

        // Construct the error message and throw an exception.
        this->build_python_error_message(moduleFunction.c_str(), errorString);
        PyGILState_Release(gstate);
        throw RuntimeException(TRACE_INFO, errorString.c_str());

        // PyErr_Occurred returns a borrowed reference, so don't do this:
        // Py_DECREF(pyError);
    }

    // Release the GIL. No Python API allowed beyond this point.
    PyGILState_Release(gstate);

    return pyReturnValue;
}

Handle PythonEval::apply(const std::string& func, Handle varargs)
{
    PyObject *pyReturnAtom = NULL;
    PyObject *pyError, *pyAtomUUID = NULL;
    UUID uuid = 0;

    // Get the atom object returned by this user function.
    pyReturnAtom = this->call_user_function(func, varargs);

    // If we got a non-null atom were no errors.
    if (pyReturnAtom) {

        // Grab the GIL.
        PyGILState_STATE gstate;
        gstate = PyGILState_Ensure();

        // Get the handle UUID from the atom.
        pyAtomUUID = PyObject_CallMethod(pyReturnAtom, (char*) "handle_uuid",
                NULL);

        // Make sure we got an atom UUID.
        pyError = PyErr_Occurred();
        if (pyError || !pyAtomUUID) {
            PyGILState_Release(gstate);
            throw RuntimeException(TRACE_INFO, 
                "Python function '%s' did not return Atom!", func.c_str());
        }

        // Get the UUID from the python UUID.
        uuid = static_cast<unsigned long>(PyLong_AsLong(pyAtomUUID));

        // Cleanup the reference counts.
        Py_DECREF(pyReturnAtom);
        Py_DECREF(pyAtomUUID);

        // Release the GIL. No Python API allowed beyond this point.
        PyGILState_Release(gstate);

    } else {

        throw RuntimeException(TRACE_INFO, 
                "Python function '%s' did not return Atom!", func.c_str());
    }

    return Handle(uuid);
}

/**
 * Apply the user function to the arguments passed in varargs and return
 * the extracted truth value.
 */
TruthValuePtr PythonEval::apply_tv(const std::string& func, Handle varargs)
{
    PyObject *pyTruthValue = NULL;
    PyObject *pyError, *pyTruthValuePtrPtr = NULL;
    TruthValuePtr* tvpPtr;
    TruthValuePtr tvp;

    // Get the python truth value object returned by this user function.
    pyTruthValue = this->call_user_function(func, varargs);

    // If we got a non-null truth value there were no errors.
    if (pyTruthValue) {

        // Grab the GIL.
        PyGILState_STATE gstate;
        gstate = PyGILState_Ensure();

        // Get the truth value pointer from the object (will be encoded
        // as a long by PyVoidPtr_asLong)
        pyTruthValuePtrPtr = PyObject_CallMethod(pyTruthValue, 
                (char*) "truth_value_ptr_object", NULL);

        // Make sure we got a truth value pointer.
        pyError = PyErr_Occurred();
        if (pyError || !pyTruthValuePtrPtr) {
            PyGILState_Release(gstate);
            throw RuntimeException(TRACE_INFO, 
                "Python function '%s' did not return TruthValue!",
                func.c_str());
        }

        // Get the pointer to the truth value pointer. Yes, it does
        // contain a pointer to the shared_ptr not the underlying.
        tvpPtr = static_cast<TruthValuePtr*>
                (PyLong_AsVoidPtr(pyTruthValuePtrPtr));

        // Assign the truth value pointer using this pointer before
        // we decrement the reference to pyTruthValue since that
        // will delete this pointer.
        tvp = *tvpPtr;

        // Cleanup the reference counts.
        Py_DECREF(pyTruthValuePtrPtr);
        Py_DECREF(pyTruthValue);

        // Release the GIL. No Python API allowed beyond this point.
        PyGILState_Release(gstate);

    } else {

        throw RuntimeException(TRACE_INFO, 
                "Python function '%s' did not return TruthValue!",
                func.c_str());
    }

    return tvp;
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

    // Execute the script. NOTE: This call replaces PyRun_SimpleString 
    // which was masking errors because it calls PyErr_Clear() so the
    // call to PyErr_Occurred below was returning false even when there 
    // was an error.
    this->execute_string(script.c_str());

    // Check for errors in the script.
    pyError = PyErr_Occurred();

    // If the script executed without error...
    if (!pyError) {
        // Get the output stream as a string so we can return it.
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
        this->build_python_error_message(NO_FUNCTION_NAME, errorString);

        // PyErr_Occurred returns a borrowed reference, so don't do this:
        // Py_DECREF(pyError);
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

void PythonEval::add_to_sys_path(std::string path)
{
    PyObject* pyPathString = PyBytes_FromString(path.c_str());
    PyList_Append(this->pySysPath, pyPathString);

    // We must decrement because, unlike PyList_SetItem, PyList_Append does
    // not "steal" the reference we pass to it. So this:
    //
    // PyList_Append(this->pySysPath, PyBytes_FromString(path.c_str()));
    // 
    // leaks memory. So we need to save the reference as above and
    // decrement it, as below.
    //
    Py_DECREF(pyPathString);
}


const int ABSOLUTE_IMPORTS_ONLY = 0;

void PythonEval::import_module( const boost::filesystem::path &file,
                                PyObject* pyFromList)
{
    // The pyFromList parameter corresponds to what would appear in an
    // import statement after the import:
    //
    // from <module> import <from list>
    //
    // When this list is empty, this corresponds to an import of the
    // entire module as is done in the simple import statement:
    //
    // import <module>
    //
    string fileName, moduleName;

    // Get the module name from the Python file name by removing the ".py"
    fileName = file.filename().c_str();
    moduleName = fileName.substr(0, fileName.length()-3);

    logger().info("    importing Python module: " + moduleName);

    // Import the entire module into the current Python environment.
    PyObject* pyModule = PyImport_ImportModuleLevel((char*) moduleName.c_str(),
            this->pyGlobal, this->pyLocal, pyFromList,
            ABSOLUTE_IMPORTS_ONLY);

    // If the import succeeded...
    if (pyModule) {
        PyObject* pyModuleDictionary = PyModule_GetDict(pyModule);
        
        // Add the ATOMSPACE object to this module
        PyObject* pyAtomSpaceObject = this->atomspace_py_object();
        PyDict_SetItemString(pyModuleDictionary,"ATOMSPACE",
                pyAtomSpaceObject);

        // This decrement is needed because PyDict_SetItemString does
        // not "steal" the reference, unlike PyList_SetItem.
        Py_DECREF(pyAtomSpaceObject);

        // We need to increment the pyModule reference because
        // PyModule_AddObject "steals" it and we're keeping a copy
        // in our modules list.
        Py_INCREF(pyModule);

        // Add the module name to the root module.
        PyModule_AddObject(this->pyRootModule, moduleName.c_str(), pyModule);

        // Add the module to our modules list. So don't decrement the
        // Python reference in this function.
        this->modules[moduleName] = pyModule;

    // otherwise, handle the error.
    } else {
        if(PyErr_Occurred())
            PyErr_Print();
        logger().warn() << "Couldn't import '" << moduleName << "' module";
    }

}

/**
* Add all the .py files in the given directory as modules to __main__ and
* keep the references in a dictionary (this->modules)
*/
void PythonEval::add_module_directory(const boost::filesystem::path &directory)
{
    vector<boost::filesystem::path> files;
    vector<boost::filesystem::path> pyFiles;

    // Loop over the files in the directory looking for Python files.
    copy(boost::filesystem::directory_iterator(directory),
            boost::filesystem::directory_iterator(), back_inserter(files));
    for(vector<boost::filesystem::path>::const_iterator it(files.begin());
            it != files.end(); ++it) {
        if(it->extension() == boost::filesystem::path(".py"))
            pyFiles.push_back(*it);
    }

    // Add the directory we are adding to Python's sys.path
    this->add_to_sys_path(directory.c_str());

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
    PyObject* pyFromList = PyList_New(0);

    // Import each of the ".py" files as a Python module.
    for(vector<boost::filesystem::path>::const_iterator it(pyFiles.begin());
            it != pyFiles.end(); ++it)
        this->import_module(*it, pyFromList);

    // Cleanup the reference count for the from list.
    Py_DECREF(pyFromList);
}

/**
* Add the .py file in the given path as a module to __main__ and add the
* reference to the dictionary (this->modules)
*/
void PythonEval::add_module_file(const boost::filesystem::path &file)
{
    // Add this file's parent path to sys.path so Python imports
    // can find it.
    this->add_to_sys_path(file.parent_path().c_str());

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

    // Import this file as a module.
    PyObject* pyFromList = PyList_New(0);
    this->import_module(file, pyFromList);
    Py_DECREF(pyFromList);
}

/**
* Get a path and determine if it is a file or directory, then call the
* corresponding function specific to directories and files.
*/
void PythonEval::add_modules_from_path(std::string pathString)
{
    boost::filesystem::path modulePath(pathString);

    logger().info("Adding Python module (or directory): " + modulePath.string());

    // Grab the GIL
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    if(boost::filesystem::exists(modulePath)){
        if(boost::filesystem::is_directory(modulePath))
            this->add_module_directory(modulePath);
        else
            this->add_module_file(modulePath);
    }
    else{
        logger().error() << modulePath << " doesn't exists";
    }

    // Release the GIL. No Python API allowed beyond this point.
    PyGILState_Release(gstate);
}

void PythonEval::eval_expr(const std::string& partial_expr)
{
    if (partial_expr == "\n")
        logger().info("[PythonEval] eval_expr: '\\n'");
    else
        logger().info("[PythonEval] eval_expr:\n%s\n", partial_expr.c_str());

    _result = "";

    // If we get a newline by itself, then we are finished.
    if (partial_expr == "\n")
    {
        _pending_input = false;
    }
    else
    {
        // XXX FIXME TODO: Need to check if there was an open
        // parenthesis, and no close parentheis.

        // If the line ends with a colon, its not a complete expression,
        // and we must wait for more input, i.e. more input is pending.
        size_t expression_size = partial_expr.size();
        size_t colon_position = partial_expr.find_last_of(':');
        if (colon_position == (expression_size - 2))
            _pending_input = true;

        // Add this expression to our evaluation buffer.
        _input_line += partial_expr;
    }

    // Process the evaluation buffer if more input is not pending.
    if (not _pending_input)
    {
        _result = this->apply_script(_input_line);
        _input_line = "";
    }
}

std::string PythonEval::poll_result()
{
	std::string r = _result;
	_result.clear();
	return r;
}
