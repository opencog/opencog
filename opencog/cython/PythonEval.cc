/*
 * @file opencog/cython/PythonEval.cc
 * @author Zhenhua Cai <czhedu@gmail.com> Ramin Barati <rekino@gmail.com>
 *         Keyvan Mir Mohammad Sadeghi <keyvan@opencog.org>
 * @date 2011-09-20
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

#include <opencog/util/exceptions.h>
#include <opencog/util/foreach.h>
#include <opencog/util/Logger.h>
#include <opencog/util/misc.h>
#include <opencog/util/oc_assert.h>

#include <opencog/server/CogServer.h>

#include "PythonEval.h"
#include "opencog/atomspace_api.h"

using std::string;
using std::vector;

using namespace opencog;

//#define DPRINTF printf
#define DPRINTF(...)

PythonEval* PythonEval::singletonInstance = NULL;

static void global_python_init()
{
    static bool already_inited = false;

    // Avoid hard crash if already inited.
    if (already_inited) return;
    already_inited = true;

    // Start up Python (this init method skips registering signal
    // handlers)
    if (not Py_IsInitialized())
        Py_InitializeEx(0);
    if (not PyEval_ThreadsInitialized()) {
        PyEval_InitThreads();
        // Without this, pyshell hangs and does nothing.
        PyEval_ReleaseLock();
    }
}

void PythonEval::init(void)
{
    static bool eval_already_inited = false;

    // Avoid hard crash if already inited.
    if (eval_already_inited) return;
    eval_already_inited = true;

    global_python_init();

    logger().info("PythonEval::%s Initialising python evaluator.",
        __FUNCTION__);

    // Save a pointer to the main PyThreadState object
    this->mainThreadState = PyThreadState_Get();

    // Get a reference to the PyInterpreterState
    this->mainInterpreterState = this->mainThreadState->interp;

    // Getting the __main__ module
    this->pyRootModule = PyImport_AddModule("__main__");
    PyModule_AddStringConstant(this->pyRootModule, "__file__", "");

    PyRun_SimpleString(
                "import sys\n"
                "import StringIO\n"
                );

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

    // Add ATOMSPACE to __main__ module
    PyDict_SetItem(PyModule_GetDict(this->pyRootModule),
                   PyBytes_FromString("ATOMSPACE"),
                   this->getPyAtomspace());

    // These are needed for calling Python/C API functions, define 
    // them once and for all
    pyGlobal = PyDict_New();
    pyLocal = PyDict_New();

    // Getting sys.path and keeping the refrence, used in this->addSysPath()
    sys_path = PySys_GetObject((char*)"path");

    logger().info("PythonEval::%s Finished initialising python evaluator.",
        __FUNCTION__);
}

PyObject* PythonEval::getPyAtomspace(AtomSpace* atomspace)
{
    init();
    PyObject * pAtomSpace;

    if (atomspace)
        pAtomSpace = py_atomspace(atomspace);
    else
        pAtomSpace = py_atomspace(this->_atomspace);

    if (pAtomSpace != NULL)
        logger().debug("PythonEval::%s Get atomspace wrapped with python object",
                       __FUNCTION__);
    else {
        if (PyErr_Occurred())
            PyErr_Print();

        logger().error("PythonEval::%s Failed to get atomspace "
                       "wrapped with python object", __FUNCTION__);
    }

    return pAtomSpace;
}

void PythonEval::printDict(PyObject* obj)
{
    if (!PyDict_Check(obj))
        return;

    PyObject *k, *keys;
    keys = PyDict_Keys(obj);
    for (int i = 0; i < PyList_GET_SIZE(keys); i++) {
        k = PyList_GET_ITEM(keys, i);
        char* c_name = PyBytes_AsString(k);
        printf("%s\n", c_name);
    }
}

PythonEval::~PythonEval()
{
    logger().info("PythonEval::%s destructor", __FUNCTION__);
    Py_Finalize();

    delete pyLocal;
    delete pyRootModule;

    delete sys_path;
    delete pyGlobal;
}

/**
* Use a singleton instance to avoid initializing python interpreter
* twice.
*/
PythonEval& PythonEval::instance(AtomSpace* atomspace)
{
    if (!singletonInstance)
    {
        if (!atomspace)
            throw (RuntimeException(TRACE_INFO, "Null Atomspace!"));
        singletonInstance = new PythonEval(atomspace);
    }
    else if (atomspace and singletonInstance->_atomspace != atomspace)
    {
        // Someone is trying to initialize the Python interpreter on a
        // different AtomSpace.  Because of the singleton design of the
        // the CosgServer+AtomSpace, there is no easy way to support this...
        throw RuntimeException(TRACE_INFO, "Trying to re-initialize"
                " python interpreter with different AtomSpace ptr!");
    }
    return *singletonInstance;
}

Handle PythonEval::apply(const std::string& func, Handle varargs)
{
    PyObject *pError, *pyModule, *pFunc, *pExecFunc;
    PyObject *pArgs, *pUUID, *pValue = NULL;
    string moduleName;
    string funcName;

    init();

    // Get the correct module and extract the function name
    int index = func.find_first_of('.');
    if(index < 0){
        pyModule = this->pyRootModule;
        funcName = func;
        moduleName = "__main__";
    }
    else{
        moduleName = func.substr(0,index);
        pyModule = this->modules[moduleName];
        funcName = func.substr(index+1);
    }


    //    PyGILState_STATE _state = PyGILState_Ensure();
    // Get a refrence to the function
    pFunc = PyDict_GetItem(PyModule_GetDict(pyModule),
                           PyBytes_FromString(funcName.c_str()));

    OC_ASSERT(pFunc != NULL);
    if(!PyCallable_Check(pFunc))
    {
        logger().error() << "Member " << func << " is not callable.";
        return Handle::UNDEFINED;
    }

    // Get a refrence to our executer function
    pExecFunc = PyDict_GetItem(PyModule_GetDict(this->pyRootModule),
                    PyBytes_FromString("execute_user_defined_function"));
    OC_ASSERT(pExecFunc != NULL);

    // Create the argument list
    pArgs = PyTuple_New(2);
    pUUID = PyLong_FromLong(varargs.value());
    OC_ASSERT(pUUID != NULL);

    PyTuple_SetItem(pArgs, 0, pFunc);
    PyTuple_SetItem(pArgs, 1, pUUID);

    // Call the executer function and store its return value
    pValue = PyObject_CallObject(pExecFunc, pArgs);
    pError = PyErr_Occurred();

    if (pError) {
        PyErr_Print();
        logger().error() << PyBytes_AsString(PyObject_GetAttrString(pError, "message"));
        return Handle::UNDEFINED;
    }

    UUID uuid = static_cast<unsigned long>(PyLong_AsLong(pValue));

    // Cleaning up
    Py_DECREF(pArgs);
    Py_DECREF(pUUID);
    Py_DECREF(pFunc);
    Py_DECREF(pExecFunc);

    //    PyGILState_Release(_state);

    return Handle(uuid);
}

std::string PythonEval::apply_script(const std::string& script)
{
    init();

    std::string result;
//    PyObject* pError;
    //    PyGILState_STATE _state = PyGILState_Ensure();
    PyRun_SimpleString("_opencog_output_stream = StringIO.StringIO()\n"
                       "_python_output_stream = sys.stdout\n"
                       "sys.stdout = _opencog_output_stream\n"
                       "sys.stderr = _opencog_output_stream\n");

    PyRun_SimpleString(script.c_str());


    PyObject *catcher = PyObject_GetAttrString(this->pyRootModule,"_opencog_output_stream");
    PyObject *output = PyObject_CallMethod(catcher, (char*)"getvalue", NULL);

//    pError = PyErr_Occurred();

//    if(pError){
//        logger().error() << PyString_AsString(PyObject_GetAttrString(pError, "message")) << std::endl;
//        result = PyString_AsString(PyObject_GetAttrString(pError, "message"));
//    }
//    else
//    {
        result = PyBytes_AsString(output);
//    }

    PyRun_SimpleString("sys.stdout = _python_output_stream\n"
                       "sys.stderr = _python_output_stream\n"
                       "_opencog_output_stream.close()\n");
    //    PyGILState_Release(_state);

    // XXX TODO FIXME: we should check for error, and throw.  Without
    // this, python scripts can silently fail, and you'd never now it.

    // printf("Python says that: %s\n", result.c_str());
    return result;
}

void PythonEval::addSysPath(std::string path)
{
    init();

    PyList_Append(this->sys_path, PyBytes_FromString(path.c_str()));
    //    PyRun_SimpleString(("sys.path += ['" + path + "']").c_str());
}

/**
* Add all the .py files in the given directory as modules to __main__ and keeping the refrences
* in a dictionary (this->modules)
*/
void PythonEval::add_module_directory(const boost::filesystem::path &p)
{
    init();

    vector<boost::filesystem::path> files;
    vector<boost::filesystem::path> pyFiles;

    copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), back_inserter(files));

    for(vector<boost::filesystem::path>::const_iterator it(files.begin()); it != files.end(); ++it){
        if(it->extension() == boost::filesystem::path(".py"))
            pyFiles.push_back(*it);
    }

    this->addSysPath(p.c_str());

    string name;
    PyObject* mod;
    PyObject* pyList = PyList_New(0);
    for(vector<boost::filesystem::path>::const_iterator it(pyFiles.begin()); it != pyFiles.end(); ++it){
        name = it->filename().c_str();
        name = name.substr(0, name.length()-3);
        mod = PyImport_ImportModuleLevel((char *)name.c_str(), pyGlobal, pyLocal, pyList, 0);

        if(mod){
            PyDict_SetItem(PyModule_GetDict(mod), PyBytes_FromString("ATOMSPACE"), this->getPyAtomspace());
            PyModule_AddObject(this->pyRootModule, name.c_str(), mod);
            this->modules[name] = mod;
        }
        else{
            if(PyErr_Occurred())
                PyErr_Print();
            logger().warn() << "Couldn't import " << name << " module from folder " << p.c_str();
        }
    }
    Py_DECREF(pyList);
}

/**
* Add the .py file in the given path as a module to __main__ and keeping the refrences
* in a dictionary (this->modules)
*/
void PythonEval::add_module_file(const boost::filesystem::path &p)
{
    init();

    this->addSysPath(p.parent_path().c_str());

    string name;
    PyObject* mod;
    PyObject* pyList = PyList_New(0);

    name = p.filename().c_str();
    name = name.substr(0, name.length()-3);
    mod = PyImport_ImportModuleLevel((char *)name.c_str(), pyGlobal, pyLocal, pyList, 0);

    PyDict_SetItem(PyModule_GetDict(mod), PyBytes_FromString("ATOMSPACE"), this->getPyAtomspace());
    if(mod){
        PyDict_SetItem(PyModule_GetDict(mod), PyBytes_FromString("ATOMSPACE"), this->getPyAtomspace());
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
* Get a path from the user and call the coresponding function for directories and files
*/
void PythonEval::addModuleFromPath(std::string path)
{
    boost::filesystem::path p(path);

    if(boost::filesystem::exists(p)){
        if(boost::filesystem::is_directory(p))
            this->add_module_directory(p);
        else
            this->add_module_file(p);
    }
    else{
        logger().error() << path << " doesn't exists";
    }

}

void PythonEval::eval_expr(const std::string& partial_expr)
{
    init();

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
