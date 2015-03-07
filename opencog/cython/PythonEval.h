/*
 * @file opencog/cython/PythonEval.h
 *
 * Simple python expression evaluator.
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 *         Ramin Barati <rekino@gmail.com>
 *         Keyvan Mir Mohammad Sadeghi <keyvan@opencog.org>
 *         Curtis Faith <curtis.m.faith@gmail.com>
 * @date   2011-09-20
 *
 * @Note
 *   Zhenhua: Many code are copied directly from original /opencog/cython/PythonModule.h|cc
 *            by Joel. I also borrowed some ideas from SchemeEval.h|cc
 *
 *   Ramin: This class is completely revised by me and Keyvan. The new code is inspired
 *          by Linas' SchemeEval and borrowed some ideas from Joel's PythonModule.
 *
 *  @todo
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

#ifndef OPENCOG_PYTHON_EVAL_H
#define OPENCOG_PYTHON_EVAL_H
#ifdef HAVE_CYTHON

#include "PyIncludeWrapper.h"

#include <map>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>

#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/shell/GenericEval.h>


namespace opencog {

class AtomSpace;
class CogServer;

/**
 * Each call of the embedded python code could be easily locked by object of this class
 */
class PythonThreadLocker
{
    private:
        PyGILState_STATE state;

    public:
        PythonThreadLocker() : state(PyGILState_Ensure())
        {}

        ~PythonThreadLocker() {
            PyGILState_Release(state);
        }
};

/**
 * Singleton class used to initialize python interpreter in the main thread.
 * It also provides some handy functions, such as getPyAtomspace. These helper
 * functions may need python GIL and you should do this manually.
 */
class PythonEval : public GenericEval
{
    private:
        void initialize_python_objects_and_imports(void);

        // Module utility functions
        void import_module( const boost::filesystem::path &file,
                            PyObject* pyFromList);
        void add_module_directory(const boost::filesystem::path &directory);
        void add_module_file(const boost::filesystem::path &file);
        void add_modules_from_path(std::string path);

        // Python utility functions
        PyObject* call_user_function(   const std::string& func,
                                        Handle varargs);
        void build_python_error_message(    const char* function_name,
                                            std::string& errorMessage);
        void add_to_sys_path(std::string path);
        PyObject * atomspace_py_object(AtomSpace * atomspace = NULL);
        void print_dictionary(PyObject* obj);
        void execute_string(const char* command);

        static PythonEval* singletonInstance;
        static AtomSpace* singletonAtomSpace;

        AtomSpace* _atomspace;

        PyObject* pyGlobal;
        PyObject* pyLocal;
        PyObject* pyRootModule;

        PyObject* pySysPath;

        std::map <std::string, PyObject*> modules;

        std::string _result;

    public:
        PythonEval(AtomSpace* atomspace);
        ~PythonEval();

        /**
         * Create the singleton instance with the supplied atomspace.
         */
        static void create_singleton_instance(AtomSpace * atomspace);

        /**
         * Delete the singleton instance.
         */
        static void delete_singleton_instance();

        /**
         * Get a reference to the singleton instance.
         */
        static PythonEval & instance(AtomSpace * atomspace = NULL);

        // The async-output interface.
        virtual void begin_eval() {}
        virtual void eval_expr(const std::string&);
        virtual std::string poll_result();

        // The synchronous-output interface.
        std::string eval(const std::string& expr)
            { begin_eval(); eval_expr(expr); return poll_result(); }

        /**
         * Runs the Python code contained in 'script'.
         */
        std::string apply_script(const std::string& script);

        /**
         * Calls the Python function passed in 'func' returning a Handle.
         */
        Handle apply(const std::string& func, Handle varargs);

        /**
         * Calls the Python function passed in 'func' returning a TruthValuePtr.
         */
        TruthValuePtr apply_tv(const std::string& func, Handle varargs);

        /**
         *
         */
        void print_root_dictionary()
            { this->print_dictionary(PyModule_GetDict(this->pyRootModule)); }

};

/**
 * Initialize Python. Must be called before any Python dependent modules
 * are loaded.
 */ 
void global_python_initialize();

/**
 * Finalize Python. Call to cleanup memory used by Python interpreters.
 */
void global_python_finalize();

} /* namespace opencog */

#endif /* HAVE_CYTHON */
#endif /* OPENCOG_PYTHON_EVAL_H */
