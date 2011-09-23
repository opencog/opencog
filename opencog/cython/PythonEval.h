/*
 * @file opencog/cython/PythonEval.h
 *
 * Simple python expression evaluator. 
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date   2011-09-20
 *
 * @Note
 *   Many code are copied directly from original /opencog/cython/PythonModule.h|cc 
 *   by Joel. I also borrowed some ideas from SchemeEval.h|cc 
 *
 *  @todo
 *   PythonEval can not work together with PythonModule. That is you should 
 *   disable PythonModule ('MODULES' in config file) before enabling C++ Fishgram
 *   mind agent. 
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

#ifndef OPENCOG_PYTHON_EVAL_H
#define OPENCOG_PYTHON_EVAL_H

#include <Python.h>

#include <string>

#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>
#include <opencog/server/Request.h>
#include <opencog/server/CogServer.h>

#include "PyMindAgent.h"
#include "PyRequest.h"
#include "agent_finder_types.h"

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
class PythonEval
{
	private:

		void init(void);

		// Make constructor, destructor private; force everyone to use the
        // singleton instance
		PythonEval(AtomSpace * atomspace) {
            this->atomspace = atomspace; 
            this->init(); 
        }

		~PythonEval();

		static PythonEval * singletonInstance;

		AtomSpace *atomspace;
		
        PyThreadState * mainThreadState;
        PyInterpreterState * mainInterpreterState;

	public:

        PyThreadState * getMainThreadState() {
            return this->mainThreadState; 
        }

        PyInterpreterState * getMainInterpreterState() {
            return this->mainInterpreterState;  
        }

        /**
         * Return a new reference of python AtomSpace, which holds c++ pointer 
         * of the given c++ AtomSpace. Then you can pass this python AtomSpace
         * to python functions within c++. 
         */
        PyObject * getPyAtomspace(AtomSpace * atomspace = NULL);

        /**
         * Display python dict in c++. 
         */
        void printDict(PyObject* obj); 

		// Use a singleton instance to avoid initializing python interpreter twice. 
		static PythonEval & instance(AtomSpace * atomspace = NULL)
		{
			if (!singletonInstance) {
                if (!atomspace) {
                    // Create our own local AtomSpace to send calls to the
                    // event loop (otherwise the getType cache breaks)
                    atomspace = new AtomSpace(*cogserver().getAtomSpace());
                }
				singletonInstance = new PythonEval(atomspace);
            }
            else if (atomspace && singletonInstance->atomspace->atomSpaceAsync != 
                     atomspace->atomSpaceAsync) {
                // Someone is trying to initialize the Python interpreter
                // on a different AtomSpace. because of the singleton design
                // there is no easy way to support this...
                throw (RuntimeException(TRACE_INFO, "Trying to re-initialize"
                            " python interpreter with different AtomSpaceAsync ptr!")
                      );
            }

			return *singletonInstance;
		}
};

} /* namespace opencog */

#endif /* OPENCOG_PYTHON_EVAL_H */
