/*
 * @file opencog/embodiment/Control/OperationalAvatarController/FishgramAgent.cc
 *
 * Make the Fishgram developed by Jared, run in background (in a seperate thread). 
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-09-13
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

#ifdef HAVE_CYTHON

#include "OAC.h"
#include "FishgramAgent.h"

#include "opencog/web/json_spirit/json_spirit.h"

#include <boost/bind.hpp>

// for backward compatibility as from boost 1.46 filesystem 3 is the default
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem/operations.hpp>

#include <boost/foreach.hpp>
#ifndef foreach
#define foreach  BOOST_FOREACH
#endif

using namespace opencog::oac;

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

FishgramAgent::~FishgramAgent()
{
    this->stopFishgram(); 

    PyGILState_STATE gState = PyGILState_Ensure(); 
    Py_XDECREF(this->pFishgram); 
    PyGILState_Release(gState); 

    logger().info("FishgramAgent::%s", __FUNCTION__);

    this->fishgram_thread->join(); 
    delete this->fishgram_thread; 
}

FishgramAgent::FishgramAgent()
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle. 
    this->forceInitNextCycle();
}

void FishgramAgent::init(opencog::CogServer * server) 
{
    logger().debug( "FishgramAgent::%s - Initialize the Agent [ cycle = %d ]",
                    __FUNCTION__, 
                    this->cycleCount
                  );

    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    const AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Initialize python interpreter (if it has not be done) 
    PythonEval::instance(oac->getAtomSpace()); 

    // Create a new thread and run fishgram inside
    this->fishgram_thread = new boost::thread( boost::bind(&FishgramAgent::startFishgram, this, server) );

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void FishgramAgent::startFishgram(opencog::CogServer * server)
{
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Used by embedded python
    PyGILState_STATE gState = PyGILState_Ensure(); 

    PythonEval & python_eval = PythonEval::instance(); 

    // Import fishgram python module
    PyObject * pModuleName = PyString_FromString("fishgram");
    PyObject * pModule = PyImport_Import(pModuleName);
    Py_XDECREF(pModuleName);

    if (!pModule) {
        if (PyErr_Occurred())
            PyErr_Print();

        logger().error("FishgramAgent::%s Can not import fishgram module.", __FUNCTION__); 
        return; 
    }
    else 
        logger().debug("FishgramAgent::%s Import fishgram module successfully.", __FUNCTION__); 

    // Get and print all the info of the fishgram module
    PyObject * pDict = PyModule_GetDict(pModule);  

    if (!pDict) {  
        logger().error("FishgramAgent::%s Can not get any info of fishgram module.", __FUNCTION__); 

        Py_XDECREF(pModule); 
        return; 
    }  

    python_eval.printDict(pDict); 
    Py_XDECREF(pDict); 

    // Get the fishgram class
    PyObject * pClass = PyObject_GetAttrString(pModule, "Fishgram");
    Py_XDECREF(pModule); 

    if ( !pClass || !PyCallable_Check(pClass) ) {
        if (PyErr_Occurred())
            PyErr_Print();
    
        logger().error("FishgramAgent::%s Can not get the class: Fishgram.", __FUNCTION__); 
        return; 
    }
    else 
        logger().debug("FishgramAgent::%s Get fishgram class successfully.", __FUNCTION__); 

    // Build python AtomSpace
    PyObject * pAtomSpace = python_eval.getPyAtomspace(oac->getAtomSpace());

    if (!pAtomSpace)
    {
        if (PyErr_Occurred())
            PyErr_Print();

        logger().error("FishgramAgent::%s Can not create python AtomSpace.", __FUNCTION__); 
        return; 
    }
    else
        logger().debug("FishgramAgent::%s Build python AtomSpace successfully.", __FUNCTION__); 

    // Build arguments
    PyObject * pArgs = Py_BuildValue("(O)", pAtomSpace);
    Py_XDECREF(pAtomSpace); 

    if (!pArgs) {
        if (PyErr_Occurred())
            PyErr_Print();

        logger().error("FishgramAgent::%s Can not build python arguments.", __FUNCTION__); 

        Py_XDECREF(pClass); 
        return; 
    }
    else
        logger().debug("FishgramAgent::%s Build python arguments successfully.", __FUNCTION__); 

    // Create an instance of fishgram
    this->pFishgram = PyInstance_New(pClass, pArgs, NULL);  
    Py_XDECREF(pClass); 
    Py_XDECREF(pArgs); 

    if (!pFishgram) { 
        if (PyErr_Occurred())
            PyErr_Print();

        logger().error("FishgramAgent::%s Can not create an instance of fishgram.", __FUNCTION__); 
        return; 
    }
    else
        logger().debug("FishgramAgent::%s Create an instance of fishgram successfully.", __FUNCTION__); 

    // Start fishgram
    PyObject * pResult = PyObject_CallMethod(this->pFishgram, "start", NULL);
    Py_XDECREF(pResult); 

    logger().debug("FishgramAgent::%s Run fishgram successfully.", __FUNCTION__); 

    PyGILState_Release(gState);
}

void FishgramAgent::stopFishgram()
{
    PyGILState_STATE gState = PyGILState_Ensure(); 

    PyObject * pResult = PyObject_CallMethod(this->pFishgram, "stop", NULL);
    Py_XDECREF(pResult); 

    logger().debug("FishgramAgent::%s Stop fishgram successfully.", __FUNCTION__); 

    PyGILState_Release(gState);
}

void FishgramAgent::run(opencog::CogServer * server)
{
    this->cycleCount ++;

    logger().debug( "FishgramAgent::%s - Executing run %d times",
                     __FUNCTION__, 
                     this->cycleCount
                  );
    // Get OAC
    OAC * oac = (OAC *) server;

    // Get AtomSpace
    AtomSpace & atomSpace = * ( oac->getAtomSpace() );

    // Get petId
    const std::string & petId = oac->getPet().getPetId();

    // Get current time stamp
    unsigned long timeStamp = oac->getPAI().getLatestSimWorldTimestamp();   

    // Initialize the Agent (modulatorMetaMap etc)
    if ( !this->bInitialized )
        this->init(server);
}

#endif /*HAVE_CYTHON*/

