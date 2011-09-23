/*
 * @file opencog/embodiment/Control/OperationalAvatarController/FishgramAgent.h
 *
 * Make the Fishgram developed by Jared, run in background (in a seperate thread). 
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date   2011-09-14
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

#ifndef FISHGRAMAGENT_H
#define FISHGRAMAGENT_H

#ifdef HAVE_CYTHON

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>
#include "opencog/cython/PythonEval.h"

#include <boost/thread/thread.hpp>

class FishgramAgentUTest;

namespace opencog { namespace oac {

/**
 * @class
*/
class FishgramAgent: public opencog::Agent
{
    friend class::FishgramAgentUTest; 

private:

    PyObject* pFishgram; 
    boost::thread * fishgram_thread; 

    unsigned long cycleCount;

    // Initialization
    void init(opencog::CogServer * server);

    // Create a new thread and run fishgram inside 
    void startFishgram(opencog::CogServer * server);

    void stopFishgram(); 

    bool bInitialized; 

public:

    FishgramAgent();
    virtual ~FishgramAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::FishgramAgent");
        return _ci;
    }

    // Entry of the Agent, CogServer will invoke this function during its cycle
    void run(opencog::CogServer * server);

    // After calling this function, the Agent will invoke its "init" method firstly 
    // in "run" function during its next cycle
    void forceInitNextCycle() {
        this->bInitialized = false;
    }

}; // class

} } // namespace opencog::oac

#endif /*FISHGRAMAGENT_H*/

#endif /*HAVE_CYTHON*/
