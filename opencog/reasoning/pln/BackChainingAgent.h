/*
 * opencog/reasoning/pln/BackChainingAgent.h
 *
 * Copyright (C) 2009 by OpenCog Foundation
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
 * All Rights Reserved
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

#ifndef _OPENCOG_BACK_CHAINING_AGENT_H
#define _OPENCOG_BACK_CHAINING_AGENT_H

#include <opencog/server/CogServer.h>
#include <opencog/server/Agent.h>
#include <opencog/util/RandGen.h>

#include "PLN.h"
//#include "BackInferenceTreeNode.h"

//using namespace opencog::pln;
//class BITNodeRoot;
namespace opencog {
namespace pln {
    class BITNodeRoot;
}}
using namespace opencog::pln;

namespace opencog
{

/** BackChainingAgent runs PLN backward chaining inference.
 * 
 * @todo Adapt MindAgent to check a RequestForService queue to see if
 * any other OpenCog processes need a particular hypothetical atom. 
 * @todo Allow backward chaining state to be saved between cycles (it will kind-of
already) and allow the iterative expansion of multiple BITs to be in progress at once.
 */
class BackChainingAgent : public Agent
{

private:

    /** Get Random number generator associated with Agent,
     * and instantiate if it does not already exist.
     *
     * @return Pointer to the internal random number generator.
     */
    opencog::RandGen* getRandGen();
    opencog::RandGen* rng; //!< Random number generator pointer.
    Logger *log;

    //! A boost::shared_ptr to the current BIT root.    
    boost::shared_ptr<BITNodeRoot> Bstate;
    //! A normal pointer to the current BIT root.
    BITNodeRoot *state;
    
    //! The number of steps run on the current target
    int steps;
    
    int maxSteps; //! @todo these should be parameters
    int stepsPerCycle; // but only after deciding on a good set of parameters

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::BackChainingAgent");
        return _ci;
    }

    BackChainingAgent(CogServer&);
    virtual ~BackChainingAgent();
    virtual void run();

    /** Choose a new target (done automatically after the current inference
     * either finishes or runs out of resources)
     */ 
    void chooseTarget();

}; // class

}  // namespace

#endif // _OPENCOG_BACK_CHAINING_AGENT_H
