/*
 * opencog/reasoning/pln/ForwardChainingAgent.h
 *
 * Copyright (C) 2010 by OpenCog Foundation
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

#ifndef FORWARDCHAININGAGENT_H_
#define FORWARDCHAININGAGENT_H_

#include <opencog/server/CogServer.h>
#include <opencog/server/Agent.h>

#include "PLN.h"
#include "ForwardChainer.h"

namespace opencog
{

/** ForwardChainingAgent runs PLN forward chaining inference.
 *
 */
class ForwardChainingAgent : public Agent
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

    ForwardChainer fc;


    //! @todo should be parameter(s)
    int stepsPerCycle; // but only after deciding on a good set of parameters

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::ForwardChainingAgent");
        return _ci;
    }

    ForwardChainingAgent(CogServer&);
    virtual ~ForwardChainingAgent();
    virtual void run();

}; // class

}  // namespace

#endif /* FORWARDCHAININGAGENT_H_ */
