/*
 * opencog/embodiment/PetaverseProxySimulator/TickerAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef TICKERAGENT_H
#define TICKERAGENT_H

#include <opencog/server/Factory.h>
#include <opencog/server/Agent.h>
#include <opencog/embodiment/Control/MessagingSystem/EmbodimentCogServer.h>
#include <time.h>
#include "SimulationConfig.h"

namespace PetaverseProxySimulator
{

using namespace opencog;

class TickerAgent : public Agent
{

private:

    time_t lastTickTime;
    int realTimeSecondsInOneTick;

public:

    virtual const ClassInfo& classinfo() const {
        return info();
    }
    static const ClassInfo& info() {
        static const ClassInfo _ci("PetaverseProxySimulator::TickerAgent");
        return _ci;
    }

    // ***********************************************/
    // Constructors/destructors

    ~TickerAgent();
    TickerAgent();
    void init();

    void run(CogServer *server);

}; // class
}  // namespace

#endif
