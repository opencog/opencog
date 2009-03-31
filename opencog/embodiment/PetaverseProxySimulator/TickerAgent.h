/*
 * opencog/embodiment/PetaverseProxySimulator/TickerAgent.h
 *
 * Copyright (C) 2007-2008 Andre Senna
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

#ifndef TICKERAGENT_H
#define TICKERAGENT_H

#include <opencog/server/Factory.h>
#include <opencog/server/Agent.h>
#include <EmbodimentCogServer.h>
#include <time.h>
#include "SimulationParameters.h"

namespace PetaverseProxySimulator {

using namespace opencog;

class TickerAgent : public Agent {

    private:

        time_t lastTickTime;
        SimulationParameters* simParameters;
        int realTimeSecondsInOneTick;

    public:

        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("PetaverseProxySimulator::TickerAgent");
            return _ci;
        }

        // ***********************************************/
        // Constructors/destructors

        ~TickerAgent();
        TickerAgent();
        void init(SimulationParameters&);

        void run(CogServer *server);

}; // class
}  // namespace

#endif
