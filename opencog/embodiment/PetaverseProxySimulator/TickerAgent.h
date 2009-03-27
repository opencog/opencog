/**
 * TickerAgent.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Fri Oct  5 23:22:43 BRT 2007
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
