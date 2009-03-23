/**
 * TickerTask.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Fri Oct  5 23:22:43 BRT 2007
 */

#ifndef TICKERTASK_H
#define TICKERTASK_H

#include <IdleTask.h>
#include <NetworkElement.h>
#include <time.h>
#include "SimulationParameters.h"

namespace PetaverseProxySimulator {

class TickerTask : public MessagingSystem::IdleTask {

    private:

        time_t lastTickTime;
        SimulationParameters& simParameters;
        int realTimeSecondsInOneTick;

    public:

        // ***********************************************/
        // Constructors/destructors

        ~TickerTask();
        TickerTask(SimulationParameters&);

        void run(MessagingSystem::NetworkElement *ne);

}; // class
}  // namespace

#endif
