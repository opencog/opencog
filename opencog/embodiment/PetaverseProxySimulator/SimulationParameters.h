/*
 * opencog/embodiment/PetaverseProxySimulator/SimulationParameters.h
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

#ifndef SIMULATIONPARAMETERS_H
#define SIMULATIONPARAMETERS_H

#include <time.h>
#include "SystemParameters.h"

namespace PetaverseProxySimulator {

class SimulationParameters : public Control::SystemParameters {

    public:

        SimulationParameters();
        ~SimulationParameters();

        time_t simulationStart;
        time_t simulationTicks;

        void startSimulation();
        int getCurrentSimulationSeconds();
        void timeTick();

}; // class
}  // namespace

#endif
