/*
 * opencog/embodiment/PetaverseProxySimulator/PhysiologicalModel.h
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


#ifndef PHYSIOLOGICALMODEL_H
#define PHYSIOLOGICALMODEL_H

#include <vector>
#include <opencog/embodiment/Control/PerceptionActionInterface/PetAction.h>
#include "SimulationConfig.h"

namespace PetaverseProxySimulator
{

class PhysiologicalModel
{

private:

    typedef enum {SLEEP = 0, IDLE, ACTIVE} PetMode;

    //static const double PET_MASS;
    //static const double FOOD_PER_EAT_COMMAND;
    //static const double WATER_PER_DRINK_COMMAND;

    static const double EAT_STOPS_PER_DAY;
    static const double DRINK_STOPS_PER_DAY;

    static const double PEE_STOPS_PER_DAY;
    static const double POO_STOPS_PER_DAY;

    static const double FITNESS_MAX;
    static const double INITIAL_FITNESS;

    static const double ENERGY_MAX;
    static const double INITIAL_ENERGY;

    // Monitored state variables
    double hunger;
    double thirst;
    double peeUrgency;
    double pooUrgency;
    double fitness;
    double energy;

    PetMode petMode;
    int time[3];

    //double amountOfEatenFood;
    //double amountOfDrunkWater;

    //int ticksSinceLastPee;
    //int ticksSinceLastPoo;
    //double waterToPee;
    //double foodToPoo;

    double minutesPerTick;

    void computeHunger();
    void computeThirst();
    void computePeeUrgency();
    void computePooUrgency();
    void computeFitness();
    void computeEnergy();
    double zeroOneCut(double f);
    double actionCost(const PerceptionActionInterface::PetAction &petAction);
    double ticksToMinutes(int ticks);

public:

    // ***********************************************/
    // Constructors/destructors

    ~PhysiologicalModel();
    PhysiologicalModel();

    // ***********************************************/
    // API

    void processCommand(const PerceptionActionInterface::PetAction &petAction);
    void timeTick();

    double getHunger();
    double getThirst();
    double getPeeUrgency();
    double getPooUrgency();
    double getFitness();
    double getEnergy();
    double getScaledEnergy();
    double getScaledFitness();
    void setMinutesPerTick(double f);

    void reset();

}; // class
}  // namespace

#endif
