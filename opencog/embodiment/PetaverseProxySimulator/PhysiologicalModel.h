/**
 * PhysiologicalModel.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Oct  3 23:05:06 BRT 2007
 */

#ifndef PHYSIOLOGICALMODEL_H
#define PHYSIOLOGICALMODEL_H

#include <vector>
#include <PetAction.h>
#include "SimulationParameters.h"

namespace PetaverseProxySimulator {

class PhysiologicalModel {

    private:

        SimulationParameters& simParameters;

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
        PhysiologicalModel(SimulationParameters&);

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
