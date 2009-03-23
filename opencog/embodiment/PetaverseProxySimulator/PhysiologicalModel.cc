/**
 * PhysiologicalModel.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Oct  3 23:05:06 BRT 2007
 */

#include "PhysiologicalModel.h"
#include <math.h>
#include "util/exceptions.h"
#include <ActionType.h>

using namespace PetaverseProxySimulator;

// values set according to PetBehaviorAspects.doc in such a way
// that the pet should eat 4 times a day and drink water 12 times a day
//const double PhysiologicalModel::PET_MASS = 240;
//const double PhysiologicalModel::FOOD_PER_EAT_COMMAND = 3.05;
//const double PhysiologicalModel::WATER_PER_DRINK_COMMAND = 0.55;

const double PhysiologicalModel::EAT_STOPS_PER_DAY = 4;
const double PhysiologicalModel::DRINK_STOPS_PER_DAY = 12;

const double PhysiologicalModel::PEE_STOPS_PER_DAY = 5;
const double PhysiologicalModel::POO_STOPS_PER_DAY = 2;

const double PhysiologicalModel::FITNESS_MAX = 100; // set according to PetBehaviorAspects.doc
const double PhysiologicalModel::INITIAL_FITNESS = 51; // arbitrary value; should be based in pet profile

const double PhysiologicalModel::ENERGY_MAX = 100;
const double PhysiologicalModel::INITIAL_ENERGY = 100; // set according to PetBehaviorAspects.doc

PhysiologicalModel::~PhysiologicalModel() {
}

PhysiologicalModel::PhysiologicalModel(SimulationParameters& _simParameters) : simParameters(_simParameters) {

    minutesPerTick = atof(simParameters.get("DEFAULT_SIMULATION_MINUTES_PER_TICK").c_str());

    petMode = IDLE;
    time[SLEEP] = 0;
    time[IDLE] = 0;
    time[ACTIVE] = 0;

    //amountOfEatenFood = 0;
    //amountOfDrunkWater = 0;

    //ticksSinceLastPee = 0;
    //ticksSinceLastPoo = 0;
    //waterToPee = 0;
    //foodToPoo = 0;

    reset();
}

void PhysiologicalModel::timeTick() {

    time[petMode]++;

    computeHunger();
    computeThirst();
    computePeeUrgency();
    computePooUrgency();
    computeFitness();
    computeEnergy();

    petMode = IDLE;
}

double PhysiologicalModel::ticksToMinutes(int ticks) {
    return (ticks * minutesPerTick);
}


void PhysiologicalModel::computeHunger() {
    /*
      double LAMBDA = ((double) 1 / 550); // set according to PetBehaviorAspects.doc
      // I didn't made LAMBDA global because in the document it may assume
      // different values for different monitored variables
      hunger = zeroOneCut(1 - exp(-LAMBDA * (12 * ticksToMinutes(time[ACTIVE]) + ticksToMinutes(time[IDLE]) - 20 * 1440 * (amountOfEatenFood / PET_MASS))));
    */
    hunger = zeroOneCut(hunger + (minutesPerTick / (2 * ((24 * 60) / EAT_STOPS_PER_DAY))));
}

void PhysiologicalModel::computeThirst() {
    /*
      double LAMBDA = ((double) 1 / 170); // set according to PetBehaviorAspects.doc
      // I didn't made LAMBDA global because in the document it may assume
      // different values for different monitored variables
      thirst = zeroOneCut(1 - exp(-LAMBDA * (12 * ticksToMinutes(time[ACTIVE]) + ticksToMinutes(time[IDLE]) - 40 * 1440 * (amountOfDrunkWater / PET_MASS))));
    */
    thirst = zeroOneCut(thirst + (minutesPerTick / (2 * ((24 * 60) / DRINK_STOPS_PER_DAY))));
}

void PhysiologicalModel::computePeeUrgency() {
    peeUrgency = zeroOneCut(peeUrgency + (minutesPerTick / (2 * ((24 * 60) / PEE_STOPS_PER_DAY))));
    /*
      ticksSinceLastPee++;
      double timeUrgency = zeroOneCut(ticksToMinutes(ticksSinceLastPee) / (((double) 24 * 60) / 5));
      double waterUrgency = zeroOneCut(waterToPee / ((12 / 5) * WATER_PER_DRINK_COMMAND));
      peeUrgency = zeroOneCut(timeUrgency + ((1 - timeUrgency) * waterUrgency));
    */
}

void PhysiologicalModel::computePooUrgency() {
    pooUrgency = zeroOneCut(pooUrgency + (minutesPerTick / (2 * ((24 * 60) / POO_STOPS_PER_DAY))));
    /*
      ticksSinceLastPoo++;
      double timeUrgency = zeroOneCut(ticksToMinutes(ticksSinceLastPoo) / (((double) 24 * 60) / 2));
      double foodUrgency = zeroOneCut(foodToPoo / ((12 / 5) * FOOD_PER_EAT_COMMAND));
      pooUrgency = zeroOneCut(timeUrgency + ((1 - timeUrgency) * foodUrgency));
    */
}

void PhysiologicalModel::computeFitness() {
    // TODO: implement proper formula
    // fitness is constant
}

void PhysiologicalModel::computeEnergy() {

    double minutes = ticksToMinutes(1);

    if (petMode == IDLE) {
        energy -= ((minutes / 720) * 80);
    } else if (petMode == SLEEP) {
        energy += 2 * minutes;
    }

    if (hunger > 0.9) {
        energy -= minutes;
    }

    if (thirst > 0.9) {
        energy -= minutes;
    }

    if (energy < 0) {
        energy = 0;
    } else if (energy > ENERGY_MAX) {
        energy = ENERGY_MAX;
    }
}

using namespace PerceptionActionInterface;

void PhysiologicalModel::processCommand(const PerceptionActionInterface::PetAction &petAction) {

    double cost = (1.5 - (fitness / FITNESS_MAX)) * actionCost(petAction);
    energy -= cost;

    ActionTypeCode actionCode = petAction.getType().getCode();

    switch (actionCode) {
    case BARK_CODE:
    case TRICK_FOR_FOOD_CODE:
    case WALK_CODE:
    case GRAB_CODE:
    case DROP_CODE:
    case SIT_CODE:
    case LIE_DOWN_CODE:
    case FLY_CODE:
    case FLY_FOLLOW_CODE:
    case STRETCH_CODE:
    case SCRATCH_SELF_NOSE_CODE:
    case SCRATCH_SELF_RIGHT_EAR_CODE:
    case SCRATCH_SELF_LEFT_EAR_CODE:
    case SCRATCH_SELF_NECK_CODE:
    case SCRATCH_SELF_RIGHT_SHOULDER_CODE:
    case SCRATCH_SELF_LEFT_SHOULDER_CODE:
    case SCRATCH_GROUND_BACK_LEGS_CODE:
    case RUN_IN_CIRCLE_CODE:
    case ANTICIPATE_PLAY_CODE:
    case BEG_CODE:
    case HEEL_CODE:
    case HIDE_FACE_CODE:
    case PLAY_DEAD_CODE:
    case FOLLOW_CODE:
    case LICK_CODE:
    case NUDGE_TO_CODE:
    case TAP_DANCE_CODE:
    case BARE_TEETH_CODE:
    case GROWL_CODE:
    case LOOK_UP_TURN_HEAD_CODE:
    case WHINE_CODE:
    case SNIFF_CODE:
    case SNIFF_AT_CODE:
    case SNIFF_PET_PART_CODE:
    case SNIFF_AVATAR_PART_CODE:
    case SHAKE_HEAD_CODE:
    case EARS_BACK_CODE:
    case EARS_TWITCH_CODE:
    case MOVE_HEAD_CODE:
    case WAG_CODE:
    case TAIL_FLEX_CODE:    
    case CHEW_CODE:
    case DREAM_CODE:
    case TURN_CODE:
    case SCRATCH_OTHER_CODE:
    case EARS_PERK_CODE:
    case JUMP_UP_CODE:
    case JUMP_TOWARD_CODE:
    case PAY_ATTENTION_CODE:
    case VOMIT_CODE:
    case LEAN_ROCK_DANCE_CODE:
    case BACK_FLIP_CODE:
    case WIDEN_EYES_CODE:
    case FEARFUL_POSTURE_CODE:
    case CLEAN_CODE:
    case BELCH_CODE:
    case WAKE_CODE: 
    case GREET_CODE:
    case DANCE1_CODE:
    case LOOK_RIGHT_CODE:
    case LOOK_LEFT_CODE:
    case KICK_LEFT_CODE:
    case KICK_RIGHT_CODE:
    case LEFT_EAR_PERK_CODE:
    case RIGHT_EAR_PERK_CODE:
    case LEFT_EAR_BACK_CODE:
    case RIGHT_EAR_BACK_CODE:
    case LEFT_EAR_TWITCH_CODE:
    case RIGHT_EAR_TWITCH_CODE:
    case ANGRY_EYES_CODE:
    case SAD_EYES_CODE:
    case HAPPY_EYES_CODE:
    case CLOSE_EYES_CODE:
    case BITE_CODE:
    case PET_CODE:
	case KICK_CODE:
	case GROUP_COMMAND_CODE:
	case RECEIVE_LATEST_GROUP_COMMANDS_CODE:
    case LOOK_AT_CODE:
        {
            // any command can wake up the pet
            petMode = ACTIVE;
            break;
        }
    case SLEEP_CODE: {
        petMode = SLEEP;
        break;
    }
    case EAT_CODE:  {
        // TODO: PetBehaviorAspects.doc is not clear on how to do this
        petMode = ACTIVE;
        energy += (ENERGY_MAX / 2);
        if (energy > ENERGY_MAX) {
            energy = ENERGY_MAX;
        }
        //amountOfEatenFood += FOOD_PER_EAT_COMMAND;
        //foodToPoo += FOOD_PER_EAT_COMMAND;
        hunger = 0;
        break;
    }
    case DRINK_CODE: {
        // TODO: PetBehaviorAspects.doc is not clear on how to do this
        petMode = ACTIVE;
        energy += (ENERGY_MAX / 2);
        if (energy > ENERGY_MAX) {
            energy = ENERGY_MAX;
        }
        //amountOfDrunkWater += WATER_PER_DRINK_COMMAND;
        //waterToPee += WATER_PER_DRINK_COMMAND;
        thirst = 0;
        break;
    }
    case PEE_CODE: {
        petMode = ACTIVE;
        //waterToPee = 0;
        //ticksSinceLastPee = 0;
        peeUrgency = 0;
        break;
    }
    case POO_CODE: {
        petMode = ACTIVE;
        //foodToPoo = 0;
        //ticksSinceLastPoo = 0;
        pooUrgency = 0;
        break;
    }
    case NUMBER_OF_ACTION_TYPES: {
        break;
    }
    }

    if (energy < 0) {
        energy = 0;
    } else if (energy > ENERGY_MAX) {
        energy = ENERGY_MAX;
    }
}


double PhysiologicalModel::actionCost(const PerceptionActionInterface::PetAction &petAction) {

    ActionTypeCode actionCode = petAction.getType().getCode();

    switch (actionCode) {
    case WHINE_CODE:                       return 0.5;
    case TAP_DANCE_CODE:                   return 0.5;
    case LOOK_UP_TURN_HEAD_CODE:           return 0.5;
    case GROWL_CODE:                       return 0.5;
    case BARE_TEETH_CODE:                  return 0.5;
    case HEEL_CODE:                        return 0.5;
    case DREAM_CODE:                       return 0.5;
    case HIDE_FACE_CODE:                   return 0.5;
    case ANTICIPATE_PLAY_CODE:             return 0.5;
    case SNIFF_CODE:                       return 0.5;
    case SNIFF_AT_CODE:                    return 0.5;
    case SNIFF_PET_PART_CODE:              return 0.5;
    case SNIFF_AVATAR_PART_CODE:           return 0.5;
    case SHAKE_HEAD_CODE:                  return 0.5;
    case EARS_BACK_CODE:                   return 0.5;
    case EARS_TWITCH_CODE:                 return 0.5;
    case MOVE_HEAD_CODE:                   return 0.5;
    case WAKE_CODE:                        return 0.5;
    case WAG_CODE:                         return 0.5;
    case LICK_CODE:                        return 0.5;
    case TAIL_FLEX_CODE:                   return 0.5;
    case CHEW_CODE:                        return 0.5;
    case BARK_CODE:                        return 0.5;
    case EARS_PERK_CODE:                   return 0.5;
    case SLEEP_CODE:                       return 0.5;
    case BELCH_CODE:                       return 0.5;
    case PAY_ATTENTION_CODE:               return 0.5;
    case WIDEN_EYES_CODE:                  return 0.5;
    case GREET_CODE:                       return 0.5;
    case LOOK_RIGHT_CODE:                  return 0.5;
    case LOOK_LEFT_CODE:                   return 0.5;
    case KICK_LEFT_CODE:                   return 0.5;
    case KICK_RIGHT_CODE:                  return 0.5;
    case LEFT_EAR_PERK_CODE:               return 0.5;
    case RIGHT_EAR_PERK_CODE:              return 0.5;
    case LEFT_EAR_BACK_CODE:               return 0.5;
    case RIGHT_EAR_BACK_CODE:              return 0.5;
    case LEFT_EAR_TWITCH_CODE:             return 0.5;
    case RIGHT_EAR_TWITCH_CODE:            return 0.5;
    case ANGRY_EYES_CODE:                  return 0.5;
    case SAD_EYES_CODE:                    return 0.5;
    case HAPPY_EYES_CODE:                  return 0.5;
    case CLOSE_EYES_CODE:                  return 0.5;
    case BITE_CODE:                        return 1.0;
    case GRAB_CODE:                        return 1.0;
    case DROP_CODE:                        return 1.0;
    case SIT_CODE:                         return 1.0;
    case LIE_DOWN_CODE:                    return 1.0;
    case STRETCH_CODE:                     return 1.0;
    case SCRATCH_SELF_NOSE_CODE:           return 1.0;
    case SCRATCH_SELF_RIGHT_EAR_CODE:      return 1.0;
    case SCRATCH_SELF_LEFT_EAR_CODE:       return 1.0;
    case SCRATCH_SELF_NECK_CODE:           return 1.0;
    case SCRATCH_SELF_RIGHT_SHOULDER_CODE: return 1.0;
    case SCRATCH_SELF_LEFT_SHOULDER_CODE:  return 1.0;
    case SCRATCH_GROUND_BACK_LEGS_CODE:    return 1.0;
    case BEG_CODE:                         return 1.0;
    case PLAY_DEAD_CODE:                   return 1.0;
    case TURN_CODE:                        return 1.0;
    case SCRATCH_OTHER_CODE:               return 1.0;
    case EAT_CODE:                         return 1.0;
    case DRINK_CODE:                       return 1.0;
    case PEE_CODE:                         return 1.0;
    case POO_CODE:                         return 1.0;
    case LEAN_ROCK_DANCE_CODE:             return 1.0;
    case BACK_FLIP_CODE:                   return 1.0;
    case FEARFUL_POSTURE_CODE:             return 1.0;
    case CLEAN_CODE:                       return 1.0;
    case VOMIT_CODE:                       return 1.5;
    case NUDGE_TO_CODE:                    return 1.5;
    case TRICK_FOR_FOOD_CODE:              return 1.5;
    case WALK_CODE:                        return 1.5;
    case RUN_IN_CIRCLE_CODE:               return 1.5;
    case FOLLOW_CODE:                      return 1.5;
    case JUMP_UP_CODE:                     return 2.0;
    case JUMP_TOWARD_CODE:                 return 2.0;
    case FLY_CODE:                         return 2.0;
    case FLY_FOLLOW_CODE:                  return 2.0;
    case DANCE1_CODE:                      return 2.0;
    case PET_CODE:                         return 1.0;
    case KICK_CODE:                        return 2.0;
    case GROUP_COMMAND_CODE:               return 0.5;
    case RECEIVE_LATEST_GROUP_COMMANDS_CODE: 
                                           return 0.5;
    case LOOK_AT_CODE:                     return 0.5;
    case NUMBER_OF_ACTION_TYPES:           return 0;
    }

    return 0; // not reached
}

double PhysiologicalModel::zeroOneCut(double f) {
    if (f > 1) {
        return 1;
    } else if (f < 0) {
        return 0;
    } else {
        return f;
    }
}

double PhysiologicalModel::getHunger() {
    return hunger;
}

double PhysiologicalModel::getThirst() {
    return thirst;
}

double PhysiologicalModel::getPeeUrgency() {
    return peeUrgency;
}

double PhysiologicalModel::getPooUrgency() {
    return pooUrgency;
}

double PhysiologicalModel::getFitness() {
    return fitness;
}

double PhysiologicalModel::getEnergy() {
    return energy;
}

double PhysiologicalModel::getScaledEnergy() {
    return zeroOneCut(energy / ENERGY_MAX);
}

double PhysiologicalModel::getScaledFitness() {
    return zeroOneCut(fitness / FITNESS_MAX);
}

void PhysiologicalModel::setMinutesPerTick(double f) {
    minutesPerTick = f;
}

void PhysiologicalModel::reset() {
    hunger = 0;       
    thirst = 0;       
    peeUrgency = 0;
    pooUrgency = 0;
    fitness = INITIAL_FITNESS;
    energy = INITIAL_ENERGY;
}
