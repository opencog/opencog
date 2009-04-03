/*
 * opencog/embodiment/AGISimSim/shared/include/world_vocabulary.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Ari A. Heljakka
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

#ifndef _WORLD_VOCABULARY_H
#define _WORLD_VOCABULARY_H

#ifdef __cplusplus
extern "C"
{
#endif

#define WROOT "world"
#define WOBJ "object"
#define WNAME "name"
#define WARG "property"

#define WSOUND "sound"
#define WFREQ "quality"
#define WINTENSITY "intensity"
#define WDURATION "duration"
#define WSMELL "smell"
#define WQUALITY "quality"
#define WTASTE "taste"

#define WDENSITY "density"
#define WENERGY "energy"

#ifdef __cplusplus
}
#endif

/*
 * Custom Sensation quality values
 */
// Original events
#define CUSTOM_SENSATION_BROADCAST_MESSAGE 110
#define CUSTOM_SENSATION_AGENT_DEATH 0
#define CUSTOM_SENSATION_NO_OBJECT_TO_EAT 104
#define CUSTOM_SENSATION_OBJECT_TOO_FAR_TO_EAT 102
#define CUSTOM_SENSATION_OBJECT_WITHOUT_ENERGY_CANNOT_BE_EAT 103
#define CUSTOM_SENSATION_ENERGY_GAIN 100
#define CUSTOM_SENSATION_ENERGY_LOSS 101
#ifdef USE_ACTUATOR_BASED_EVENTS
// Actuator-related events
#define CUSTOM_SENSATION_LEG_ACTUATOR 500
#define CUSTOM_SENSATION_LEG_ACTION_STARTED 501
#define CUSTOM_SENSATION_LEG_ACTION_DONE 502
#define CUSTOM_SENSATION_LEG_ACTION_FAILED 503
#define CUSTOM_SENSATION_ARM_ACTUATOR 600
#define CUSTOM_SENSATION_ARM_ACTION_STARTED 601
#define CUSTOM_SENSATION_ARM_ACTION_DONE 602
#define CUSTOM_SENSATION_ARM_ACTION_FAILED 603
#define CUSTOM_SENSATION_HEAD_ACTUATOR 700
#define CUSTOM_SENSATION_HEAD_ACTION_STARTED 701
#define CUSTOM_SENSATION_HEAD_ACTION_DONE 702
#define CUSTOM_SENSATION_HEAD_ACTION_FAILED 703
#else
// Animation-type-related events
#define CUSTOM_SENSATION_MOVE_ANIMATION 500
#define CUSTOM_SENSATION_MOVE_ANIMATION_STARTED 501
#define CUSTOM_SENSATION_MOVE_ANIMATION_DONE 502
#define CUSTOM_SENSATION_MOVE_ANIMATION_FAILED 503
#define CUSTOM_SENSATION_TURN_ANIMATION 600
#define CUSTOM_SENSATION_TURN_ANIMATION_STARTED 601
#define CUSTOM_SENSATION_TURN_ANIMATION_DONE 602
#define CUSTOM_SENSATION_TURN_ANIMATION_FAILED 603
#define CUSTOM_SENSATION_GENERIC_ANIMATION 700
#define CUSTOM_SENSATION_GENERIC_ANIMATION_STARTED 701
#define CUSTOM_SENSATION_GENERIC_ANIMATION_DONE 702
#define CUSTOM_SENSATION_GENERIC_ANIMATION_FAILED 703
#define CUSTOM_SENSATION_SEMIPOSE_ANIMATION 800
#define CUSTOM_SENSATION_SEMIPOSE_ANIMATION_STARTED 801
#define CUSTOM_SENSATION_SEMIPOSE_ANIMATION_DONE 802
#define CUSTOM_SENSATION_SEMIPOSE_ANIMATION_FAILED 803
#endif
// Aditional events
#define CUSTOM_SENSATION_HOLDING_OBJECT 900

#endif /* _WORLD_VOCABULARY_H */
