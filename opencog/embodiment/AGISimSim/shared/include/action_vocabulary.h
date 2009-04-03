/*
 * opencog/embodiment/AGISimSim/shared/include/action_vocabulary.h
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

//--- Complex actions -------------------------------------------------------

#define LIFT  "lift"
#define DROP  "drop"
#define GOTO  "goto"
#define SAY   "say"
#define FOLLOW  "follow"
#define WATCH  "watch"

//--- Agent movement --------------------------------------------------------
#define TURN_LEFT "turn.left"
#define TURN_RIGHT "turn.right"
#define FORWARD  "move.forward"
#define BACKWARD "move.backward"
#define STRAFE_LEFT "strafe.left"
#define STRAFE_RIGHT "strafe.right"
#define KICK_LOW    "kick.low"
#define KICK_HIGH   "kick.high"
#define WALK_TOWARDS "walk.towards"
#define NUDGE_TO "nudge.to"
#define TURN_TO "turn.to"

//--- Hand manipulation -----------------------------------------------------
#define HAND_LEFT "hand.left"
#define HAND_RIGHT "hand.right"
#define HAND_FORWARD "hand.forward"
#define HAND_BACKWARD "hand.backward"
#define HAND_UP  "hand.up"
#define HAND_DOWN "hand.down"

#define HAND_GRASP "hand.grasp"
#define HAND_UNGRASP "hand.ungrasp"

//--- Eye movement ----------------------------------------------------------
#define EYE_UP  "eye.up"
#define EYE_DOWN "eye.down"
#define EYE_LEFT "eye.left"
#define EYE_RIGHT "eye.right"

//--- Misc ------------------------------------------------------------------
#define EAT   "eat"
#define DRINK   "drink"
#define SMILE "smile"
#define FROWN "frown"

//--- Communication ---------------------------------------------------------
#define NOISE  "noise.make"
#define MESSAGE  "message"

//--- Cart put & get --------------------------------------------------------
#define CART  "cart.attach"
#define DECART  "cart.detach"

//--- Internal State Commands: Use these ops after the stem object: ---------
#define ON   "on"
#define OFF   "off"

//--- The Stem Objects: -----------------------------------------------------

//--- Whether to move cart along when adjacent ------------------------------
#define CART_MOVE "cart."

//--- Turning specific senses on / off --------------------------------------
#define SIGHT  "sight."
#define SMELL  "smell."
#define TASTE  "taste."
#define HEARING  "hearing."
#define TOUCH  "touch."

//--- How data is received --------------------------------------------------

#define TIME_CHUNK "time_quantum"
#define DATA_CHUNK "data_quantum"

//#define
