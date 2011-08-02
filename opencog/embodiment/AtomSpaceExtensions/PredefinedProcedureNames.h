/*
 * opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef _PREDEFINED_PROCEDURE_NAMES_H_
#define _PREDEFINED_PROCEDURE_NAMES_H_
/**
 * PredefinedProcedureNames.h
 *
 * Defines all procedure names used
 */

// Schema names

#define SAY_SCHEMA_NAME "say"
#define UNKNOWN_TRICK_SCHEMA_NAME "unknownTrick"
#define PAY_ATTENTION_SCHEMA_NAME "PAYATTENTION"

// Predicate names

// basic predicates
#define PLAN_DONE_PREDICATE_NAME "planDone"
#define PLAN_FAILED_PREDICATE_NAME "planFailed"
#define ACTION_TRIED_PREDICATE_NAME "actionTried"
#define ACTION_DONE_PREDICATE_NAME "actionDone"
#define ACTION_FAILED_PREDICATE_NAME "actionFailed"
#define ACTION_STARTED_PREDICATE_NAME "actionStarted"
#define ACTION_REQUESTED_PREDICATE_NAME "ActionRequested"
#define OBJECT_STATE_PREDICATE_NAME "ObjectState"
#define AGISIM_POSITION_PREDICATE_NAME "AGISIM_position"
#define AGISIM_VELOCITY_PREDICATE_NAME "AGISIM_velocity"
#define AGISIM_ROTATION_PREDICATE_NAME "AGISIM_rotation"
#define SIZE_PREDICATE_NAME "size"
#define OWNERSHIP_PREDICATE_NAME "owns"

// high level predicates
#define NEAR_PREDICATE_NAME "near"

// Modulator names
//
// Used by WorldWrapperUtil::evalPerception method
// They should be the schema names used in "xxx_rules.scm" 
// without the suffix "ModulatorUpdater" and 
// should be exactly the same as defined in "EmbodimentConfig.h"
#define ACTIVATION_MODULATOR_NAME "Activation"
#define RESOLUTION_MODULATOR_NAME "Resolution"
#define SECURING_THRESHOLD_MODULATOR_NAME "SecuringThreshold"
#define SELECTION_THRESHOLD_MODULATOR_NAME "SelectionThreshold"

// Demand names
//
// Used by WorldWrapperUtil::evalPerception method
// They should be the schema names used in "xxx_rules.scm" 
// without the suffix "DemandUpdater" and 
// should be exactly the same as defined in "EmbodimentConfig.h"
#define ENERGY_DEMAND_NAME       "Energy" 
#define WATER_DEMAND_NAME        "Water"
#define INTEGRITY_DEMAND_NAME    "Integrity"
#define AFFILIATION_DEMAND_NAME  "Affiliation"
#define CERTAINTY_DEMAND_NAME    "Certainty"
#define COMPETENCE_DEMAND_NAME   "Competence"

// high level predicates - traits
#define CURIOSITY_PREDICATE_NAME     "curiosity"
#define PLAYFULNESS_PREDICATE_NAME    "playfulness"
#define FEARFULNESS_PREDICATE_NAME    "fearfulness"
#define EXCITABILITY_PREDICATE_NAME    "excitability"
#define FRIENDLINESS_PREDICATE_NAME    "friendliness"
#define AGGRESSIVENESS_PREDICATE_NAME  "aggressiveness"
#define APPRECIATIVENESS_PREDICATE_NAME "appreciativeness"

// high level predicates - pet signals
#define HUNGER_PREDICATE_NAME   "hunger"
#define THIRST_PREDICATE_NAME    "thirst"
#define ENERGY_PREDICATE_NAME    "energy"
#define FITNESS_PREDICATE_NAME   "fitness"
#define PEE_URGENCY_PREDICATE_NAME  "pee_urgency"
#define POO_URGENCY_PREDICATE_NAME  "poo_urgency"

// high level predicates - pet emotional feelings
#define FEAR_PREDICATE_NAME     "fear"
#define LOVE_PREDICATE_NAME     "love"
#define HATE_PREDICATE_NAME     "hate"
#define ANGER_PREDICATE_NAME     "anger"
#define PRIDE_PREDICATE_NAME     "pride"
#define HAPPINESS_PREDICATE_NAME  "happiness"
#define GRATITUDE_PREDICATE_NAME  "gratitude"
#define EXCITEMENT_PREDICATE_NAME "excitement"

// Goal names

#define INTERNAL_NOVELTY_GOAL_NAME "InternalNovelty"
#define OWNER_SATISFACTION_GOAL_NAME "SatisfyOwner"

#define SELECTED_RULE_PREDICATE_NAME "SelectedRule"

#endif // _PREDEFINED_PROCEDURE_NAMES_H_
