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
#define ACTION_TRIED_PREDICATE_NAME "actionTried"
#define ACTION_DONE_PREDICATE_NAME "actionDone"
#define ACTION_FAILED_PREDICATE_NAME "actionFailed"
#define ACTION_STARTED_PREDICATE_NAME "actionStarted"
#define ACTION_REQUESTED_PREDICATE_NAME "ActionRequested"
#define OBJECT_STATE_PREDICATE_NAME "ObjectState"
#define AGISIM_POSITION_PREDICATE_NAME "AGISIM_position"
#define AGISIM_ROTATION_PREDICATE_NAME "AGISIM_rotation"
#define SIZE_PREDICATE_NAME "size"
#define OWNERSHIP_PREDICATE_NAME "owns"

// high level predicates
#define NEAR_PREDICATE_NAME "near"

// Goal names

#define INTERNAL_NOVELTY_GOAL_NAME "InternalNovelty"
#define OWNER_SATISFACTION_GOAL_NAME "SatisfyOwner"

#endif // _PREDEFINED_PROCEDURE_NAMES_H_
