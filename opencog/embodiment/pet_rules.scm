;
; @file embodiment/pet_rules.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2010-11-25
;
; Scheme scripts for adding Modulators, Demands and Rules into AtomSpace
;

;******************************************************************************
;
; Check that the PET_HANDLE, OWNER_HANDLE and CURRENT_TIMESTAMP are not null
;
; Because the Scheme shell would never know these Handles automatically, 
; which would be used by add_action function in "rules_core.scm",
; you should call the Scheme scripts as follows (just an example), 
;
;    (set! PET_HANDLE (get_agent_handle 'agent_id') )   
;    (set! OWNER_HANDLE (get_owner_handle 'owner_id') ) 
;    (set! CURRENT_TIMESTAMP 'current_time_stamp')
;
; firstly, in C++ code before actually loading any Rules!
;

; Check that PET_HANDLE is not null
;
(if (null? PET_HANDLE)
    (begin
        (print_debug_info INFO_TYPE_FAIL "pet_rules.scm" 
                          (string-append "PET_HANDLE is null. " 
                                        "Please call (set! PET_HANDLE (get_agent_handle 'agent_id') ) "
                                        "firstly, in c++ code before actually loading any Rules!"
                          )              
        );print_debug_info

        (exit -1)
    );begin
);if

; Check that OWNER_HANDLE is not null
;
(if (null? OWNER_HANDLE)
    (begin
        (print_debug_info INFO_TYPE_FAIL "pet_rules.scm" 
                          (string-append "PET_HANDLE is null. " 
                                         "Please call (set! OWNER_HANDLE (get_owner_handle 'owner_id') ) "
                                         "firstly, in c++ code before actually loading any Rules!"
                          )              
        );print_debug_info

        (exit -1)
    );begin
);if

; Check that CURRENT_TIMESTAMP is not null
;
(if (null? CURRENT_TIMESTAMP)
    (begin
        (print_debug_info INFO_TYPE_FAIL "pet_rules.scm" 
                          (string-append "CURRENT_TIMESTAMP is null. " 
                                         "Please call (set! CURRENT_TIMESTAMP 'current_time_stamp') "
                                         "firstly, in c++ code before actually loading any Rules!"
                          )              
        );print_debug_info

        (exit -1)
    );begin
);if

;******************************************************************************
;
; Add Modulators
;
; Usage:
;     (add_modulator modulator_name default_value)
;
; The updater of the modulator is a combo script, named after modulator_name with suffix "Updater".
; And the modulator_name should be something like xxxModulator, so the schema name is xxxModulatorUpdater
;
; Modulator is represented as:
;
; SimilarityLink (stv 1.0 1.0)
;     NumberNode: "modulator_value"
;     ExecutionOutputLink
;         GroundedSchemaNode: xxxModulatorUpdater
;         ListLink
;             PET_HANDLE
;

(add_modulator "ActivationModulator" 0.3)
(add_modulator "ResolutionModulator" 0.3)
(add_modulator "CertaintyModulator"  0.8)
(add_modulator "SelectionThresholdModulator" 0.85)

;******************************************************************************
;
; Add DemandSchemas
;
; Usage:
;     (add_demand_schema demand_name default_value)
;
; The updater of the demand value is a combo script, named after demand_name with suffix "Updater".
; And the demand_name should be something like xxxDemand, so the schema name is xxxDemandUpdater
; 
; DemandSchema/DemandValue is represented as:
;
; SimilarityLink (stv 1.0 1.0)
;     NumberNode: "demand_value"
;     ExecutionOutputLink
;         GroundedSchemaNode: "demand_schema_name"
;         ListLink
;             PET_HANDLE
;
; DemandValue is the output of DemandSchema.
;

(add_demand_schema "EnergyDemand" 0.85)
(add_demand_schema "WaterDemand" 0.80)
(add_demand_schema "IntegrityDemand" 0.90)
(add_demand_schema "AffiliationDemand" 0.70)
(add_demand_schema "CertaintyDemand" 0.65)
(add_demand_schema "CompetenceDemand" 0.65)

;******************************************************************************
;
; Add DemandGoals
;
; Usage:
;     (add_demand_goal demand_name min_acceptable_value max_acceptable_value default_value)
;
; DemandGoal is represented as:
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         PredicateNode: "demand_name_goal" 
;                        (SimpleTruthValue indicates how well the demand is satisfied)
;                        (ShortTermInportance indicates the urgency of the demand)
;     EvaluationLink
;         GroundedPredicateNode: "FuzzyWithin"
;         ListLink
;             NumberNode: "min_acceptable_value"
;             NumberNode: "max_acceptable_value"
;             SimilarityLink (stv 1.0 1.0)
;                 NumberNode: "demand_value"
;                 ExecutionOutputLink
;                     GroundedSchemaNode: "demand_schema_name"
;

(add_demand_goal "EnergyDemand" 0.1 1.0 0.85)
(add_demand_goal "WaterDemand" 0.1 0.9 0.80)
(add_demand_goal "IntegrityDemand" 0.3 1.0 0.90)
(add_demand_goal "AffiliationDemand" 0.3 0.9 0.70)
(add_demand_goal "CertaintyDemand" 0.2 1.0 0.65)
(add_demand_goal "CompetenceDemand" 0.25 0.95 0.65)

;******************************************************************************
;
; Add Rules
;
; Usage:
;     (rule rule_name tv_handle precondition_name action demand_name)
;
; Note: 
;     Each Rule here means a cognitive schematic, that is 
;         Contex & Procedure ==> Goal
;
;     Rules here has nothing to do with that in PLN, so don't confuse them!
;
; Rule is represented as:
;
; PredictiveImplicationLink (CompositeTruthValue indicates the mode strengths)
;     AndLink
;         AtTimeLink
;             TimeNode CURRENT_TIMESTAMP
;             EvaluationLink
;                 GroundedPredicateNode "precondition_name"
;                 ListLink (empty)
;             ExecutionLink
;                 GroundedSchemaNode "schema_name"
;                 ListLink
;                     Node:params ...
;                     ...
;     AtTimeLink
;         TimeNode CURRENT_TIMESTAMP
;         EvaluationLink
;             PredicateNode: "demand_name_goal" 
;                            (SimpleTruthValue indicates how well the demand is satisfied)
;                            (ShortTermInportance indicates the urgency of the demand)
;

; Example, only for test
(rule (cog-new-stv 0.8 0.8) "EatRulePrecondition" "beg_to_owner()" "EnergyDemand")
