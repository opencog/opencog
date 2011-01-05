;
; @file embodiment/pet_rules.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-01-04
;
; Scheme scripts for adding Modulators, Demands and Rules into AtomSpace
;

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
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

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
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

(define ActivationModulator 
    (add_modulator "ActivationModulator" 0.3)
)

(define ResolutionModulator 
    (add_modulator "ResolutionModulator" 0.3)
)

(define CertaintyModulator 
    (add_modulator "CertaintyModulator"  0.8)
)

(define SelectionThresholdModulator 
    (add_modulator "SelectionThresholdModulator" 0.85)
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
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

(define EnergyDemandSchema
    (add_demand_schema "EnergyDemand" 0.85)
)

(define WaterDemandSchema
    (add_demand_schema "WaterDemand" 0.80)
)

(define IntegrityDemandSchema
    (add_demand_schema "IntegrityDemand" 0.90)
)

(define AffiliationDemandSchema
    (add_demand_schema "AffiliationDemand" 0.70)
)

(define CertaintyDemandSchema
    (add_demand_schema "CertaintyDemand" 0.65)
)

(define CompetenceDemandSchema
    (add_demand_schema "CompetenceDemand" 0.65)
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Goals
;
; Usage:
;     (add_goal goal_pred_name)
;
; Goal is represented as:
;
; EvaluationLink
;     (SimpleTruthValue indicates how well the demand is satisfied)
;     (ShortTermInportance indicates the urgency of the demand)
;     PredicateNode "goal_pred_name"
;     ListLink (empty)
;
; There are two kinds of Goals, Final Goal and Intermediate Goal.
;
; A Final Goal, also known as Demand Goal, is to keep a specific Demand in a suitable range, 
; Which is the starting point of backward chaining. 
;
; While an Intermediate Goal should be used as other Rule's Precondition. 
;

; Final Goals, also known as Demand Goals
;

(define EnergyDemandGoal 
    (add_goal "EnergyDemandGoal")
)

(define WaterDemandGoal
    (add_goal "WaterDemandGoal")
)

(define IntegrityDemandGoal
    (add_goal "IntegrityDemandGoal")
)

(define AffiliationDemandGoal
    (add_goal "AffiliationDemandGoal")
)

(define CertaintyDemandGoal
    (add_goal "CertaintyDemandGoal")
)

(define CompetenceDemandGoal
    (add_goal "CompetenceDemandGoal")
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Connect Demands with their corresponding Demand Goals. 
;
; Usage:
;     (connect_demand_goal demand_schema_handle goal_handle min_acceptable_value max_acceptable_value)
;
; The connection is represented as:
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         (SimpleTruthValue indicates how well the demand is satisfied)
;         (ShortTermInportance indicates the urgency of the demand)
;         PredicateNode: "demand_name_goal" 
;         ListLink (empty)
;     EvaluationLink
;         GroundedPredicateNode: "FuzzyWithin"
;         ListLink
;             NumberNode: "min_acceptable_value"
;             NumberNode: "max_acceptable_value"
;             SimilarityLink (stv 1.0 1.0)
;                 NumberNode: "demand_value"
;                 ExecutionOutputLink
;                     GroundedSchemaNode: "demand_schema_name"
;                     ListLink
;                         PET_HANDLE
;

(define ConnectEnergyDemandGoal
    (connent_demand_goal EnergyDemandSchema         EnergyDemandGoal 0.1 1.0)
)

(define ConnectWaterDemandGoal
    (connect_demand_goal WaterDemandSchema          WaterDemandGoal 0.1 0.9)
)

(define ConnectIntegrityDemandGoal
    (connect_demand_goal IntegrityDemandSchema      IntegrityDemandGoal 0.3 1.0)
)

(define ConnectAffiliationDemandGoal
    (connect_demand_goal AffiliationDemandSchema    AffiliationDemandGoal 0.3 0.9)
)

(define ConnectCertaintyDemandGoal
    (connect_demand_goal CertaintyDemandSchema      CertaintyDemandGoal 0.2 1.0)
)

(define ConnectCompetenceDemandGoal
    (connect_demand_goal CompetenceDemandSchema     CompetenceDemandGoal 0.25 0.95)
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Rules
;
; Usage:
;     (add_rule tv_handle goal_handle action_handle precondition_handle_1 precondition_handle_2 ...)
;
; Note: 
;     Each Rule here means a cognitive schematic, that is 
;         Contex & Procedure ==> Goal
;
;     Rules here has nothing to do with that in PLN, so don't confuse them!
;
; A Rule is represented as below (AtTimeLink is missing currently): 
;
; PredictiveImplicationLink
;     AndLink
;         AtTimeLink
;             TimeNode CURRENT_TIMESTAMP
;             AndLink
;                 EvaluationLink
;                     GroundedPredicateNode "precondition_1_name"
;                     ListLink
;                         Node:arguments ...
;                         ...
;                 EvaluationLink
;                     PredicateNode         "precondition_2_name"
;                     ListLink (empty)
;                 ...
;                        
;         ExecutionLink
;             GroundedSchemaNode "schema_name"
;             ListLink
;                 Node:arguments ...
;                 ...
;
;     AtTimeLink
;         TimeNode CURRENT_TIMESTAMP
;         EvaluationLink
;             (SimpleTruthValue indicates how well the demand is satisfied)
;             (ShortTermInportance indicates the urgency of the demand)
;             PredicateNode: "demand_name_goal" 
;             ListLink (empty)
;
; For each Rule, there's only a Goal, an Action and a bunch of Preconditions. 
; And all these Preconditions should be grouped in an AndLink.
; If you want to use OrLink, then just split the Rule into several Rules.
; For the efficiency and simplicity of the planer (backward chainging), NotLink is forbidden currently.  
;

; Example, only for test
;

;(rule (cog-new-stv 0.8 0.8) "EatRulePrecondition" "beg_to_owner()" "EnergyDemand")
