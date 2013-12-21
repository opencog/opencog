;
; @file embodiment/pet_rules.scm
;
; @author Jinhua Chua <JinhuaChua@gmail.com>
; @date   2011-12-09
;
; Scheme scripts for adding Modulators, Demands and Rules into the AtomSpace.
;
; Many of the OpenPsi rules in this file are inspired from the original rules used by
; the old embodiment RuleEngine. 
;
; TODO: These Rules are very experimental and only for debugging. 
;       Once PsiActionSelectionAgent is finished, we would tune these Rules, observing Pet's behaviors.

;
; Initialize PET_HANDLE and OWNER_HANDLE for Test 
; If you run the Multiverse Client, comment the lines below 
;

(define PET_HANDLE (PetNode "TestPet") )
(define OWNER_HANDLE (AvatarNode "TestAvatar") )      

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Check that the PET_HANDLE and OWNER_HANDLE are not null
;
; Because the Scheme shell would never know these Handles automatically, 
; which would be used by add_action function in "rules_core.scm",
; you should call the Scheme scripts as follows (just an example), 
;
;    (set! PET_HANDLE (get_agent_handle 'agent_id') )   
;    (set! OWNER_HANDLE (get_owner_handle 'owner_id') ) 
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
; SimilarityLink
;     NumberNode: "modulator_value"
;     ExecutionOutputLink
;         GroundedSchemaNode: xxxModulatorUpdater
;

(define ActivationModulator 
    (add_modulator "ActivationModulator" 0.85)
)

(define ResolutionModulator 
    (add_modulator "ResolutionModulator" 0.8)
)

(define SecuringThreshold 
    (add_modulator "SecuringThresholdModulator"  0.60)
)

(define SelectionThresholdModulator 
    (add_modulator "SelectionThresholdModulator" 0.75)
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
; SimilarityLink
;     NumberNode: "demand_value"
;     ExecutionOutputLink
;         GroundedSchemaNode: "demand_schema_name"
;
; DemandValue is the output of DemandSchema.
;

(define EnergyDemandSchema
    (add_demand_schema "EnergyDemand" 0.85)
)

(define WaterDemandSchema
    (add_demand_schema "WaterDemand" 0.90)
)

(define IntegrityDemandSchema
    (add_demand_schema "IntegrityDemand" 0.85)
)

(define AffiliationDemandSchema
    (add_demand_schema "AffiliationDemand" 0.9)
)

(define CertaintyDemandSchema
    (add_demand_schema "CertaintyDemand" 0.50)
)

(define CompetenceDemandSchema
    (add_demand_schema "CompetenceDemand" 0.60)
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Goal to AtomSpace. 
;
; A Goal is an EvaluationLink with a PredicateNode or GroundedSchemaNode, 
; which can be represented as below:
;
; EvaluationLink
;     PredicateNode "ungrounded_goal_name"
;     ListLink
;         Node:arguments
;         ...
;
; or 
;
; EvaluationLink
;     GroundedPredicateNode "grounded_goal_name"
;     ListLink
;         Node:arguments
;         ...
;
; After you add an ungrounded goal, you should connect it with a grounded
; PredicateNode via 'connect_goal_updater' function
;
; Usage: 
;     (add_goal pred_or_gpn_handle . arguments)
;     (add_goal_updater gpn_handle . arguments)
;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Final Goals, also known as Demand Goals
;

(define EnergyDemandGoal 
    (add_goal (PredicateNode "EnergyDemandGoal") )
)

(define WaterDemandGoal
    (add_goal (PredicateNode "WaterDemandGoal") )
)

(define IntegrityDemandGoal
    (add_goal (PredicateNode "IntegrityDemandGoal") )
)

(define AffiliationDemandGoal
    (add_goal (PredicateNode "AffiliationDemandGoal") )
)

(define CertaintyDemandGoal
    (add_goal (PredicateNode "CertaintyDemandGoal") )
)

(define CompetenceDemandGoal
    (add_goal (PredicateNode "CompetenceDemandGoal") )
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Relation Goals, these usually serve as another Rule's precondition.
;

(define EntityVar (VariableNode "$EntityVar"))

(define FriendRelation
    (add_goal (PredicateNode "friend") "'self'" EntityVar) 
)

(define EnemyRelation
    (add_goal (PredicateNode "enemy") "'self'" EntityVar)
)

(define GratitudeRelation
    (add_goal (PredicateNode "gratitude") "'self'" EntityVar)
)

(define LoveRelation
    (add_goal (PredicateNode "love") "'self'" EntityVar)
)

(define FearRelation
    (add_goal (PredicateNode "fear") "'self'" EntityVar)
)

(define AngerRelation
    (add_goal (PredicateNode "anger") "'self'" EntityVar) 
)

(define KnowRelation
    (add_goal (PredicateNode "know") "'self'" EntityVar)
)

(define CuriousAboutRelation
    (add_goal (PredicateNode "curious_about") "'self'" EntityVar)
)

(define FamiliarWithRelation
    (add_goal (PredicateNode "familiar_with") "'self'" EntityVar)
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Connect a ungrounded goal and a grounded predicate node. 
;
; Usage:
;     (connect_goal_updater goal_evaluation_link goal_truth_value_updater_evaluation_link)
; 
; Each ungrounded goal or precondition should have a corresponding 
; GroundedPredicateNode to check if the goal or precondition has been achieved or
; not. They are related via a SimultaneousEquivalenceLink as follows:
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         PredicateNode "none_grounded_goal_or_precondition_name"
;         ListLink
;             ...
;
;     EvaluationLink
;         GroundedPredicateNode "updater_schema_name"
;         ListLink
;             ...
;
; Below is the specific case for DemandGoals in detail:
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         PredicateNode: "XxxDemandGoal" 
;     EvaluationLink
;         GroundedPredicateNode: "fuzzy_within"
;         ListLink
;             NumberNode: "min_acceptable_value"
;             NumberNode: "max_acceptable_value"
;             ExecutionOutputLink
;                 GroundedSchemaNode: "demand_schema_name"
;

(connect_goal_updater 
     EnergyDemandGoal
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.85 1.0 EnergyDemandSchema)
)

(connect_goal_updater
     WaterDemandGoal
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.90 1.0 WaterDemandSchema)
)

(connect_goal_updater
     IntegrityDemandGoal
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.80 1.0 IntegrityDemandSchema)
)

(connect_goal_updater   
     AffiliationDemandGoal 
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.8 1.0 AffiliationDemandSchema)
)

(connect_goal_updater
     CertaintyDemandGoal
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.70 1.0 CertaintyDemandSchema)
)

(connect_goal_updater
     CompetenceDemandGoal 
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.80 1.0 CompetenceDemandSchema) 
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Rules
;
; Usage: 
;
; (add_rule truth_value goal_evaluation_link action_execution_link
;     precondition_1
;     precondition_2
;     ...
; )   
;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; Rules for Relations (Only used for debugging)
;

;(define ObjBall
;    (ObjectNode "Ball") 
;)
;
;(define ObjChicken
;    (ObjectNode "Chicken")
;)
;
;(define ObjToy
;    (ObjectNode "Toy") 
;)
;
;(define EvaNoveltyBall
;    (EvaluationLink (stv 1.0 1.0)
;        (PredicateNode "RelationNovelty")
;        (ListLink
;            PET_HANDLE
;            ObjBall
;        )
;    )
;)
;
;(define EvaKnowBall
;    (EvaluationLink 
;        (PredicateNode "RelationKnow")
;        (ListLink
;            PET_HANDLE 
;            ObjBall
;        )
;    )
;)
;
;;(ImplicationLink (stv 1.0 1.0)
;;    EvaNoveltyBall
;;    EvaKnowBall
;;)
;
;(define EvaNovelty
;    (EvaluationLink 
;        (PredicateNode "RelationNovelty")
;        (ListLink
;            PET_HANDLE
;            EntityVar
;        )
;    ) 
;)
;
;(define EvaKnow
;    (EvaluationLink
;        (PredicateNode "RelationKnow")
;        (ListLink
;            PET_HANDLE
;            EntityVar
;        )
;    )
;)
;
;(AverageLink (stv 1.0 1.0)
;    (ListLink EntityVar)
;
;    (ImplicationLink 
;        EvaNovelty
;        EvaKnow
;    )
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to EnergyDemandGoal
;

;; build a block with random texture. offset should be a NumberNode
;(define (build_dark_block offset)
;    (OrLink
;        (add_action (GroundedSchemaNode "build_block") offset (WordNode "Stone") )
;        (add_action (GroundedSchemaNode "build_block") offset (WordNode "Dirt") )
;        (add_action (GroundedSchemaNode "build_block") offset (WordNode "TopSoil") )
;        (add_action (GroundedSchemaNode "build_block") offset (WordNode "Light") )
;    )
;)
;
;(define (build_light_block offset)
;    (OrLink
;        (add_action (GroundedSchemaNode "build_block") offset (WordNode "Lava") )
;        (add_action (GroundedSchemaNode "build_block") offset (WordNode "Leaves") )
;    )
;)
;
;(define multiple_step_forward
;    (SequentialAndLink
;        (build_dark_block (NumberNode "0") )
;;        (add_action (GroundedSchemaNode "step_forward") ) ; TODO: direction for step_backward is wrong sometimes
;        (add_action (GroundedSchemaNode "jump_forward") (NumberNode "1") ) 
;    ) 
;)
;
;; Rotate randomly
;(define random_rotate_right
;    (OrLink
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "73") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "79") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "83") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "89") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "97") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "101") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "103") )  
;    )
;)
;
;(define random_rotate_left
;    (OrLink
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "-73") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "-79") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "-83") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "-89") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "-97") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "-101") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "-103") )  
;    )
;)
;
;(define random_rotate_back
;    (OrLink
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "173") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "-179") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "181") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "-191") )  
;        (add_action (GroundedSchemaNode "rotate") (NumberNode "193") )  
;    )
;)
;
;(define random_rotate
;    (OrLink
;        random_rotate_left
;        random_rotate_left
;        random_rotate_left
;        random_rotate_right
;        random_rotate_right
;        random_rotate_right
;        random_rotate_right
;        random_rotate_back
;    )
;)
;
;(define random_build_destroy_blocks
;    (SequentialAndLink
;        random_rotate
;        (build_dark_block (NumberNode "1") )
;        (OrLink
;            (SequentialAndLink
;                (add_action (GroundedSchemaNode "jump_forward") (NumberNode "1") ) ;jump on to the block 
;                (build_dark_block (NumberNode "0") )
;            )
;            (SequentialAndLink
;                (add_action (GroundedSchemaNode "destroy_block") ) 
;                (add_action (GroundedSchemaNode "jump_forward") (NumberNode "1") ) ;jump on to the block 
;            )
;            (SequentialAndLink
;                (add_action (GroundedSchemaNode "destroy_block") ) 
;                (add_action (GroundedSchemaNode "jump_forward") (NumberNode "1") ) ;jump on to the block 
;            )
;        )
;    )        
;)    
;
;(define random_search
;;    random_build_destroy_blocks
;    (SequentialAndLink
;        random_rotate
;        (add_action (GroundedSchemaNode "step_forward") ) 
;    ) 
;)

;------------------------------------------------------------------------------
;
; This rule will guide the robot directly to the battery cube 
;

(define GetFoodGoal
    (AndLink 
        (add_goal (PredicateNode "is_edible") (VariableNode "$var_food") ) 
        (add_goal (PredicateNode "exist") (VariableNode "$var_food") )
    )
)

(AverageLink (stv 1 1)
    (ListLink
        (VariableNode "$var_food") 
    ) 

    (add_rule (stv 0 0) EnergyDemandGoal 
        (SequentialAndLink
            (add_action (GroundedSchemaNode "goto_obj") (VariableNode "$var_food") (NumberNode "2") )
            (add_action (GroundedSchemaNode "grab") (VariableNode "$var_food") )
            (add_action (GroundedSchemaNode "eat") (VariableNode "$var_food") )
        )
        GetFoodGoal
    )
)

;------------------------------------------------------------------------------
;
; These heuristic rules will help robot approaching the battery cube step by step
;
; Note: If you want to enable these rules, you should also comment the GetFoodGoal
;       and the corresponding rule above. Also you should enalbe the function calling 
;       of 'computeObserverInvolvedSpatialRelations' in file (line 73)
;       ./opencog/embodiment/Control/PredicateUpdaters/SpatialPredicateUpdater.cc
;       Finally, bacause of the limits of the action planner, make sure there's only 
;       one battery cube in the environment. Since the calculation of spatial relations 
;       between the robot and other objects is very time consuming, you should run 
;       opencog in real linux, rather than on a virtual machine. 
;

;(define GetFoodGoal
;    (AndLink 
;        (add_goal (PredicateNode "is_edible") (VariableNode "$var_food") ) 
;        (add_goal (PredicateNode "exist") (VariableNode "$var_food") )
;        (add_goal (PredicateNode "can_do") (ConceptNode "grab") PET_HANDLE (VariableNode "$var_food") )
;    )
;)
;
;(AverageLink (cog-new-av 1 1 1)
;    (ListLink
;        (VariableNode "$var_food") 
;    ) 
;
;    (add_rule (stv 1.0 1.0) EnergyDemandGoal 
;        (SequentialAndLink
;            (add_action (GroundedSchemaNode "grab") (VariableNode "$var_food") )
;            (add_action (GroundedSchemaNode "eat") (VariableNode "$var_food") )
;        )    
;        GetFoodGoal
;    )
;)    
;
;(AverageLink (cog-new-av 1 1 1)
;    (ListLink
;        (VariableNode "$var_food") 
;    ) 
;
;    (add_rule (stv 1.0 1.0) GetFoodGoal 
;        (SequentialAndLink
;            (build_dark_block (NumberNode "1") )
;            (add_action (GroundedSchemaNode "jump_forward") (NumberNode "1") ) ;jump on to the block 
;            (build_dark_block (NumberNode "0") )
;        )
;        (AndLink 
;            (add_goal (PredicateNode "is_edible") (VariableNode "$var_food") ) 
;            (add_goal (PredicateNode "exist") (VariableNode "$var_food") )
;            (add_precondition (PredicateNode "above") (VariableNode "$var_food") PET_HANDLE) 
;        )
;    )
;)
;
;(AverageLink (cog-new-av 1 1 1)
;    (ListLink
;        (VariableNode "$var_food") 
;    ) 
;
;    (add_rule (stv 1.0 1.0) GetFoodGoal 
;        (SequentialAndLink
;            random_rotate_left
;            multiple_step_forward
;        )
;        (AndLink 
;            (add_goal (PredicateNode "is_edible") (VariableNode "$var_food") ) 
;            (add_goal (PredicateNode "exist") (VariableNode "$var_food") )
;            (add_precondition (PredicateNode "left_of") (VariableNode "$var_food") PET_HANDLE) 
;        )
;    )
;)
;
;(AverageLink (cog-new-av 1 1 1)
;    (ListLink
;        (VariableNode "$var_food") 
;    ) 
;
;    (add_rule (stv 1.0 1.0) GetFoodGoal 
;        (SequentialAndLink
;            random_rotate_right
;            multiple_step_forward
;        )
;        (AndLink 
;            (add_goal (PredicateNode "is_edible") (VariableNode "$var_food") ) 
;            (add_goal (PredicateNode "exist") (VariableNode "$var_food") )
;            (add_precondition (PredicateNode "right_of") (VariableNode "$var_food") PET_HANDLE) 
;        )
;    )
;)
;
;(AverageLink (cog-new-av 1 1 1)
;    (ListLink
;        (VariableNode "$var_food") 
;    ) 
;
;    (add_rule (stv 1.0 1.0) GetFoodGoal 
;        multiple_step_forward
;        (AndLink 
;            (add_goal (PredicateNode "is_edible") (VariableNode "$var_food") ) 
;            (add_goal (PredicateNode "exist") (VariableNode "$var_food") )
;            (add_precondition (PredicateNode "in_front_of") (VariableNode "$var_food") PET_HANDLE) 
;        )
;    )
;)
;
;(AverageLink (cog-new-av 1 1 1)
;    (ListLink
;        (VariableNode "$var_food") 
;    ) 
;
;    (add_rule (stv 1.0 1.0) GetFoodGoal 
;        (SequentialAndLink
;            random_rotate_back
;            multiple_step_forward
;        )
;        (AndLink 
;            (add_goal (PredicateNode "is_edible") (VariableNode "$var_food") ) 
;            (add_goal (PredicateNode "exist") (VariableNode "$var_food") )
;            (add_precondition (PredicateNode "behind") (VariableNode "$var_food") PET_HANDLE) 
;        )
;    )
;)

;------------------------------------------------------------------------------

;(add_rule (stv 0.2 1.0) GetFoodGoal 
;    random_search
;    NULL_PRECONDITION
;)

;TODO: very simple rules only for testing dialog_system
(add_rule (stv 0.3 1.0) GetFoodGoal
    (add_action (SpeechActSchemaNode "AskForFood") )
    NULL_PRECONDITION
)

;(add_rule (stv 0.5 1.0) GetFoodGoal
;    (SequentialAndLink
;        (add_action (GroundedSchemaNode "lick") ) 
;    )    
;    NULL_PRECONDITION
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to WaterDemandGoal
;

(define GetWaterGoal
    (AndLink 
        (add_goal (PredicateNode "is_drinkable") (VariableNode "$var_water") ) 
;        (add_goal (PredicateNode "exist") (VariableNode "$var_water") )
;        (add_goal (PredicateNode "near") PET_HANDLE (VariableNode "$var_water") )
    )    
)

(AverageLink (cog-new-av 1 1 1)
    (ListLink
        (VariableNode "$var_water") 
    ) 

    (add_rule (stv 1.0 1.0) WaterDemandGoal
        (add_action (GroundedSchemaNode "drink_water") (VariableNode "$var_water") ) 
        GetWaterGoal
    )
)    

;(add_rule (stv 1.0 1.0) GetWaterGoal 
;    random_search
;    NULL_PRECONDITION
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to IntegrityDemandGoal
;
; TODO: if the pet/avatar in the virtual can really fight or get hurt, change 
;       the definition of IntegrityDemandGoal updater and the rules below. 
;

(add_rule (stv 1.0 1.0) IntegrityDemandGoal 
    (add_action (GroundedSchemaNode "go_home") )     
    NULL_PRECONDITION
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to AffiliationDemandGoal
;

(add_rule (stv 0.2 1.0) AffiliationDemandGoal 
    (add_action (GroundedSchemaNode "goto_obj") OWNER_HANDLE (NumberNode "2") )
    NULL_PRECONDITION
)

; This rule will please conversation partner by answering their questions
; The question could be TruthValueQuestion, WhatQuestion, WhyQuestion, even HowQuestion
(add_rule (stv 1.0 1.0) AffiliationDemandGoal
    (add_action (GroundedSchemaNode "scm:answer_question") )
    (add_goal (PredicateNode "has_unanswered_question") )
)

; This rule will please other agents by making a random statement,
; because it's boring if you keep silent for too long time. 
;(add_rule (stv 0.3 1.0) AffiliationDemandGoal
;    (add_action (GroundedSchemaNode "scm:unsolicited_observation") )
;    NULL_PRECONDITION
;)

; This rule will notify other agents important changes, 
(add_rule (stv 0.75 1.0) AffiliationDemandGoal
    (add_action (GroundedSchemaNode "scm:notify_changes") )
    (add_precondition (PredicateNode "has_dramatic_changes") )
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to CertaintyDemandGoal
;

;(define build_jump_line_ladder
;    (SequentialAndLink
;        (build_dark_block (NumberNode "1") ) ;build block in middle front
;        (add_action (GroundedSchemaNode "jump_forward") (NumberNode "1") ) ;jump on to the block 
;        (build_dark_block (NumberNode "0") ) ;build block in lower front
;    )        
;)
;
;(define build_jump_twisted_ladder
;    (SequentialAndLink
;        (build_dark_block (NumberNode "1") ) ;build block in middle front
;        (add_action (GroundedSchemaNode "jump_forward") (NumberNode "1") ) ;jump on to the block 
;        (OrLink
;            (add_action (GroundedSchemaNode "rotate") (NumberNode "90") )  
;            (add_action (GroundedSchemaNode "rotate") (NumberNode "-90") )  
;        )
;        (build_dark_block (NumberNode "0") ) ;build block in lower front
;    )        
;)
;
;(add_rule (cog-new-stv 0.0 1.0) CertaintyDemandGoal 
;    random_build_destroy_blocks
;;    random_search
;;    build_jump_twisted_ladder
;    NULL_PRECONDITION
;)

; This rule will increase the agent's certainty by asking questions to other agents. 
;(add_rule (cog-new-stv 1.0 1.0) CertaintyDemandGoal 
;    (add_action (GroundedSchemaNode "scm:ask_question") )
;    NULL_PRECONDITION
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to CompetenceDemandGoal
;

;(define random_simple_action
;    (OrLink
;        random_rotate
;        (add_action (GroundedSchemaNode "jump_forward") (NumberNode "1") ) 
;        (add_action (GroundedSchemaNode "step_forward") ) 
;        (add_action (GroundedSchemaNode "step_backward") ) 
;    )
;)
;
;(add_rule (cog-new-stv 0.65 1.0) CompetenceDemandGoal 
;    random_simple_action 
;    NULL_PRECONDITION 
;)
;
;(add_rule (cog-new-stv 0.35 1.0) CompetenceDemandGoal 
;    random_build_destroy_blocks
;    NULL_PRECONDITION 
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to Relations 
;
; Note: 1. Some Relation Rules hold NULL_ACTION, some are not. 
;       2. NULL_ACTION which means "Do nothing". 'PsiRelationUpdaterAgent' would retrieve all the 
;          Relation Rules with NULL_ACTION during its Initialization. It will inspect their preconditions 
;          within each 'Cognitive Cycle', and then update the truth value of corresponding Relations
;          (EvaluationLinks actually), if all the preconditions are satisfied. 
;       3. For those Relation Rules with none NULL_ACTION, it menas the pet should be active to take 
;          actions to reach the Relations it desires. 
;
; Why there are Relation rules with normal/ NULL_ACTION?
;
; Example 1: Some one attacks you, then the relation between you and the guy would probably be Enemy. 
; Example 2: When you want to borrow something from a stranger, the first step may be approach him/ her 
;            and say hello to him/her. So what? You are tring to reach 'familiar_with' relation between 
;            you both!
;  
; In example 1, you did nothing, then the relation happens. These relation rules taking NULL_ACTION
; would be handled by 'PsiRelationUpdaterAgent'
;
; In example 2, the relation is considered as a Goal, or a Precondition before you borrowing stuff
; from the guy. In order to reach the Goal, you should take some actions. 'PsiActionSelectionAgent' is 
; in charege of these relation rules. 
;
; [ By Zhenhua Cai, on 2011-02-23 ]
;

; curious_about
; It is the very basic relation before others, such as know, familiar_with etc.
; 'PsiRelationUpdaterAgent' will handle this directly 

; familiar_with
; familiar_with is the first step before knowing
; for instance sniffing something makes the pet familiar with it

;(AverageLink (stv 1.0 1.0)
;    (ListLink EntityVar)	    
;
;    (add_rule (cog-new-stv 1.0 1.0) FamiliarWithRelation NULL_ACTION
;        CuriousAboutRelation
;        (add_gpn "familiarWithPrecondition")
;    )
;)

; know
; know it one step above familiar with, for instance if the pet is familiar
; with something and then lick it is considered to be known 

;(AverageLink (stv 1.0 1.0)
;    (ListLink EntityVar)	    
;
;    (add_rule (cog-new-stv 1.0 1.0) KnowRelation NULL_ACTION
;        FamiliarWithRelation 
;        (add_gpn "knowPrecondition")	      
;    )
;)

; enemy

;(AverageLink (stv 1.0 1.0)
;    (ListLink EntityVar) 
;
;    (add_rule (cog-new-stv 1.0 1.0) EnemyRelation NULL_ACTION
;        (add_gpn "enemyPrecondition")	      
;    )
;)

; friend

;(AverageLink (stv 1.0 1.0)
;    (ListLink EntityVar)	    
;
;    (add_rule (cog-new-stv 1.0 1.0) FriendRelation NULL_ACTION
;        FamiliarWithRelation
;        (add_gpn "friendPrecondition")	      
;    )
;)

; anger

;(AverageLink (stv 1.0 1.0)
;    (ListLink EntityVar)	    
;
;    (add_rule (cog-new-stv 1.0 1.0) AngerRelation NULL_ACTION
;        (add_gpn "angerForFoodThreatenPrecondition")      
;    )
;)
;


;(AverageLink (stv 1.0 1.0)
;    (ListLink EntityVar)
;
;    (add_rule (cog-new-stv 1.0 1.0) AngerRelation NULL_ACTION
;        (add_gpn "angerForEnemyPrecondition")     
;    )
;)
;
;(AverageLink (stv 1.0 1.0)
;    (ListLink EntityVar)	    
;
;    (add_rule (cog-new-stv 1.0 1.0) AngerRelation NULL_ACTION
;        (add_gpn "angerWhenAttackedPrecondition")	      
;    )
;)


;begin adding facts for Einstein puzzle:
; Base concepts

(ConceptNode "pet" (stv 0.05 1))
(ConceptNode "drink" (stv 0.05 1))

; Pets
(EvaluationLink (stv 1 1)
   (PredicateNode "is_pet")
   (ListLink
      (ConceptNode "dogs")
      (ConceptNode "true")
   )
)


(EvaluationLink (stv 1 1)
   (PredicateNode "is_pet")
   (ListLink
      (ConceptNode "cats")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_pet")
   (ListLink
      (ConceptNode "fish")
      (ConceptNode "true")
   )
)

; Drinks
(EvaluationLink (stv 1 1)
   (PredicateNode "is_drink")
   (ListLink
      (ConceptNode "tea")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_drink")
   (ListLink
      (ConceptNode "water")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_drink")
   (ListLink
      (ConceptNode "milk")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "id_British_man")
      (AvatarNode "id_German_man")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "id_German_man")
      (AvatarNode "id_British_man")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "id_German_man")
      (AvatarNode "id_American_man")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "id_American_man")
      (AvatarNode "id_German_man")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "id_British_man")
      (AvatarNode "id_American_man")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "id_American_man")
      (AvatarNode "id_British_man")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "fish")
      (AvatarNode "dogs")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "dogs")
      (AvatarNode "fish")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "dogs")
      (AvatarNode "cats")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "cats")
      (AvatarNode "dogs")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "cats")
      (AvatarNode "fish")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "fish")
      (AvatarNode "cats")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "milk")
      (AvatarNode "water")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "water")
      (AvatarNode "milk")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "water")
      (AvatarNode "tea")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "tea")
      (AvatarNode "water")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "tea")
      (AvatarNode "milk")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_different")
   (ListLink
      (AvatarNode "milk")
      (AvatarNode "tea")
      (ConceptNode "true")
   )
)



