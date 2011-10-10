;
; @file embodiment/pet_rules.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-07-05
;
; Scheme scripts for adding Modulators, Demands and Rules into the AtomSpace.
;
; Many of the OpenPsi rules in this file are inspired from the original rules used by
; the old embodiment RuleEngine. 
;
; TODO: These Rules are very experimental and only for debugging. 
;       Once PsiActionSelectionAgent is finished, we would tune these Rules, observing Pet's behaviors.

;
; Initialize PET_HANDLE, OWNER_HANDLE and CURRENT_TIMESTAMP for Test 
; If you run the Multiverse Client, comment the lines below 
;

;(set! PET_HANDLE (PetNode "TestPet") )
;(set! OWNER_HANDLE (AvatarNode "TestAvatar") )      
;(set! CURRENT_TIMESTAMP 0)


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
;(if (null? CURRENT_TIMESTAMP)
;    (begin
;        (print_debug_info INFO_TYPE_FAIL "pet_rules.scm" 
;                          (string-append "CURRENT_TIMESTAMP is null. " 
;                                         "Please call (set! CURRENT_TIMESTAMP 'current_time_stamp') "
;                                         "firstly, in c++ code before actually loading any Rules!"
;                          )              
;        );print_debug_info
;
;        (exit -1)
;    );begin
;);if

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
;

(define ActivationModulator 
    (add_modulator "ActivationModulator" 0.6)
)

(define ResolutionModulator 
    (add_modulator "ResolutionModulator" 0.8)
)

(define SecuringThreshold 
    (add_modulator "SecuringThresholdModulator"  0.35)
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
; SimilarityLink (stv 1.0 1.0)
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
    (add_demand_schema "IntegrityDemand" 0.8)
)

(define AffiliationDemandSchema
    (add_demand_schema "AffiliationDemand" 0.75)
)

(define CertaintyDemandSchema
    (add_demand_schema "CertaintyDemand" 0.65)
)

(define CompetenceDemandSchema
    (add_demand_schema "CompetenceDemand" 0.65)
)

; TODO: TestEnergy is only used for debugging. Remove it once finished. 
(define TestEnergyDemandSchema
    (add_demand_schema "TestEnergyDemand" 0.85)
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

; TODO: TestEnergy is only used for debugging. Remove it once finished. 
(define TestEnergyDemandGoal
    (add_goal (PredicateNode "TestEnergyDemandGoal") )
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
;             SimilarityLink (stv 1.0 1.0)
;                 NumberNode: "demand_value"
;                 ExecutionOutputLink
;                     GroundedSchemaNode: "demand_schema_name"
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
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.8 1.0 IntegrityDemandSchema)
)

(connect_goal_updater   
     AffiliationDemandGoal 
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.4 1.0 AffiliationDemandSchema)
)

(connect_goal_updater
     CertaintyDemandGoal
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.6 1.0 CertaintyDemandSchema)
)

(connect_goal_updater
     CompetenceDemandGoal 
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.75 1.0 CompetenceDemandSchema) 
)

; TestEnergy is only used for debugging. Remove it once finished. 
(connect_goal_updater     
     TestEnergyDemandGoal 
     (add_goal (GroundedPredicateNode "fuzzy_within") 0.1 0.95 TestEnergyDemandSchema)
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
; Rules connect with TestEnergy
;
; TODO: TestEnergy is only used for debugging. Remove it once finished.
;

;(define TestGetFoodGoal 
;    (add_goal "TestGetFoodGoal")
;)
;
;(define test_eat_food
;    (add_action (GroundedSchemaNode "test_eat_food") )
;)
;
;(add_rule (cog-new-stv 0.8 1.0) TestEnergyDemandGoal
;          (add_action (GroundedSchemaNode "goto_owner") )
;          truePrecondition
;)
;
;(define test_search_food
;    (add_action (GroundedSchemaNode "test_search_food") )
;)
;
;(add_rule (cog-new-stv 0.8 1.0) TestGetFoodGoal test_search_food
;          (add_gpn "testSearchFoodPrecondition") 
;)

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
;(ForAllLink (stv 1.0 1.0)
;    (ListLink EntityVar)
;
;    (ImplicationLink 
;        EvaNovelty
;        EvaKnow
;    )
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules to test the whole OpenPsi, can be used as a very simple demo
;

;(define wander_searching_food
;    (add_action (GroundedSchemaNode "wander_searching_food") )
;)
;
;(define goto_owner
;    (add_action (GroundedSchemaNode "goto_owner") )
;)
;
;(define random_step_searching
;    (add_action (GroundedSchemaNode "random_step_searching") )
;)
;
;(add_rule (cog-new-stv 0.8 1.0)  EnergyDemandGoal random_step_searching
;    (add_gpn "searchForFoodPrecondition")
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to EnergyDemandGoal
;

(define GetFoodGoal
    (AndLink 
        (add_goal (PredicateNode "is_edible") (VariableNode "$var_food") ) 
        (add_goal (PredicateNode "exist") (VariableNode "$var_food") )
    )
)

(ForAllLink (cog-new-av 1 1 1)
    (ListLink
        (VariableNode "$var_food") 
    ) 

    (add_rule (stv 1.0 1.0) EnergyDemandGoal 
        (SequentialAndLink
            (add_action (GroundedSchemaNode "goto_obj") (VariableNode "$var_food") (NumberNode "2") )
            (add_action (GroundedSchemaNode "grab") (VariableNode "$var_food") )
            (add_action (GroundedSchemaNode "eat") (VariableNode "$var_food") )
        )    
        GetFoodGoal
    )
)    

;(add_rule (stv 1.0 1.0) GetFoodGoal 
;    (add_action (GroundedSchemaNode "random_search") )
;    NULL_PRECONDITION
;)

; TODO: very file rules only for testing dialog_system
(add_rule (stv 0.6 1.0) GetFoodGoal
    (add_action (SpeechActSchemaNode "AskForFood") )
    NULL_PRECONDITION
)

(add_rule (stv 0.3 1.0) GetFoodGoal
    (SequentialAndLink
        (add_action (GroundedSchemaNode "lick") ) 
    )    
    NULL_PRECONDITION
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to WaterDemandGoal
;

(define GetWaterGoal
    (AndLink 
        (add_goal (PredicateNode "is_drinkable") (VariableNode "$var_water") ) 
        (add_goal (PredicateNode "exist") (VariableNode "$var_water") )
;        (add_goal (PredicateNode "near") PET_HANDLE (VariableNode "$var_water") )
    )    
)

(ForAllLink (cog-new-av 1 1 1)
    (ListLink
        (VariableNode "$var_water") 
    ) 

    (add_rule (stv 1.0 1.0) WaterDemandGoal
        (add_action (GroundedSchemaNode "drink_water") (VariableNode "$var_water") ) 
        GetWaterGoal
    )
)    

(add_rule (stv 1.0 1.0) GetWaterGoal 
    (add_action (GroundedSchemaNode "random_search") )
    NULL_PRECONDITION
)

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

(add_rule (stv 1.0 1.0) AffiliationDemandGoal 
    (add_action (GroundedSchemaNode "goto_obj") OWNER_HANDLE (NumberNode "2") )
    NULL_PRECONDITION
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to CertaintyDemandGoal
;

(define random_build_destroy_blocks
    (SequentialAndLink
        (add_action (GroundedSchemaNode "step_backward") ) ; TODO: the direction is wrong 
        (add_action (GroundedSchemaNode "step_forward") )  ; TODO: the direction is wrong 
;        (add_action (GroundedSchemaNode "rotate_right") )  ; TODO: doesn't work 
        (add_action (GroundedSchemaNode "build_block_at") (WordNode "dummy_arg") ) 
;        (add_action (GroundedSchemaNode "build_block") (NumberNode "0") ) 
        (add_action (GroundedSchemaNode "jump_up") (NumberNode "1") ) ;TODO: jump to the block 
        (add_action (GroundedSchemaNode "destroy_block_at") ) 
    )        
)    

(add_rule (cog-new-stv 1.0 1.0) CertaintyDemandGoal 
;    (add_action (GroundedSchemaNode "random_search") ) ; TODO: Doesn't work, should be implemented in scheme
    random_build_destroy_blocks    
    NULL_PRECONDITION
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to CompetenceDemandGoal
;

(add_rule (cog-new-stv 0.25 1.0) CompetenceDemandGoal 
    (add_action (GroundedSchemaNode "random_simple_action") ) 
    NULL_PRECONDITION 
)

(add_rule (cog-new-stv 0.75 1.0) CompetenceDemandGoal 
    random_build_destroy_blocks    
    NULL_PRECONDITION 
)

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

;(ForAllLink (stv 1.0 1.0)
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

;(ForAllLink (stv 1.0 1.0)
;    (ListLink EntityVar)	    
;
;    (add_rule (cog-new-stv 1.0 1.0) KnowRelation NULL_ACTION
;        FamiliarWithRelation 
;        (add_gpn "knowPrecondition")	      
;    )
;)

; enemy

;(ForAllLink (stv 1.0 1.0)
;    (ListLink EntityVar) 
;
;    (add_rule (cog-new-stv 1.0 1.0) EnemyRelation NULL_ACTION
;        (add_gpn "enemyPrecondition")	      
;    )
;)

; friend

;(ForAllLink (stv 1.0 1.0)
;    (ListLink EntityVar)	    
;
;    (add_rule (cog-new-stv 1.0 1.0) FriendRelation NULL_ACTION
;        FamiliarWithRelation
;        (add_gpn "friendPrecondition")	      
;    )
;)

; anger

;(ForAllLink (stv 1.0 1.0)
;    (ListLink EntityVar)	    
;
;    (add_rule (cog-new-stv 1.0 1.0) AngerRelation NULL_ACTION
;        (add_gpn "angerForFoodThreatenPrecondition")      
;    )
;)
;
;(ForAllLink (stv 1.0 1.0)
;    (ListLink EntityVar)
;
;    (add_rule (cog-new-stv 1.0 1.0) AngerRelation NULL_ACTION
;        (add_gpn "angerForEnemyPrecondition")     
;    )
;)
;
;(ForAllLink (stv 1.0 1.0)
;    (ListLink EntityVar)	    
;
;    (add_rule (cog-new-stv 1.0 1.0) AngerRelation NULL_ACTION
;        (add_gpn "angerWhenAttackedPrecondition")	      
;    )
;)

