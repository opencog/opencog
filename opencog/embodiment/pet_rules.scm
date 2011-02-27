;
; @file embodiment/pet_rules.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-01-28
;
; Scheme scripts for adding Modulators, Demands and Rules into AtomSpace
;
; Note: Many of the openPsi Rules in this file are inspired by original Rules used in RuleEngine. 
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

(define SecuringThreshold 
    (add_modulator "SecuringThresholdModulator"  0.8)
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
    (add_demand_schema "EnergyDemand" 0.1)
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

; TODO: TestEnergy is only used for debugging. Remove it once finished. 
(define TestEnergyDemandSchema
    (add_demand_schema "TestEnergyDemand" 0.85)
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Goals
;
; Usage:
;     (add_goal goal_pred_name arg1 arg2 ...)
;
; Goal is represented as:
;
; EvaluationLink
;     (SimpleTruthValue indicates how well the demand is satisfied)
;     (ShortTermInportance indicates the urgency of the demand)
;     PredicateNode "goal_pred_name"
;     ListLink
;         Node:arguments
;         ...
;
; There are two kinds of Goals, Final Goal and Intermediate Goal.
;
; A Final Goal, also known as Demand Goal, is to keep a specific Demand in a suitable range, 
; Which is the starting point of backward chaining. 
;
; While an Intermediate Goal should be used as other Rule's Precondition. 
;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Final Goals, also known as Demand Goals
;

(define CurrentDemandGoal
    (add_goal "CurrentDemandGoal") 
)

(define PreviousDemandGoal
    (add_goal "PreviousDemandGoal") 
)

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

; TODO: TestEnergy is only used for debugging. Remove it once finished. 
(define TestEnergyDemandGoal
    (add_goal "TestEnergyDemandGoal")
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Relation Goals, which are usually served as other Rule's Precondition  
;
; Note: There's no need to set truth value and attention value for Nodes. 
;       If you insist in doing this, be carefule about the sequence of node name and truth value. 
;
;       For instance, 
;           (VariableNode (DEFAULT_STV) "$VarName") 
;       is fault. 
;       The correct version is
;           (VariableNode "$VarName" (DEFAULT_STV))
;

(define EntityVar
    (VariableNode "$EntityVar")
)

(define FriendRelation
    (add_goal "friend" "'self'" EntityVar) 
)

(define EnemyRelation
    (add_goal "enemey" "'self'" EntityVar)
)

(define GratitudeRelation
    (add_goal "gratitude" "'self'" EntityVar)
)

(define LoveRelation
    (add_goal "love" "'self'" EntityVar)
)

(define FearRelation
    (add_goal "fear" "'self'" EntityVar)
)

(define AngerRelation
    (add_goal "anger" "'self'" EntityVar) 
)

(define KnowRelation
    (add_goal "know" "'self'" EntityVar)
)

(define CuriousAboutRelation
    (add_goal "curious_about" "'self'" EntityVar)
)

(define FamiliarWithRelation
    (add_goal "familiar_with" "'self'" EntityVar)
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
    (connect_demand_goal EnergyDemandSchema         EnergyDemandGoal 0.1 1.0)
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

; TestEnergy is only used for debugging. Remove it once finished. 
(define ConnectTestEnergyDemandGoal
    (connect_demand_goal TestEnergyDemandSchema     TestEnergyDemandGoal 0.1 0.95)
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
;         AndLink
;             EvaluationLink
;                 GroundedPredicateNode "precondition_1_name"
;                 ListLink
;                     Node:arguments
;                     ...
;             EvaluationLink
;                 PredicateNode         "precondition_2_name"
;                 ListLink
;                     Node:arguments
;                     ...
;             ...
;                        
;         ExecutionLink
;             GroundedSchemaNode "schema_name"
;             ListLink
;                 Node:arguments
;                 ...
;
;     EvaluationLink
;         (SimpleTruthValue indicates how well the demand is satisfied)
;         (ShortTermInportance indicates the urgency of the demand)
;         PredicateNode: "goal_name" 
;         ListLink
;             Node:arguments
;             ...
;
; For each Rule, there's only a Goal, an Action and a bunch of Preconditions. 
; And all these Preconditions should be grouped in an AndLink.
; If you want to use OrLink, then just split the Rule into several Rules.
; For the efficiency and simplicity of the planer (backward chainging), NotLink is forbidden currently.  
;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules connect with TestEnergy
;
; TODO: TestEnergy is only used for debugging. Remove it once finished.
;

(define TestGetFoodGoal 
    (add_goal "TestGetFoodGoal")
)

(define test_eat_food
    (add_action "test_eat_food")
)

(add_rule (cog-new-stv 0.8 1.0) TestEnergyDemandGoal test_eat_food
          TestGetFoodGoal 
)

(define test_search_food
    (add_action "test_search_food") 
)

(add_rule (cog-new-stv 0.8 1.0) TestGetFoodGoal test_search_food
          (add_gpn_precondition "testSearchFoodPrecondition") 
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; Rules for Relations (Only used for debugging)
;

(define ObjBall
    (ObjectNode "Ball") 
)

(define ObjChicken
    (ObjectNode "Chicken")
)

(define ObjToy
    (ObjectNode "Toy") 
)

(define EvaNoveltyBall
    (EvaluationLink (stv 1.0 1.0)
        (PredicateNode "RelationNovelty")
        (ListLink
            PET_HANDLE
            ObjBall
        )
    )
)

(define EvaKnowBall
    (EvaluationLink 
        (PredicateNode "RelationKnow")
        (ListLink
            PET_HANDLE 
            ObjBall
        )
    )
)

;(ImplicationLink (stv 1.0 1.0)
;    EvaNoveltyBall
;    EvaKnowBall
;)

(define EvaNovelty
    (EvaluationLink 
        (PredicateNode "RelationNovelty")
        (ListLink
            PET_HANDLE
            EntityVar
        )
    ) 
)

(define EvaKnow
    (EvaluationLink
        (PredicateNode "RelationKnow")
        (ListLink
            PET_HANDLE
            EntityVar
        )
    )
)

(ForAllLink (stv 1.0 1.0)
    (ListLink EntityVar)

    (ImplicationLink 
        EvaNovelty
        EvaKnow
    )
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules to test the whole OpenPsi, can be used as a very simple demo
;

;(define wander_searching_food
;    (add_action "wander_searching_food")
;)
;
;(define goto_owner
;    (add_action "goto_owner")
;)
;
;(define random_step_searching
;    (add_action "random_step_searching")
;)
;
;(add_rule (cog-new-stv 0.8 1.0)  EnergyDemandGoal random_step_searching
;    (add_gpn_precondition "searchForFoodPrecondition")
;)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to EnergyDemandGoal
;

(define truePrecondition
    (add_gpn_precondition "truePrecondition")  
)

(define eat_food
    (add_action "eat_food")  
)

(define GetFoodGoal
    (add_goal "GetFoodGoal") 
)

(add_rule (cog-new-stv 1.0 1.0) EnergyDemandGoal eat_food
    GetFoodGoal   	  
)

(define search_for_food 
    (add_action "search_for_food")  
)

(add_rule (cog-new-stv 1.0 1.0) GetFoodGoal search_for_food
    truePrecondition	  
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to WaterDemandGoal
;

(define drink_water
    (add_action "drink_water")  
)

(define GetWaterGoal 
    (add_goal "GetWaterGoal")  
)

(add_rule (cog-new-stv 1.0 1.0) WaterDemandGoal drink_water
    GetWaterGoal	  
)

(define search_for_water
    (add_action "search_for_water")  
)

(add_rule (cog-new-stv 1.0 1.0) GetWaterGoal search_for_water
    truePrecondition	  
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to IntegrityDemandGoal
;

(define flee
    (add_action "flee")
)

(add_rule (cog-new-stv 1.0 1.0) IntegrityDemandGoal flee
    (add_gpn_precondition "fleePrecondition")
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Rules related to AffiliationDemandGoal
;

(define goto_owner
    (add_action "goto_owner")  
)

(add_rule (cog-new-stv 1.0 1.0) AffiliationDemandGoal goto_owner
    truePrecondition
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to CertaintyDemandGoal
;

(define play
    (add_action "play")  
)

(add_rule (cog-new-stv 1.0 1.0) CertaintyDemandGoal play
    truePrecondition	  
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules related to CompetenceDemandGoal
;

(add_rule (cog-new-stv 1.0 1.0) CompetenceDemandGoal play
    truePrecondition	  
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

(ForAllLink (stv 1.0 1.0)
    (ListLink EntityVar)	    

    (add_rule (cog-new-stv 1.0 1.0) FamiliarWithRelation NULL_ACTION
        CuriousAboutRelation
	(add_gpn_precondition "familiarWithPrecondition")
    )
)

; know
; know it one step above familiar with, for instance if the pet is familiar
; with something and then lick it is considered to be known 

(ForAllLink (stv 1.0 1.0)
    (ListLink EntityVar)	    

    (add_rule (cog-new-stv 1.0 1.0) KnowRelation NULL_ACTION
	FamiliarWithRelation 
        (add_gpn_precondition "knowPrecondition")	      
    )
)

; enemy

(ForAllLink (stv 1.0 1.0)
    (ListLink EntityVar) 

    (add_rule (cog-new-stv 1.0 1.0) EnemyRelation NULL_ACTION
        (add_gpn_precondition "enemyPrecondition")	      
    )
)

; friend

(ForAllLink (stv 1.0 1.0)
    (ListLink EntityVar)	    

    (add_rule (cog-new-stv 1.0 1.0) FriendRelation NULL_ACTION
	FamiliarWithRelation
        (add_gpn_precondition "friendPrecondition")	      
    )
)

; anger

(ForAllLink (stv 1.0 1.0)
    (ListLink EntityVar)	    

    (add_rule (cog-new-stv 1.0 1.0) AngerRelation NULL_ACTION
	(add_gpn_precondition "angerForFoodThreatenPrecondition")      
    )
)

(ForAllLink (stv 1.0 1.0)
    (ListLink EntityVar)

    (add_rule (cog-new-stv 1.0 1.0) AngerRelation NULL_ACTION
        (add_gpn_precondition "angerForEnemyPrecondition")     
    )
)

(ForAllLink (stv 1.0 1.0)
    (ListLink EntityVar)	    

    (add_rule (cog-new-stv 1.0 1.0) AnagerRelation NULL_ACTION
        (add_gpn_precondition "angerWhenAttackedPrecondition")	      
    )
)

