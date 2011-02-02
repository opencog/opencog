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

(set! PET_HANDLE (PetNode "TestPet") )
(set! OWNER_HANDLE (AvatarNode "TestAvatar") )      
(set! CURRENT_TIMESTAMP 0)


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

(define FriendRelationGoal
    (add_goal "FriendRelationGoal" "'self'" EntityVar) 
)

(define EnemyRelationGoal
    (add_goal "EnemyRelationGoal" "'self'" EntityVar)
)

(define GratitudeRelationGoal
    (add_goal "GratitudeRelationGoal" "'self'" EntityVar)
)

(define LoveRelationGoal
    (add_goal "LoveRelationGoal" "'self'" EntityVar)
)

(define FearRelationGoal
    (add_goal "FearRelationGoal" "'self'" EntityVar)
)

(define AngerRelationGoal
    (add_goal "AngerRelationGoal" "'self'" EntityVar) 
)

(define KnowRelationGoal
    (add_goal "KnowRelationGoal" "'self'" EntityVar)
)

(define CuriousAboutRelationGoal
    (add_goal "CuriousAboutRelationGoal" "'self'" EntityVar)
)

(define FamiliarWithRelationGoal
    (add_goal "FamiliarWithRelationGoal" "'self'" EntityVar)
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
; Below are the original Rules used by RuleEngine that are not covered 
; by current OpenPsi Rules.
;
; They might be useful in near future. 
;
; By Zhenhua Cai, on 2011-01-07
;

;---------------------- RECOMMENDATIONS---------------
;-- 1) The rules format is as follows:
;--    rule ( rule name
;--			 rule type
;--			 rule strength
;--			 rule precondition
;--			 rule effect ) 
;-- 
;-- 2) Currently there are three rule types: 
;--    * SCHEMA_RULE - whose effect is the execution of a combo schema
;--	  * FEELING_RULE - whose effect is the change of a agent feeling
;--	  * RELATION_RULE - whose effect is the change in the agent relation 
;-- 		with something or someone
;--
;-- 3) Strength makes sense only for SCHEMA_RULES, so when creating rules
;--	  from other types just set strength to 1.0 meaning that, if actived,
;--	  the rule effect must be executed
;--
;-- 4) Rule precondition should be a combo script coded in 
;--	  RulesPreconditions.combo file. These preconditions should be named
;--	  with the rule name adding the "Precondition" suffix
;--
;-----------------------------------------------------
;
;
;---------------------- ACTIONS ----------------------
;
;--- BEG ---
;
;rule( "begCompany", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "begCompanyPrecondition",
;      "beg_to_owner()" );
;
;rule( "begMercy", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "begMercyPrecondition",
;      "beg_to_target('_*_')" );
;
;--- EAT ---
;
;rule( "eatAlone", SCHEMA_RULE, { PLAYING_MODE=0.5, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "eatAlonePrecondition",
;      "drop_n_eat('_*_')" );
;
;--- DRINK ---
;
;rule( "drinkAlone", SCHEMA_RULE, { PLAYING_MODE=0.5, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "drinkAlonePrecondition",
;      "drop_n_drink('_*_')" );
;
;--- PEE ---
;
;rule( "peeToMarkTerritory", SCHEMA_RULE, { PLAYING_MODE=0.1, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "peeToMarkTerritoryPrecondition",
;      "drop_n_pee('_*_')" );
;
;--- APPROACH ---
;
;rule( "hideWhenUnknownAvatarIsComming", SCHEMA_RULE, { PLAYING_MODE=0.6, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "hideWhenUnknownAvatarIsCommingPrecondition",
;      "goto_owner()" )
;
;rule( "approachToDistractOwner", SCHEMA_RULE, { PLAYING_MODE=0.71, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "approachToDistractOwnerPrecondition",
;      "goto_owner()" );
;
;--- BARK ---
;
;rule( "barkToFrighten", SCHEMA_RULE, { PLAYING_MODE=0.21, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "barkToFrightenPrecondition",
;      "bark_to_target('_*_')" );
;
;rule( "barkFear", SCHEMA_RULE, { PLAYING_MODE=0.5, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "barkFearPrecondition",
;      "bark()" );
;
;rule( "barkWhenPlay", SCHEMA_RULE, { PLAYING_MODE=0.2, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "barkWhenPlayPrecondition",
;      "bark()" );
;
;rule( "barkAfterDetectANewObject", SCHEMA_RULE, { PLAYING_MODE=0.3, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "barkAfterDetectANewObjectPrecondition",
;      "bark()" );
;
;--- LICK ---
;
;rule( "lickForGratitude", SCHEMA_RULE, { PLAYING_MODE=0.1, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "lickForGratitudePrecondition",
;      "drop_n_lick('_*_')" );
;
;--- Nil : Warning the weight of lickToInspect must be higher than
;--- the weight of sniffToInspect, otherwise there may be the risk that
;--- the pet would sniff forever (at least when no
;--- random noise is introduced in the rule selection process)
;
;rule( "lickForLove", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "lickForLovePrecondition",
;      "drop_n_lick('_*_')" )
;
;--- SNIFF ---
;
;--- Nil : Warning the weight of sniffToInspect must be lower than
;--- the weight of lickToInspect, otherwise there may be the risk that
;--- the pet would sniff forever (at least when no
;--- random noise is introduced in the rule selection process)
;
;rule( "sniffJealousy",  SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "sniffJealousyPrecondition",
;      "drop_n_sniff('_*_')" )
;
;--- JUMP ---
;
;rule( "jumpJoy",  SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "jumpJoyPrecondition",
;      "jump_up()" )
;
;rule( "jumpPlay", SCHEMA_RULE, { PLAYING_MODE=0.2, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "jumpPlayPrecondition",
;      "goto_back_flip('_*_')" )
;
;
;--- PLAY ---
;
;rule( "playWithOwner", SCHEMA_RULE, { PLAYING_MODE=0.15, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "playWithOwnerPrecondition",
;      "play()" )
;
;rule( "playWhenAvatarBecomeFriend", SCHEMA_RULE, { PLAYING_MODE=0.6, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "playWhenAvatarBecomeFriendPrecondition",
;      "play()" )
;
;--- BRING ---
;
;rule( "bringToPetFriend", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "bringRandomObjectToPetFriendPrecondition",
;      "bringRandomObject('_*_')" );
;
;--rule( "bringToOwner", SCHEMA_RULE, { PLAYING_MODE=0.1, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "bringRandomObjectToOwnerPrecondition",
;--      "bringRandomObject('owner')" );
;
;--- WANDER ---
;
;rule( "lookForNewThings", SCHEMA_RULE, { PLAYING_MODE=0.15, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "lookForNewThingsPrecondition",
;      "goto_random_pickupable()" );
;
;-- if the goto or gonear target is a moving object, update the target position if necessary
;-- Mapinfo delays is causing errors on replanning path to target, so it will be commented until
;-- that problem is not resolved
;--rule( "keepMovingToTarget", SCHEMA_RULE, { PLAYING_MODE=0.781, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "keepMovingToTargetPrecondition",
;--      "keepMoving()" );
;
;--- CHASE MOVING OBJECT --
;
;rule( "chasePetHoldingSomething", SCHEMA_RULE, { PLAYING_MODE=0.75, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "chasePetHoldingSomethingPrecondition",
;      "chasePetHoldingSomething('_*_')" );
;
;--- FOLLOW_OWNER ---
;
;--- the rule below has been commented
;--- for the social behavior video shot
;
;---------------------- FEELINGS ----------------------
; 
;rule( "feelHate", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "feelHatePrecondition",
;      "feel('hate', 0.7)" );
;
;rule( "feelAngerNaturally", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelAngerNaturallyPrecondition",
;      "feel( 'anger', 0.8 )" );
;
;rule( "feelFearWhenUnknownAvatarIsComming", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelFearWhenUnknownAvatarIsCommingPrecondition",
;      "feel( 'fear', 0.7 )" )  
;
;rule( "feelFearWhenAttacked", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelFearWhenAttackedPrecondition",
;      "feel( 'fear', 0.7 )" )
;
;rule( "feelFearNaturally", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelFearNaturallyPrecondition",
;      "feel( 'fear', 0.8 )" );
;
;rule( "feelPride", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelPridePrecondition",
;      "feel( 'pride', 0.7 )" );
;
;rule( "feelExcitement",  FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelExcitementPrecondition",
;      "feel('excitement', 0.7)" );
;
;rule( "feelAngerWhenBored", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelAngerWhenBoredPrecondition",
;      "feel('anger', 0.7)" );
;
;rule( "feelAngerWhenAttacked", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelAngerWhenAttackedPrecondition",
;      "feel('anger', 0.7)" );
;
;rule( "feelComfortableAfterBark", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelComfortableAfterBarkPrecondition",
;      "feel('happiness', 0.7 )" );
;
;rule( "feelHappinessWhenNearFriend", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelHappinessWhenNearFriendPrecondition",
;      "feel( 'happiness', 0.7 )" );
;
;--rule( "feelHappinessWhenBelongsToAGroup", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;--      "feelHappinessWhenBelongsToAGroupPrecondition",
;--      "feel( 'happiness', 0.7 )" );
;
;rule( "feelGratitudeWhenReceiveFood", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelGratitudeWhenReceiveFoodPrecondition",
;      "feel('gratitude', 0.7)" );
;
;rule( "feelHappiness", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelHappinessPrecondition",
;      "feel('happiness', 0.7)" );
;
;rule( "feelExcitementAfterEat", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelExcitementAfterEatPrecondition",
;      "feel('excitement', 0.7)" );
;
;rule( "feelExcitementeWhenNovelty", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelExcitementeWhenNoveltyPrecondition",
;      "feel('excitement', 0.8)" );
;
;rule( "feelExcitementWhenBringObjectToOwner", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelExcitementWhenBringObjectToOwnerPrecondition",
;      "feel( 'excitement', 0.6 )" )
;
;rule( "feelLove", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelLovePrecondition",
;      "feel('love', 0.7)" );
;
;rule( "feelAnger", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelAngerPrecondition",
;      "feel('anger', 0.7)" );
;
;rule( "feelGratitude", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelGratitudePrecondition",
;      "feel('gratitude', 0.7 )" );
;
;rule( "feelFear", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
;      "feelFearPrecondition",
;      "feel('fear', 0.7)" );
;
;------ GROUP ACTIONS -------
;--rule( "goBehindPetToInspectIt",  SCHEMA_RULE, { PLAYING_MODE=0.22, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "goBehindPetToInspectItPrecondition",
;--      "gobehind_obj('_*_' )" );
;
;--rule( "goBehindFriendlyPetWhenNear", SCHEMA_RULE, { PLAYING_MODE=0.22, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "goBehindFriendlyPetWhenNearPrecondition",
;--      "gobehind_obj('_*_' )" );
;
;--rule( "requestGrouping", SCHEMA_RULE, { PLAYING_MODE=0.23, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "requestGroupingPrecondition",
;--      "group_command('request_grouping', '_*_' )" );
;
;--rule( "joinOwnGroup", SCHEMA_RULE, { PLAYING_MODE=0.23, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "joinOwnGroupPrecondition",
;--      "group_command('join_group', self )" );
;
;--rule( "reJoinGroup", SCHEMA_RULE, { PLAYING_MODE=0.24, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "reJoinGroupPrecondition",
;--      "group_command('join_group', getGroupLeaderId( ) )" );
;
;--rule( "joinGroup", SCHEMA_RULE, { PLAYING_MODE=0.23, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "joinGroupPrecondition",
;--      "group_command('join_group', '_*_' )" );
;
;--rule( "followGroupLeader", SCHEMA_RULE, { PLAYING_MODE=0.23, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "followGroupLeaderPrecondition",
;--      "gonear_obj( getGroupLeaderId( ) )" );
;
;--rule( "abandonGroup", SCHEMA_RULE, { PLAYING_MODE=0.24, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "abandonGroupPrecondition",
;--      "group_command( 'abandon_group', self )" );
;
;--rule( "executeGroupLeaderCommand", SCHEMA_RULE, { PLAYING_MODE=0.25, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;--      "executeGroupLeaderCommandPrecondition",
;--      "group_command( 'execute_leader_command', '_*_' )" );
;
;------ DEFAULT ACTION ------
;
;rule( "defaultAction", SCHEMA_RULE, { PLAYING_MODE=0.001, SCAVENGER_HUNT_MODE=0.001, LEARNING_MODE=0.001 }, 
;      "defaultActionPrecondition",
;      "sit()" );
;
;rule( "defaultAggressiveAction", SCHEMA_RULE, { PLAYING_MODE=0.001, SCAVENGER_HUNT_MODE=0.001, LEARNING_MODE=0.001 },
;      "defaultAggressiveActionPrecondition",
;      "bark()" );
;
;--- If you want the pet executes learned tricks spontaneously, set the weight of this rule to a non-null value (e.g., 0.27)
;rule( "selectLearnedTrick", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "selectLearnedTrickPrecondition",
;      "runRandomTopFiveLearnedTrick()" );
;
;----------------------------------------
;------ PET-TO-PET SOCIAL BEHAVIOR ------
;----------------------------------------
;
;--------------- ACTIONS ----------------
;
;
;---- APPROACH ----
;
;rule( "approachCuriousPet", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "approachCuriousPetPrecondition",
;      "goto_sniff('_*_')" );
;
;rule ( "goAwayFearfulPet", SCHEMA_RULE, { PLAYING_MODE=0.61, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;       "goAwayFearfulPetPrecondition",
;       "go_away_from('_*_')" );
;
;rule ( "approachAngryPet", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;       "approachAngryPetPrecondition",
;       "goto_growl_or_bite('_*_')" );
;
;---- REACTION ----
;
;rule ( "beenGrowledPet", SCHEMA_RULE, { PLAYING_MODE=0.65, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;       "beenGrowledPetPrecondition",
;       "react_from_growl('_*_')" );
;
;rule ( "beenBittenPet", SCHEMA_RULE, { PLAYING_MODE=0.9, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;       "beenBittenPetPrecondition",
;       "react_from_bite('_*_')" );
;
;rule ( "beenSniffedPet", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;       "beenSniffedPetPrecondition",
;       "react_from_sniff('_*_')" );
;
;---- if near friend and friend has been bitten then bark at the bitter ----
;
;rule ( "defendFriendFromBite",
;       SCHEMA_RULE,
;       { PLAYING_MODE=0.8, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;       "defendFriendFromBitePrecondition",
;       "goto_bark_to_target('_*_')" );
;
;--------------- FEELINGS ----------------
;
;rule ( "feelLessAngryAfterGotoGrowlOrBite", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;       "feelLessAngryAfterGotoGrowlOrBitePrecondition",
;       "feel( 'anger', 0.4 )" );
;
;
;--------------- FOR TESTING INDIVIDUAL PREDICATE ------------
;
;------ SCAVENGER HUNTER ACTIONS ------
;--rule( "approachOwnerToReceiveInstructions", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.87, LEARNING_MODE=0 },
;--      "approachOwnerToReceiveInstructionsPrecondition",
;--      "gonear_obj('_*_')" );
;--
;--rule( "payAttentionOnOwner", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.4, LEARNING_MODE=0 },
;--      "payAttentionOnOwnerPrecondition",
;--      "pay_attention('_*_')" );
;--
;--rule( "goHideAtHome", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.5, LEARNING_MODE=0 },
;--      "goHideAtHomePrecondition",
;--      "goto_pet_home()" );
;--
;--rule( "stayHindingAtHome", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.6, LEARNING_MODE=0 },
;--      "stayHindingAtHomePrecondition",
;--      "pay_attention('_*_')" );
;--
;--rule( "exploreObject", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.6, LEARNING_MODE=0 },
;--      "exploreObjectPrecondition",
;--      "walkto_obj(custom_path)" );
;--
;--rule( "exploreHiddenArea", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.6, LEARNING_MODE=0 },
;--      "exploreHiddenAreaPrecondition",
;--      "walkto_obj(custom_path)" );
;--
;--rule( "gotoTreasure", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.99, LEARNING_MODE=0 },
;--      "gotoTreasurePrecondition",
;--      "walkto_obj(custom_object)" );
;--
;--rule( "pickupTreasure", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.99, LEARNING_MODE=0 },
;--      "pickupTreasurePrecondition",
;--      "grab_nearest_object()" );
;--
;--rule( "goBackToOwner", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.99, LEARNING_MODE=0 },
;--      "goBackToOwnerPrecondition",
;--      "goto_owner()" );
;--
;--rule( "celebrateVictory", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.99, LEARNING_MODE=0 },
;--      "celebrateVictoryPrecondition",
;--      "drop_n_celebrate()" );
;--
;--rule( "cryDefeat", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.99, LEARNING_MODE=0 },
;--      "cryDefeatPrecondition",
;--      "beg()" );
;
