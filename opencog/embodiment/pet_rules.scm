;
; @file embodiment/pet_rules.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-01-07
;
; Scheme scripts for adding Modulators, Demands and Rules into AtomSpace
;
; Note: Many of the openPsi Rules in this file are inspired by original Rules used in RuleEngine. 
;
; TODO: These Rules are very experimental and only for debugging. 
;       Once PsiActionSelectionAgent is finished, we would tune these Rules, observing Pet's behaviors.
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Relation Goals, which are usually served as other Rule's Precondition 
;

(define EntityVar
    (VariableNode (DEFAULT_STV) (DEFAULT_AV) 
                  "$EntityVar" 
    )
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
; Rules for Relation Goals
;

; FamiliarWith is the first step before knowing. 
; For instance sniffing something makes the pet familiar with it
;

(define drop_n_sniff
    (add_action "drop_n_sniff" EntityVar)
)

(define gobehind_n_sniff_butt
    (add_action "gobehind_n_sniff_butt" EntityVar) 
)

(define goto_sniff
    (add_action "goto_sniff" EntityVar)
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.9 1.0) FamiliarWithRelationGoal drop_n_sniff
              CuriousAboutRelationGoal
    )
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.85 1.0) FamiliarWithRelationGoal gobehind_n_sniff_butt
              CuriousAboutRelationGoal
    )
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.8 1.0) FamiliarWithRelationGoal goto_sniff
              CuriousAboutRelationGoal
    )
)

; Know it one step above familiar with, for instance if the pet is 
; familiar with something and then lick it is consireded to be known
;

(define drop_n_lick
    (add_action "drop_n_lick" EntityVar)
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.8 1.0) KnowRelationGoal drop_n_lick
              FamiliarWithRelationGoal
    )
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.9 1.0) KnowRelationGoal (NULL_ACTION)
              FamiliarWithRelationGoal
              (add_gpn_precondition "relateKnowPrecondition" EntityVar)
    )
)

; Enemy
;

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.6 1.0) EnemyRelationGoal (NULL_ACTION)
              (add_gpn_precondition "relateEnemyWhenSeekedPrecondition" EntityVar)
    )
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.75 1.0) EnemyRelationGoal (NULL_ACTION)
              (add_gpn_precondition "relateEnemyWhenAttackedPrecondition" EntityVar)
    )
)


; CuriousAbout
;

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.85 1.0) CuriousAboutRelationGoal (NULL_ACTION)
              (add_gpn_precondition "relateCuriousAboutPrecondition" EntityVar)
    )
)


; Friend
;

(define is_pet_or_avatar
    (add_gpn_precondition "is_pet_or_avatar" EntityVar)
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.6 1.0) FriendRelationGoal drop_n_lick
              CuriousAboutRelationGoal
              is_pet_or_avatar
    )
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.85 1.0) FriendRelationGoal (NULL_ACTION)
              (add_gpn_precondition "relateFriendWhenReceiveFoodPrecondition" EntityVar)
    )
)

; Anger
;

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.9 1.0) AngerRelationGoal (NULL_ACTION)
              (add_gpn_precondition "relateAngerPrecondition" EntityVar)
    )
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.7 1.0) AngerRelationGoal (NULL_ACTION)
              EnemyRelationGoal
              (add_gpn_precondition "relateAngerEnemyPrecondition" EntityVar)
    )
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.9 1.0) AngerRelationGoal (NULL_ACTION)
              (add_gpn_precondition "relateAngerFoodThreatenPrecondition" EntityVar)
    )
)

; Gratitude
;
; TBD

; Love
;
; TBD

; Fear
;
; TBD

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules that connect with EnergyDemandGoal 
;

; Food
;

(define FoodVar
    (VariableNode (DEFAULT_STV) (DEFAULT_AV) 
                  "$FoodVar" 
    )
)


(define GetFoodGoal
    (add_goal "GetFoodGoal" FoodVar)
)


(ForAllLink
    (ListLink FoodVar)

    (add_rule (cog-new-stv 1.0 1.0) GetFoodGoal (NULL_ACTION)
              (add_gpn_precondition) "getFoodPrecondition" FoodVar)
    )
)

(define beg_to_owner
    (add_action "beg_to_owner")
)

(ForAllLink
    (ListLink FoodVar)

    (add_rule (cog-new-stv 0.8 1.0) GetFoodGoal beg_to_owner
              (add_gpn_precondition "begFoodPrecondition")
    )
)

(define wander_searching_food
    (add_action "wander_searching_food")
)

(ForAllLink
    (ListLink FoodVar)

    (add_rule (cog-new-stv 0.85 1.0) GetFoodGoal wander_searching_food
              (add_gpn_precondition "searchForFoodPrecondition")
    )
)

(define pay_attention
    (add_action "pay_attention" EntityVar)
)

(ForAllLink
    (ListLink FoodVar EntityVar)

    (add_rule (cog-new-stv 0.8 1.0) GetFoodGoal pay_attention
              FriendRelationGoal
              (add_gpn_precondition "sitToGetFoodPrecondition" EntityVar)
    )
)

(define bark_to_target
    (add_action "bark_to_target" EntityVar)
)

(ForAllLink
    (ListLink FoodVar EntityVar)

    (add_rule (cog-new-stv 0.9 1.0) GetFoodGoal bark_to_target
              (add_gpn_precondition "barkToProtectFoodPrecondition" EntityVar)
    )
)

(define drop_n_bite
    (add_action "drop_n_bite" EntityVar)
)

(ForAllLink
    (ListLink FoodVar EntityVar)

    (add_rule (cog-new-stv 0.8 1.0) GetFoodGoal drop_n_bite
              (add_gpn_precondition "biteToProtectFoodPrecondition" EntityVar)
    )
)

(define drop_n_eat
    (add_action "drop_n_eat" FoodVar)
)

(ForAllLink
    (ListLink FoodVar)

    (add_rule (cog-new-stv 0.9 1.0) EnergyDemandGoal drop_n_eat
              GetFoodGoal
    )              
)

; Nap
;
; TBD

;rule( "takeANapNearFriend", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "takeANapNearFriendPrecondition",
;      "sleep()" );
;
;rule( "takeANapAtHome",  SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "takeANapAtHomePrecondition",
;      "sleep()" );
;
;rule( "veryTiredNap", SCHEMA_RULE, { PLAYING_MODE=0.91, SCAVENGER_HUNT_MODE=0.91, LEARNING_MODE=0.91 },
;      "veryTiredNapPrecondition",
;      "sleep()" );
;
;rule( "napBeforeStarvation", SCHEMA_RULE, { PLAYING_MODE=0.92, SCAVENGER_HUNT_MODE=0.92, LEARNING_MODE=0.92 },
;      "napBeforeStarvationPrecondition",
;      "sleep()" );
;
;rule( "sitToRest", SCHEMA_RULE, { PLAYING_MODE=0.2, SCAVENGER_HUNT_MODE=0.2, LEARNING_MODE=0.2 },
;      "sitToRestPrecondition",
;      "sit()" );

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules that connect with WaterDemandGoal 
;

(define WaterVar
    (VariableNode (DEFAULT_STV) (DEFAULT_AV) 
                  "$WaterVar" 
    )
)

(define GetWaterGoal
    (add_goal "GetWaterGoal" WaterVar)
)

(define wander_searching_water
    (add_action "wander_searching_water")
)

(ForAllLink
    (ListLink WaterVar)

    (add_rule (cog-new-stv 0.9 1.0) GetWaterGoal wander_searching_water
              (add_gpn_precondition "searchForWaterPrecondition")
    )
)

(define drop_n_drink
    (add_action "drop_n_drink" WaterVar)
)

(ForAllLink
    (ListLink WaterVar)

    (add_rule (cog-new-stv 0.9 1.0) WaterDemandGoal drop_n_drink
              GetWaterGoal
    )              
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules that connect with IntegrityDemandGoal 
;

; Poo
;
; TBD

;rule( "wanderToFindPooPlace", SCHEMA_RULE, { PLAYING_MODE=0.88, SCAVENGER_HUNT_MODE=0.88, LEARNING_MODE=0.88 },
;      "wanderToFindPooPlacePrecondition",
;      "wander_searching_poo_place()" );
;
;rule( "poo", SCHEMA_RULE, { PLAYING_MODE=0.9, SCAVENGER_HUNT_MODE=0.9, LEARNING_MODE=0.9 },
;      "pooPrecondition",
;      "drop_n_poo('_*_')" );

; Pee
;
; TBD

;rule( "pee", SCHEMA_RULE, { PLAYING_MODE=0.9, SCAVENGER_HUNT_MODE=0.9, LEARNING_MODE=0.9 },
;      "peePrecondition",
;      "drop_n_pee('_*_')" );
;
;rule( "wanderToFindPeePlace", SCHEMA_RULE, { PLAYING_MODE=0.87, SCAVENGER_HUNT_MODE=0.5, LEARNING_MODE=0.5 },
;      "wanderToFindPeePlacePrecondition",
;      "wander_searching_pee_place()" );

; Bite
;

;rule( "biteEnemy", SCHEMA_RULE, { PLAYING_MODE=0.5, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "biteEnemyPrecondition",
;      "drop_n_bite('_*_')" );
;
;rule( "biteWhenAttackedButCalm", SCHEMA_RULE, { PLAYING_MODE=0.2, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "biteWhenAttackedButCalmPrecondition",
;      "drop_n_bite('_*_')" );
;
;rule( "biteWhenAttacked", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "biteWhenAttackedPrecondition",
;      "drop_n_bite('_*_')" );
;
;rule( "biteEnemyWhenAngry", SCHEMA_RULE, { PLAYING_MODE=0.6, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "biteEnemyWhenAngryPrecondition",
;      "drop_n_bite( '_*_' )" );

; Flee
;

(define goto_owner
    (add_action "goto_owner")
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.9 1.0) IntegrityDemandGoal goto_owner
              (add_gpn_precondition "fleeWhenAttackedPrecondition" EntityVar)
    )
)

(define wander
    (add_action "wander")
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.85 1.0) IntegrityDemandGoal wander
              (add_gpn_precondition "fleeWhenChasedByAMovingObjectPrecondition" EntityVar) 
    )
)

;rule( "lookForNearThingsWhenFearful", SCHEMA_RULE, { PLAYING_MODE=0.16, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "lookForNearThingsWhenFearfulPrecondition",
;      "goto_nearest_and_grabit()" );

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules that connect with AffiliationDemandGoal
;

(define walkto_obj
    (add_action "walkto_obj" EntityVar)
)

(ForAllLink
   (ListLink EntityVar)

   (add_rule (cog-new-stv 0.55 1.0) AffiliationDemandGoal walkto_obj
             FriendRelationGoal
   )
)

(define beg_to_owner
    (add_action "beg_to_owner")
)

(define gonear_obj
    (add_action "gonear_obj" EntityVar)
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.65 1.0) AffiliationDemandGoal beg_to_owner 
              (add_gpn_precondition "lookForCompanyPrecondition" EntityVar)
    )
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.7 1.0) AffiliationDemandGoal gonear_obj 
              (add_gpn_precondition "lookForCompanyPrecondition" EntityVar)
    )
)

(define goto_owner
    (add_action "goto_owner")
)

(add_rule (cog-new-stv 0.8 1.0) AffiliationDemandGoal goto_owner
          (add_gpn_precondition "followOwnerPrecondition")
)    

(define play
    (add_action "play")
)

(ForAllLink
    (ListLink EntityVar)
    (add_rule (cog-new-stv 0.7 1.0) AffiliationDemandGoal play
              (add_gpn_precondition "playPrecondition" EntityVar)
    )
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules that connect with CertaintyDemandGoal
;

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.9 1.0) CertaintyDemandGoal walkto_obj
              (add_gpn_precondition "approachUnknownAvatarToInspectPrecondition" EntityVar)
              CuriousAboutRelationGoal
    )
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.9 1.0) CertaintyDemandGoal walkto_obj
              (add_gpn_precondition "approachToInspectPrecondition" EntityVar)
              CuriousAboutRelationGoal
    )
)

(define drop_n_lick
    (add_action "drop_n_lick" EntityVar)
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.8 1.0) CertaintyDemandGoal drop_n_lick
              CuriousAboutRelationGoal
              FamiliarWithRelationGoal
              (add_gpn_precondition "lickToInspectPrecondition" EntityVar)
    )
)

(define drop_n_sniff
    (add_action "drop_n_sniff" EntityVar)
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.8 1.0) CertaintyDemandGoal drop_n_sniff
              CuriousAboutRelationGoal
              (add_gpn_precondition "sniffToInspectPrecondition" EntityVar)
    )
)

;rule( "moveObjectToPlay", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "moveObjectToPlayPrecondition",
;      "moveObjectToPlay()" );
;
;rule( "chaseMovingObject", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
;      "chaseMovingObjectPrecondition",
;      "trotto_obj('_*_')" );

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules that connect with CompetenceDemandGoal
;

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.6 1.0) CompetenceDemanGoal walkto_obj
              (add_gpn_precondition "approachToAttackPrecondition" EntityVar)
    )
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.75 1.0) CompetenceDemanGoal play
              (add_gpn_precondition "playPrecondition" EntityVar)
    )
)

(define goto_random_pickupable
    (add_action "goto_random_pickupable")
)

(ForAllLink
    (ListLink EntityVar)

    (add_rule (cog-new-stv 0.85 1.0) CompetenceDemand goto_random_pickupable
              (add_gpn_precondition "lookForThingsToDestroy")
    )
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Rules that connect with Learning
;
; Learning new tricks can link to Final Goals (in another word Demand Goals) directly, 
; such as CompetenceDemanGoal.
;
; It can also connect to Final Goals indirectly. For example, it can link to SatisfyOwnerGoal firstly, 
; which can be connected to AffiliationDemandGoal finally. 
;
; Below are some ideas form Nil, which I would think over and implement latetr. 
;
; We could have a bunch of rules and cognitive schematics such as
; 
; 1.a) LearningSessionStart & nearby_owner => DoneShowing
; 
;     That rule tells that if the pet tends to be nearby the owner during a
;     learning session, it will eventually be done showing an exemplar.
;     Otherwise if the owner sees the pet running away it'll give up the
;     learning session. Of course here => are predictiveImplications 
; 
; 1.b) FarFromOwner & go_nearby(owner) => nearby_owner
; 
; 2) DoneShowing & LaunchImitationLearningProcedure => GotAGoodCandidateSubGoal
; 
;     LaunchImitationLearningProcedure is a schema that runs the C++ stuff
;     to send to the learning server the episodic memory with the exemplar
;     to imitate. The TV of the PredictiveImplication may be more uncertain
;     (could be initialized a certain way and then updated based on
;     experience).
; 
; 3) GotAGoodCandidateGoal & TryLastBestProcedure => satisfying_owner_goal
; 
;     DoneShowing and GotAGoodCandidateSubGoal should naturally be inferred
;     to belong to the goal pool and got some attention.
; 

(define SatisfyOwnerGoal
    (add_goal "SatisfyOwnerGoal")
)

(add_rule (cog-new-stv 0.95 1.0) AffiliationDemandGoal (NULL_ACTION)
          SatisfyOwnerGoal
)

(define gonear_exemplar_avatar
    (add_action "gonear_exemplar_avatar")
)

(add_rule (cog-new-stv 0.9 1.0) SatisfyOwnerGoal gonear_exemplar_avatar
          (add_gpn_precondition "approachToLearnPrecondition")
)

(define pay_attention_exemplar_avatar
    (add_action "pay_attention_exemplar_avatar")
)

(add_rule (cog-new-stv 0.9 1.0) SatisfyOwnerGoal pay_attention_exemplar_avatar
          (add_gpn_precondition "learningModePrecondition")
)

(define try
    (add_action "try")
)

(add_rule (cog-new-stv 0.9 1.0) SatisfyOwnerGoal try
          (add_gpn_precondition "tryRunSchemaPrecondition")
)

(define executeRequestedAction
    (add_action "executeRequestedAction") 
)

(add_rule (cog-new-stv 0.9 1.0) SatisfyOwnerGoal executeRequestedAction
          (add_gpn_precondition "executeRequestedActionPrecondition") 
)

(define say_n_bark
    (add_action "say_n_bark" "'custom_message'" "'all_agents'")
)

(add_rule (cog-new-stv 0.8 1.0) SatisfyOwnerGoal say_n_bark
          (add_gpn_precondition "saySomethingPrecondition")
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
