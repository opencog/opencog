---------------------- RECOMMENDATIONS---------------
-- 1) The rules format is as follows:
--    rule ( rule name
--			 rule type
--			 rule strength
--			 rule precondition
--			 rule effect ) 
-- 
-- 2) Currently there are three rule types: 
--    * SCHEMA_RULE - whose effect is the execution of a combo schema
--	  * FEELING_RULE - whose effect is the change of a agent feeling
--	  * RELATION_RULE - whose effect is the change in the agent relation 
-- 		with something or someone
--
-- 3) Strength makes sense only for SCHEMA_RULES, so when creating rules
--	  from other types just set strength to 1.0 meaning that, if actived,
--	  the rule effect must be executed
--
-- 4) Rule precondition should be a combo script coded in 
--	  RulesPreconditions.combo file. These preconditions should be named
--	  with the rule name adding the "Precondition" suffix
--
-----------------------------------------------------


---------------------- ACTIONS ----------------------

--- BEG ---

rule( "begCompany", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "begCompanyPrecondition",
      "beg_to_owner()" );

rule( "begFood", SCHEMA_RULE, { PLAYING_MODE=0.6, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "begFoodPrecondition",
      "beg_to_owner()" );

rule( "begMercy", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "begMercyPrecondition",
      "beg_to_target('_*_')" );

--- POO ---

rule( "wanderToFindPooPlace", SCHEMA_RULE, { PLAYING_MODE=0.88, SCAVENGER_HUNT_MODE=0.88, LEARNING_MODE=0.88 },
      "wanderToFindPooPlacePrecondition",
      "wander_searching_poo_place()" );

rule( "poo", SCHEMA_RULE, { PLAYING_MODE=0.9, SCAVENGER_HUNT_MODE=0.9, LEARNING_MODE=0.9 },
      "pooPrecondition",
      "drop_n_poo('_*_')" );

--- PEE ---

rule( "peeToMarkTerritory", SCHEMA_RULE, { PLAYING_MODE=0.1, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "peeToMarkTerritoryPrecondition",
      "drop_n_pee('_*_')" );

rule( "pee", SCHEMA_RULE, { PLAYING_MODE=0.9, SCAVENGER_HUNT_MODE=0.9, LEARNING_MODE=0.9 },
      "peePrecondition",
      "drop_n_pee('_*_')" );

rule( "wanderToFindPeePlace", SCHEMA_RULE, { PLAYING_MODE=0.87, SCAVENGER_HUNT_MODE=0.5, LEARNING_MODE=0.5 },
      "wanderToFindPeePlacePrecondition",
      "wander_searching_pee_place()" );

--- NAP ---

rule( "takeANapNearFriend", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "takeANapNearFriendPrecondition",
      "sleep()" );

rule( "takeANapAtHome",  SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "takeANapAtHomePrecondition",
      "sleep()" );

rule( "veryTiredNap", SCHEMA_RULE, { PLAYING_MODE=0.91, SCAVENGER_HUNT_MODE=0.91, LEARNING_MODE=0.91 },
      "veryTiredNapPrecondition",
      "sleep()" );

rule( "napBeforeStarvation", SCHEMA_RULE, { PLAYING_MODE=0.92, SCAVENGER_HUNT_MODE=0.92, LEARNING_MODE=0.92 },
      "napBeforeStarvationPrecondition",
      "sleep()" );

--- SIT ---

rule( "sitToGetFood", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "sitToGetFoodPrecondition",
      "pay_attention('_*_')" );

rule( "sitToRest", SCHEMA_RULE, { PLAYING_MODE=0.2, SCAVENGER_HUNT_MODE=0.2, LEARNING_MODE=0.2 },
      "sitToRestPrecondition",
      "sit()" );


--- APPROACH ---

rule( "hideWhenUnknownAvatarIsComming", SCHEMA_RULE, { PLAYING_MODE=0.6, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "hideWhenUnknownAvatarIsCommingPrecondition",
      "goto_owner()" )

rule( "approachUnknownAvatarToInspect", SCHEMA_RULE, { PLAYING_MODE=0.6, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "approachUnknownAvatarToInspectPrecondition",
      "walkto_obj('_*_' )" )

rule( "approachFriendWhenTired", SCHEMA_RULE, { PLAYING_MODE=0.5, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "approachFriendWhenTiredPrecondition",
      "walkto_obj('_*_' )" );

rule( "approachToInspect", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "approachToInspectPrecondition",
      "walkto_obj('_*_' )" );

rule( "approachToAttack", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "approachToAttackPrecondition",
      "walkto_obj('_*_' )" );

rule( "approachToDistractOwner", SCHEMA_RULE, { PLAYING_MODE=0.71, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "approachToDistractOwnerPrecondition",
      "goto_owner()" );

--- BARK ---

rule( "barkToFrighten", SCHEMA_RULE, { PLAYING_MODE=0.21, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "barkToFrightenPrecondition",
      "bark_to_target('_*_')" );

rule( "barkToProtectFood", SCHEMA_RULE, { PLAYING_MODE=0.6, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
   	  "barkToProtectFoodPrecondition",
      "bark_to_target('_*_')" );

rule( "barkFear", SCHEMA_RULE, { PLAYING_MODE=0.5, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "barkFearPrecondition",
      "bark()" );

rule( "barkWhenPlay", SCHEMA_RULE, { PLAYING_MODE=0.2, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "barkWhenPlayPrecondition",
      "bark()" );

rule( "barkAfterDetectANewObject", SCHEMA_RULE, { PLAYING_MODE=0.3, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "barkAfterDetectANewObjectPrecondition",
      "bark()" );

--- BITE ---

rule( "biteEnemy", SCHEMA_RULE, { PLAYING_MODE=0.5, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "biteEnemyPrecondition",
      "drop_n_bite('_*_')" );

rule( "biteWhenAttackedButCalm", SCHEMA_RULE, { PLAYING_MODE=0.2, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "biteWhenAttackedButCalmPrecondition",
      "drop_n_bite('_*_')" );

rule( "biteWhenAttacked", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "biteWhenAttackedPrecondition",
      "drop_n_bite('_*_')" );

rule( "biteToProtectFood", SCHEMA_RULE, { PLAYING_MODE=0.9, SCAVENGER_HUNT_MODE=0.9, LEARNING_MODE=0.9 },
      "biteToProtectFoodPrecondition",
      "drop_n_bite('_*_')" );

rule( "biteEnemyWhenAngry", SCHEMA_RULE, { PLAYING_MODE=0.6, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "biteEnemyWhenAngryPrecondition",
      "drop_n_bite( '_*_' )" );

--- LICK ---

rule( "lickForGratitude", SCHEMA_RULE, { PLAYING_MODE=0.1, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "lickForGratitudePrecondition",
      "drop_n_lick('_*_')" );


--- Nil : Warning the weight of lickToInspect must be higher than
--- the weight of sniffToInspect, otherwise there may be the risk that
--- the pet would sniff forever (at least when no
--- random noise is introduced in the rule selection process)
rule( "lickToInspect", SCHEMA_RULE, { PLAYING_MODE=0.8, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "lickToInspectPrecondition",
      "drop_n_lick('_*_')" );

rule( "lickForLove", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "lickForLovePrecondition",
      "drop_n_lick('_*_')" )

--- SNIFF ---

--- Nil : Warning the weight of sniffToInspect must be lower than
--- the weight of lickToInspect, otherwise there may be the risk that
--- the pet would sniff forever (at least when no
--- random noise is introduced in the rule selection process)
rule( "sniffToInspect",  SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "sniffToInspectPrecondition",
      "drop_n_sniff('_*_')" )

rule( "sniffJealousy",  SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "sniffJealousyPrecondition",
      "drop_n_sniff('_*_')" )

--- JUMP ---

rule( "jumpJoy",  SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "jumpJoyPrecondition",
      "jump_up()" )

rule( "jumpPlay", SCHEMA_RULE, { PLAYING_MODE=0.2, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "jumpPlayPrecondition",
      "goto_back_flip('_*_')" )


--- PLAY ---

rule( "playWithOwner", SCHEMA_RULE, { PLAYING_MODE=0.15, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "playWithOwnerPrecondition",
      "play()" )

rule( "playWhenAvatarBecomeFriend", SCHEMA_RULE, { PLAYING_MODE=0.6, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "playWhenAvatarBecomeFriendPrecondition",
      "play()" )

--- BRING ---

rule( "bringToPetFriend", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "bringRandomObjectToPetFriendPrecondition",
      "bringRandomObject('_*_')" );

rule( "moveObjectToPlay", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "moveObjectToPlayPrecondition",
      "moveObjectToPlay()" );

--rule( "bringToOwner", SCHEMA_RULE, { PLAYING_MODE=0.1, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "bringRandomObjectToOwnerPrecondition",
--      "bringRandomObject('owner')" );

--- WANDER ---

rule( "lookForThingsToDestroy", SCHEMA_RULE, { PLAYING_MODE=0.1, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "lookForThingsToDestroyPrecondition",
      "goto_random_pickupable()" );

rule( "lookForNewThings", SCHEMA_RULE, { PLAYING_MODE=0.15, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "lookForNewThingsPrecondition",
      "goto_random_pickupable()" );

rule( "lookForNearThingsWhenFearful", SCHEMA_RULE, { PLAYING_MODE=0.16, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "lookForNearThingsWhenFearfulPrecondition",
      "goto_nearest_and_grabit()" );

rule( "lookForCompany", SCHEMA_RULE, { PLAYING_MODE=0.78, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "lookForCompanyPrecondition",
      "gonear_obj('_*_')" );

-- if the goto or gonear target is a moving object, update the target position if necessary
-- Mapinfo delays is causing errors on replanning path to target, so it will be commented until
-- that problem is not resolved
--rule( "keepMovingToTarget", SCHEMA_RULE, { PLAYING_MODE=0.781, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "keepMovingToTargetPrecondition",
--      "keepMoving()" );

--- CHASE MOVING OBJECT --

rule( "chaseMovingObject", SCHEMA_RULE, { PLAYING_MODE=0.4, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "chaseMovingObjectPrecondition",
      "trotto_obj('_*_')" );

rule( "chasePetHoldingSomething", SCHEMA_RULE, { PLAYING_MODE=0.75, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "chasePetHoldingSomethingPrecondition",
      "chasePetHoldingSomething('_*_')" );

--- FOLLOW_OWNER ---

--- the rule below has been commented
--- for the social behavior video shot

rule( "followOwner", SCHEMA_RULE, { PLAYING_MODE=0.71, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "followOwnerPrecondition",
      "goto_owner()" );

---------------------- FEELINGS ----------------------
 
rule( "feelHate", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "feelHatePrecondition",
      "feel('hate', 0.7)" );

rule( "feelAngerNaturally", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelAngerNaturallyPrecondition",
      "feel( 'anger', 0.8 )" );

rule( "feelFearWhenUnknownAvatarIsComming", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelFearWhenUnknownAvatarIsCommingPrecondition",
      "feel( 'fear', 0.7 )" )  

rule( "feelFearWhenAttacked", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelFearWhenAttackedPrecondition",
      "feel( 'fear', 0.7 )" )

rule( "feelFearNaturally", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelFearNaturallyPrecondition",
      "feel( 'fear', 0.8 )" );

rule( "feelPride", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelPridePrecondition",
      "feel( 'pride', 0.7 )" );

rule( "feelExcitement",  FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelExcitementPrecondition",
      "feel('excitement', 0.7)" );

rule( "feelAngerWhenBored", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelAngerWhenBoredPrecondition",
      "feel('anger', 0.7)" );

rule( "feelAngerWhenAttacked", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelAngerWhenAttackedPrecondition",
      "feel('anger', 0.7)" );

rule( "feelComfortableAfterBark", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelComfortableAfterBarkPrecondition",
      "feel('happiness', 0.7 )" );

rule( "feelHappinessWhenNearFriend", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelHappinessWhenNearFriendPrecondition",
      "feel( 'happiness', 0.7 )" );

--rule( "feelHappinessWhenBelongsToAGroup", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
--      "feelHappinessWhenBelongsToAGroupPrecondition",
--      "feel( 'happiness', 0.7 )" );

rule( "feelGratitudeWhenReceiveFood", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelGratitudeWhenReceiveFoodPrecondition",
      "feel('gratitude', 0.7)" );

rule( "feelHappiness", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelHappinessPrecondition",
      "feel('happiness', 0.7)" );

rule( "feelExcitementAfterEat", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelExcitementAfterEatPrecondition",
      "feel('excitement', 0.7)" );

rule( "feelExcitementeWhenNovelty", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelExcitementeWhenNoveltyPrecondition",
      "feel('excitement', 0.8)" );

rule( "feelExcitementWhenBringObjectToOwner", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelExcitementWhenBringObjectToOwnerPrecondition",
      "feel( 'excitement', 0.6 )" )

rule( "feelLove", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelLovePrecondition",
      "feel('love', 0.7)" );

rule( "feelAnger", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelAngerPrecondition",
      "feel('anger', 0.7)" );

rule( "feelGratitude", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelGratitudePrecondition",
      "feel('gratitude', 0.7 )" );

rule( "feelFear", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 }, 
      "feelFearPrecondition",
      "feel('fear', 0.7)" );

------ LEARNING MODE ACTIONS ------

rule( "approachToLearn", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0.87 },
      "approachToLearnPrecondition",
      "gonear_exemplar_avatar()" );

rule( "learningMode", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0.86 },
      "learningModePrecondition",
      "pay_attention_exemplar_avatar()" );

rule( "tryRunSchema", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0.86 },
      "tryRunSchemaPrecondition",
      "try()" );

rule( "executeRequestedAction", SCHEMA_RULE, { PLAYING_MODE=0.861, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0.861 },
      "executeRequestedActionPrecondition",
      "executeRequestedAction()" );

rule( "saySomething", SCHEMA_RULE, { PLAYING_MODE=0.9, SCAVENGER_HUNT_MODE=0.9, LEARNING_MODE=0.9 },
      "saySomethingPrecondition",
      "say_n_bark('custom_message', 'all_agents')" );

------ GROUP ACTIONS -------
--rule( "goBehindPetToInspectIt",  SCHEMA_RULE, { PLAYING_MODE=0.22, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "goBehindPetToInspectItPrecondition",
--      "gobehind_obj('_*_' )" );

--rule( "goBehindFriendlyPetWhenNear", SCHEMA_RULE, { PLAYING_MODE=0.22, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "goBehindFriendlyPetWhenNearPrecondition",
--      "gobehind_obj('_*_' )" );

--rule( "requestGrouping", SCHEMA_RULE, { PLAYING_MODE=0.23, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "requestGroupingPrecondition",
--      "group_command('request_grouping', '_*_' )" );

--rule( "joinOwnGroup", SCHEMA_RULE, { PLAYING_MODE=0.23, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "joinOwnGroupPrecondition",
--      "group_command('join_group', self )" );

--rule( "reJoinGroup", SCHEMA_RULE, { PLAYING_MODE=0.24, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "reJoinGroupPrecondition",
--      "group_command('join_group', getGroupLeaderId( ) )" );

--rule( "joinGroup", SCHEMA_RULE, { PLAYING_MODE=0.23, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "joinGroupPrecondition",
--      "group_command('join_group', '_*_' )" );

--rule( "followGroupLeader", SCHEMA_RULE, { PLAYING_MODE=0.23, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "followGroupLeaderPrecondition",
--      "gonear_obj( getGroupLeaderId( ) )" );

--rule( "abandonGroup", SCHEMA_RULE, { PLAYING_MODE=0.24, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "abandonGroupPrecondition",
--      "group_command( 'abandon_group', self )" );

--rule( "executeGroupLeaderCommand", SCHEMA_RULE, { PLAYING_MODE=0.25, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
--      "executeGroupLeaderCommandPrecondition",
--      "group_command( 'execute_leader_command', '_*_' )" );

------ DEFAULT ACTION ------

rule( "defaultAction", SCHEMA_RULE, { PLAYING_MODE=0.001, SCAVENGER_HUNT_MODE=0.001, LEARNING_MODE=0.001 }, 
      "defaultActionPrecondition",
      "sit()" );

rule( "defaultAggressiveAction", SCHEMA_RULE, { PLAYING_MODE=0.001, SCAVENGER_HUNT_MODE=0.001, LEARNING_MODE=0.001 },
      "defaultAggressiveActionPrecondition",
      "bark()" );

--- If you want the pet executes learned tricks spontaneously, set the weight of this rule to a non-null value (e.g., 0.27)
rule( "selectLearnedTrick", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "selectLearnedTrickPrecondition",
      "runRandomTopFiveLearnedTrick()" );

----------------------------------------
------ PET-TO-PET SOCIAL BEHAVIOR ------
----------------------------------------

--------------- ACTIONS ----------------


---- APPROACH ----

rule( "approachCuriousPet", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
      "approachCuriousPetPrecondition",
      "goto_sniff('_*_')" );

rule ( "goAwayFearfulPet", SCHEMA_RULE, { PLAYING_MODE=0.61, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
       "goAwayFearfulPetPrecondition",
       "go_away_from('_*_')" );

rule ( "approachAngryPet", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
       "approachAngryPetPrecondition",
       "goto_growl_or_bite('_*_')" );

---- REACTION ----

rule ( "beenGrowledPet", SCHEMA_RULE, { PLAYING_MODE=0.65, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
       "beenGrowledPetPrecondition",
       "react_from_growl('_*_')" );

rule ( "beenBittenPet", SCHEMA_RULE, { PLAYING_MODE=0.9, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
       "beenBittenPetPrecondition",
       "react_from_bite('_*_')" );

rule ( "beenSniffedPet", SCHEMA_RULE, { PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
       "beenSniffedPetPrecondition",
       "react_from_sniff('_*_')" );

---- if near friend and friend has been bitten then bark at the bitter ----

rule ( "defendFriendFromBite",
       SCHEMA_RULE,
       { PLAYING_MODE=0.8, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
       "defendFriendFromBitePrecondition",
       "goto_bark_to_target('_*_')" );

--------------- FEELINGS ----------------

rule ( "feelLessAngryAfterGotoGrowlOrBite", FEELING_RULE, { PLAYING_MODE=1.0, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
       "feelLessAngryAfterGotoGrowlOrBitePrecondition",
       "feel( 'anger', 0.4 )" );


--------------- FOR TESTING INDIVIDUAL PREDICATE ------------

---rule ( "nearTest", SCHEMA_RULE, { PLAYING_MODE=0.01, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
---       "nearPrecondition",
---       "dummy('_*_')" );

---rule ( "curiousTest", SCHEMA_RULE, { PLAYING_MODE=0.01, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
---       "curiousPrecondition",
---       "dummy('_*_')" );

------ SCAVENGER HUNTER ACTIONS ------
--rule( "approachOwnerToReceiveInstructions", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.87, LEARNING_MODE=0 },
--      "approachOwnerToReceiveInstructionsPrecondition",
--      "gonear_obj('_*_')" );
--
--rule( "payAttentionOnOwner", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.4, LEARNING_MODE=0 },
--      "payAttentionOnOwnerPrecondition",
--      "pay_attention('_*_')" );
--
--rule( "goHideAtHome", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.5, LEARNING_MODE=0 },
--      "goHideAtHomePrecondition",
--      "goto_pet_home()" );
--
--rule( "stayHindingAtHome", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.6, LEARNING_MODE=0 },
--      "stayHindingAtHomePrecondition",
--      "pay_attention('_*_')" );
--
--rule( "exploreObject", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.6, LEARNING_MODE=0 },
--      "exploreObjectPrecondition",
--      "walkto_obj(custom_path)" );
--
--rule( "exploreHiddenArea", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.6, LEARNING_MODE=0 },
--      "exploreHiddenAreaPrecondition",
--      "walkto_obj(custom_path)" );
--
--rule( "gotoTreasure", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.99, LEARNING_MODE=0 },
--      "gotoTreasurePrecondition",
--      "walkto_obj(custom_object)" );
--
--rule( "pickupTreasure", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.99, LEARNING_MODE=0 },
--      "pickupTreasurePrecondition",
--      "grab_nearest_object()" );
--
--rule( "goBackToOwner", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.99, LEARNING_MODE=0 },
--      "goBackToOwnerPrecondition",
--      "goto_owner()" );
--
--rule( "celebrateVictory", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.99, LEARNING_MODE=0 },
--      "celebrateVictoryPrecondition",
--      "drop_n_celebrate()" );
--
--rule( "cryDefeat", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.99, LEARNING_MODE=0 },
--      "cryDefeatPrecondition",
--      "beg()" );

--- DEBUGGING RULE ---

--- rule ( "chainAllActions",
---       SCHEMA_RULE,
---       { PLAYING_MODE=1, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
---       "chainAllActionsPrecondition",
---       "chainAllActions()" );

--- rule ( "debugRule",
---       SCHEMA_RULE,
---       { PLAYING_MODE=1, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0 },
---       "truePrecondition",
---       "debugSchema()" );
