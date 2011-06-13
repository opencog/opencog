---------------------- RECOMMENDATIONS---------------
-- 1) The rules format is as follows:
--    rule ( rule name
--			 rule type
--			 rule strengths {one strength for each agent mode}
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

------ DEFAULT RULES -------

rule( "defaultAction", SCHEMA_RULE, { PLAYING_MODE=0.001, SCAVENGER_HUNT_MODE=0.001, LEARNING_MODE=0.001 },
      "defaultActionPrecondition",
      "receive_latest_group_commands()" );




------ SCAVENGER HUNTER ACTIONS ------

rule( "callOtherAgentsToPlay",  SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "callOtherAgentsToPlayPrecondition",
      "group_command_4( all_agents, 'lets_play_scavenger_hunt', randbool, rand )" );

rule( "answerCallToPlay",  SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "answerCallToPlayPrecondition",
      "group_command_4( all_agents, 'lets_play_scavenger_hunt', randbool, rand )" );

rule( "waitForOtherAgentsAnswers",  SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "waitForOtherAgentsAnswersPrecondition",
      "receive_latest_group_commands( )" );

rule( "gotoTeamBase", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "gotoTeamBasePrecondition",
      "goto_obj(custom_object, 4)" );

rule( "waitForGameStart", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "waitForGameStartPrecondition",
      "receive_latest_group_commands( )" );

rule( "gotoTreasure", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "gotoTreasurePrecondition",
      "goto_obj(custom_object, 6)" );

rule( "grabTreasure", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "grabTreasurePrecondition",
      "grab_nearest_pickupable()" );

rule( "bringTreasureToBase", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "bringTreasureToBasePrecondition",
      "goto_obj(custom_object, 6)" );

rule( "dropTreasure", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "dropTreasurePrecondition",
      "drop()" );



rule( "exploreObstacle", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "exploreObstaclePrecondition",
      "goto_obj(custom_path, 4)" );

rule( "lookAtTileDirection", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "lookAtTileDirectionPrecondition",
      "turn_to_face(custom_position)" );

rule( "sendExploredAreaNotification", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "sendExploredAreaNotificationPrecondition",
      "group_command_3( all_agents, 'explored_area', custom_object )" );

rule( "waitForOtherAgentsChosenAreaConfirmation", SCHEMA_RULE,{PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "waitForOtherAgentsChosenAreaConfirmationPrecondition",
      "receive_latest_group_commands( )" );

rule( "lookAtAreaCenter", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "lookAtAreaCenterPrecondition",
      "turn_to_face(custom_position)" );

rule( "gotoBaseToCompute", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "gotoBaseToComputePointsPrecondition",
      "goto_obj(custom_object, 6)" );

rule( "sendGameStartedMessage", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "sendGameStartedMessagePrecondition",
      "group_command_2( all_agents, 'game_started' )" );


-- group actions
rule( "chooseAnAreaToExplore", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "chooseAnAreaToExplorePrecondition",
      "group_command_4( all_agents, 'desired_area_to_explore', rand, rand )" );

rule( "gotoVisibleTile", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "gotoVisibleTilePrecondition",
      "goto_obj(custom_path, 6)" );

rule( "lookAtTileDirection", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "lookAtTileDirectionPrecondition",
      "turn_to_face(custom_position)" );

rule( "followTarget", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "followTargetPrecondition",
      "goto_obj(custom_path, 4)" );

rule( "waitingForCommand", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "waitingForCommandPrecondition",
      "stay()" );

rule( "lookForTarget", SCHEMA_RULE, { PLAYING_MODE=0, SCAVENGER_HUNT_MODE=0.8, LEARNING_MODE=0 },
      "lookForTargetPrecondition",
      "turn_to_face(custom_position)" );
