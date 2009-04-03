feelingsDecreaseFactor = 0.01
-- A new discover(avatar/object) will remains novelty for X cycles until it's exclusion
cyclesDuringNovelty = 5
-- An owner action will remain active for X cycles until it's exclusion
cyclesDuringRequestedAction = 7
-- An agent action will remain active considered as latest during X cycles
cyclesDuringAgentLastAction = 3

LESSER_THAN = 0
LESSER_EQUALS = 1
EQUALS = 2
GREATER_EQUALS = 3
GREATER_THAN = 4

-- rule types
-- the ones that execute a schema as its effect
SCHEMA_RULE = 0
-- the ones that change pet feelings as its effect
FEELING_RULE = 1
-- the ones that change pet relations as its effect 
RELATION_RULE = 2

petSignals = {
   "hunger",
   "thirst",
   "pee_urgency",
   "poo_urgency",
   "fitness",
   "energy"   
}

feelings = {
   happiness=0.51,
   fear=0.5,
   pride=0.5,
   love=0.5,
   hate=0.5,
   anger=0.5,
   gratitude=0.5,
   excitement=0.51,
}

-- Helper functions with variable parameters
-- You must inform the number of fixed parameters that becomes before variable parameters
helperFunctionsVariableParameters = {
   isLastAgentAction = 2,
   isNotLastAgentAction = 2,
   isLastPetAction = 1,
   isNotLastPetAction = 1
}

helperFunctions = {
   isInsidePetFov = 1,
   isNotInsidePetFov = 1,
   isPet = 1,
   isNotPet = 1,
   isPeePlace = 1,
   isNotPeePlace = 1,
   isPooPlace = 1,
   isNotPooPlace = 1,
   isDrinkable = 1,
   isNotDrinkable = 1,
   isEdible = 1,
   isNotEdible = 1,
   isPetLearning = 0,
   isPetInGroupingMode = 0,
   isExemplarInProgress = 0,
   isNotExemplarInProgress = 0,
   isThereRelation = 2,
   isNotThereRelation = 2,
   isAtLearningPerimeter = 1,
   isNotAtLearningPerimeter = 1,
   isNear = 1,
   isNotNear = 1,
   isNext = 1,
   isNotNext = 1,
   isMovingTowardMe = 1,
   isNotMovingTowardMe = 1,
   isProportionalNextOperation = 3,
   isNovel = 1,
   isNotNovel = 1,
   isNovelty = 0,
   isNotNovelty = 0,
   isAvatarNovelty = 0,
   isNotAvatarNovelty = 0,
   isObjectNovelty = 0,
   isNotObjectNovelty = 0,
   isObject = 1,
   isNotObject = 1,
   isAvatar = 1,
   isNotAvatar = 1,
   isThereARequestedAction = 0,
   isOwner = 1,
   isNotOwner = 1,
   isMoving = 1,
   isNotMoving = 1,
   hasLearnedTricks = 0,
   currentActionRepetitions = 0,
   getGroupLeaderId = 0,
   getTeacherId = 0
}

contexts = {
   "home",
   "outside",
   "near_avatar",
   "near_friend",
   "near_enemy",
   "near_owner",
   "near_food",
   "near_water",
   "near_poo_place",
   "near_pee_place",
   "next_owner",
   "night"
}

-- action name and arity
actions = {
   beg      = 0,
   beg_to_target = 1,
   eat      = 1,
   drink    = 1,
   sit      = 0,
   goto_obj = 1,
   gobehind_obj = 1,
   gobehind_n_sniff_butt = 1,
   sleep	 = 0,
   bark     = 0,
   bark_to_target = 1,
   bite     = 1,
   flee     = 0,
   lick     = 1,
   sniff    = 1,
   jump_up  = 0,
   play     = 0,
   wander   = 0,
   poo      = 1,
   pee	     = 1,
   gonear_obj = 1,
   drop_n_eat = 1,
   drop_n_drink = 1,
   drop_n_bite = 1,
   drop_n_lick = 1,
   drop_n_sniff = 1,
   drop_n_poo = 1,
   drop_n_pee = 1,
   goto_random_pickupable = 0,
   goto_random_edible = 0,
   goto_random_object = 0,
   goto_random_drinkable = 0,
   goto_random_avatar = 0,
   goto_random_pet = 0,
   goto_random_small = 0,
   goto_random_moving = 0,
   goto_random_poo_place = 0,
   goto_random_pee_place = 0,
   
   goto_nearest_pickupable = 0,
   goto_nearest_edible = 0,
   goto_nearest_object = 0,
   goto_nearest_drinkable = 0,
   goto_nearest_avatar = 0,
   goto_nearest_pet = 0,
   goto_nearest_small = 0,
   goto_nearest_moving = 0,
   goto_nearest_poo_place = 0,
   goto_nearest_pee_place = 0,
   
   pay_attention = 1,
   wander_searching_food = 0,
   wander_searching_water = 0,
   wander_searching_pee_place = 0,
   wander_searching_poo_place = 0,
   goto_object_and_grabit = 1,
   goto_nearest_and_grabit = 0,
   full_of_doubts = 0,
   
   group_command = 2
}

--- special actions (parsed by ruleEngine) ---
special_actions = {
   try = 0, -- used to try to execute a learned schema (do not requires combo action)
   executeRequestedAction = 0, -- used to execute a requested by avatar trick
   runRandomTopFiveLearnedTrick = 0, -- select one learned trick among the top five ranked
   keepMoving = 0, -- update target position while moving
}
