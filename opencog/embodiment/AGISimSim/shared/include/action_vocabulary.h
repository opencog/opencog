/***************************************************************************
 *  Pre-defined Action strings        
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *
 *	19.01.06	FP	formatting 	
 ****************************************************************************/
	
//--- Complex actions -------------------------------------------------------

#define LIFT		"lift"
#define DROP		"drop"
#define GOTO		"goto"
#define SAY			"say"
#define FOLLOW		"follow"
#define WATCH		"watch"

//--- Agent movement --------------------------------------------------------
#define TURN_LEFT	"turn.left"
#define TURN_RIGHT	"turn.right"
#define FORWARD		"move.forward"
#define BACKWARD	"move.backward"
#define STRAFE_LEFT	"strafe.left"
#define STRAFE_RIGHT "strafe.right"
#define KICK_LOW    "kick.low"
#define KICK_HIGH   "kick.high"
#define WALK_TOWARDS "walk.towards"
#define NUDGE_TO "nudge.to"
#define TURN_TO "turn.to"

//--- Hand manipulation -----------------------------------------------------
#define HAND_LEFT	"hand.left"
#define HAND_RIGHT	"hand.right"
#define HAND_FORWARD "hand.forward"
#define HAND_BACKWARD "hand.backward"
#define HAND_UP		"hand.up"
#define HAND_DOWN	"hand.down"

#define HAND_GRASP	"hand.grasp"
#define HAND_UNGRASP "hand.ungrasp"

//--- Eye movement ----------------------------------------------------------
#define EYE_UP		"eye.up"
#define EYE_DOWN	"eye.down"
#define EYE_LEFT	"eye.left"
#define EYE_RIGHT	"eye.right"

//--- Misc ------------------------------------------------------------------
#define EAT			"eat"
#define DRINK			"drink"
#define SMILE "smile"
#define FROWN "frown"

//--- Communication ---------------------------------------------------------
#define NOISE		"noise.make"
#define MESSAGE		"message"

//--- Cart put & get --------------------------------------------------------
#define CART		"cart.attach"
#define DECART		"cart.detach"

//--- Internal State Commands: Use these ops after the stem object: ---------
#define ON			"on"
#define OFF			"off"

//--- The Stem Objects: -----------------------------------------------------

//--- Whether to move cart along when adjacent ------------------------------
#define CART_MOVE	"cart."

//--- Turning specific senses on / off --------------------------------------
#define SIGHT		"sight."
#define SMELL		"smell."
#define TASTE		"taste."
#define HEARING		"hearing."
#define TOUCH		"touch."

//--- How data is received --------------------------------------------------

#define	TIME_CHUNK	"time_quantum"
#define	DATA_CHUNK	"data_quantum"

//#define
