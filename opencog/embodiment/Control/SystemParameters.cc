/*
 * opencog/embodiment/Control/SystemParameters.cc
 *
 * Copyright (C) 2007-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "SystemParameters.h"
#include "util/Logger.h"
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <sstream>

using namespace Control;

SystemParameters::~SystemParameters() {
}

SystemParameters::SystemParameters() {
    emptyString.assign("");
    
    table["CONFIG_FILE"] = "config.cfg"; 
    table["AUTOMATED_SYSTEM_TESTS"] = "0"; 

    // Ids and network settings
    table["ROUTER_ID"] = "ROUTER";
    table["ROUTER_IP"] = "127.0.0.1";
    table["ROUTER_PORT"] = "16312";
    table["ROUTER_AVAILABLE_NOTIFICATION_INTERVAL"] = "15"; // interval to send alive notifications to NE's (in seconds)

    table["SPAWNER_ID"] = "SPAWNER";
    table["SPAWNER_IP"] = "127.0.0.1";
    table["SPAWNER_PORT"] = "16313";

    table["LS_ID"] = "LS";
    table["LS_IP"] = "127.0.0.1";
    table["LS_PORT"] = "16314";

    // These 2 parameters define the maximal number of pets monitored by a same spawner
    table["MIN_OPC_PORT"] = "16326";
    table["MAX_OPC_PORT"] = "16330";

    table["COMBO_SHELL_ID"] = "COMBO_SHELL";
    table["COMBO_SHELL_IP"] = "127.0.0.1";
    table["COMBO_SHELL_PORT"] = "16316";

    // IP and Port are informed by Proxy when it sends a LOGIN message to router.
    // Anyway, fyi, the mockyProxy uses port 16315 and the real Proxy 8211
    table["PROXY_ID"] = "PROXY"; 

    // OPC settings
    table["UNKNOWN_PET_OWNER"] = "no_owner_id";
    table["UNKNOWN_PET_NAME"] = "no_name";
    table["PET_INTERFACE_ENABLED"] = "0";
    table["PET_INTERFACE_UPDATE_PERIOD"] = "10"; // in cycles
    table["NAVIGATION_ALGORITHM"] = "tangentbug"; // options: tangentbug, astar or hpa
    table["HPA_MAXIMUM_CLUSTERS"] = "16";

    table["PET_WALKING_SPEED"] = "2.0"; // in m/s . A non-positive number means a random value between 0.5 and 3.5 will be used.

    // log levels
    char fine[64], debug[64], info[64], warning[64], error[64];
    sprintf(fine, "%d", opencog::Logger::FINE);
    sprintf(debug, "%d", opencog::Logger::DEBUG);
    sprintf(info, "%d", opencog::Logger::INFO);
    sprintf(warning, "%d", opencog::Logger::WARN);
    sprintf(error, "%d", opencog::Logger::ERROR);
    
    // component log levels
    table["PROXY_LOG_LEVEL"] = fine;
    table["LS_LOG_LEVEL"] = fine;
    table["OPC_LOG_LEVEL"] = fine;
    table["ROUTER_LOG_LEVEL"] = fine;
    table["SPAWNER_LOG_LEVEL"] = fine;
    table["OPENCOG_LOG_LEVEL"] = fine;
    table["TIMESTAMP_ENABLED_IN_LOGS"] = "1";
    // TODO: Convert all log level parameters above in string, which is more
    // readable (see opencog/util/Config.h and Logger::getLevelFromString), just
    // like bellow:
    table["BACK_TRACE_LOG_LEVEL"] = "warn"; 
    
    // paths
    table["LOG_DIR"] = "/tmp/$USER/Petaverse/Logs";
    table["MESSAGE_DIR"] = "/tmp/$USER/Petaverse/queue";
    table["PET_DATABASE"] = "/tmp/$USER/Petaverse/PetDatabase";
    table["PROXY_DATABASE_DIR"] = "/tmp/$USER/Petaverse/ProxyDataBase";
    table["ROUTER_DATABASE_DIR"] = "/tmp/$USER/Petaverse/RouterDataBase";

    // filenames
    table["PET_DUMP"] = "pet.dump";
    table["PROXY_DATA_FILE"] = "proxy.dat";
    table["ROUTER_DATA_FILE"] = "router.dat";
    table["VOCABULARY_FILE"]  = "petavese.voc";
    table["ATOM_SPACE_DUMP"]  = "atomSpace.dump";
    table["COMBO_STDLIB_REPOSITORY_FILE"] = "stdlib.combo";
    table["COMBO_RULES_PRECONDITIONS_REPOSITORY_FILE"] = "rules_preconditions.combo";
    table["COMBO_SELECT_RULES_PRECONDITIONS_REPOSITORY_FILE"] = "rules_preconditions.combo-select";
    table["COMBO_RULES_ACTION_SCHEMATA_REPOSITORY_FILE"] = "rules_action_schemata.combo";

    // Rule Engine parameters
    table["RE_CYCLE_PERIOD"] = "2"; //counted in term multiple idle cycles
    table["RE_DEFAULT_MEAN"] = "0.5";
    table["RE_DEFAULT_COUNT"] = "18"; 				 // set to make confidence = 0.9 (see #define KKK 
    												 // in SimpleTruthValue.cc and 
    											     // SimpleTruthvalue::getConfidence()
    table["RE_CYCLES_DURING_NOVELTY"] = "5";
    table["RE_FEELINGS_DECREASE_FACTOR"] = "0.01";   
    table["RE_CYCLES_FOR_REQUESTED_SCHEMA"] = "7";
    table["RE_CYCLES_DURING_AGENT_LAST_ACTION"] = "3";

    // rule engine configurations
    table["RE_CORE_FILE"] = "rules_core.lua";    
    table["RE_DEFAULT_PET_TRAITS"] = "maxie";
    table["RE_DEFAULT_HUMANOID_TRAITS"] = "maria";
    table["RE_RULES_FILENAME_MASK"]  = "%s_rules.lua";
    table["RE_TRAITS_FILENAME_MASK"] = "%s_traits_%s.lua";	
    table["RE_PET_DEFINITIONS_FILE"] = "petDefinitions.lua";

    //about the randomization of schema rule selection
    //0 means, no random noise, the engine chooses the rule with max weight
    //1 means maximal noise
    //and any float in between represents the noise intensity
    table["RE_SCHEMA_SELECTION_RANDOM_NOISE"] = "0";
    //0 means no random selection the first found is taken
    //1 means with random selection (uniform here)
    table["RE_WILD_CARD_RANDOM_SELECTION"] = "0";

    // rule validation configurations
    table["RV_CORE_FILE"] = "validation_core.lua";    
       
    // Reinforcement learning parameters
    table["RL_REWARD"] = "0.05";                     // max reward to be applied to a rule implication link
    table["RL_PUNISH"] = "0.05";                     // max punish to be applied to a rule implication link  
    table["RL_TIME_WINDOW"] = "5.0";                 // in secs 
    table["RL_GAUSSIAN_MEAN"] = "2.5";               // in secs
    table["RL_GAUSSIAN_STD_DEVIATION"] = "0.5";

    table["MAX_RULE_STRENGTH"] = "0.95";             // the maximum strength allowd to a rule via
                                                     // reinforcement learning
   
    // pet commands
    table["STOP_LEARNING_CMD"] = "stop learning";
    table["TRY_SCHEMA_CMD"] = "try";
    
    // reward values
    table["POSITIVE_REWARD"] =  "1.0";
    table["NEGATIVE_REWARD"] = "-1.0";

    // Parameter to control the maximal time spent by a procedure to be executed by Procedure Interpreter (in seconds)
    table["PROCEDURE_EXECUTION_TIMEOUT"] = "90"; 

    // Indicates if walk actions sent to PVP can be canceled so that new navigation plan be sent to PVP when needed. 
    table["ALLOW_WALKING_CANCELATION"] = "false"; 

    //LearningServer parameters
    //number of fitness estimations computed per cycle
    //between 1 and infty
    //the higher the less responsive but the lower the more wasted CPU
    table["NUMBER_OF_ESTIMATIONS_PER_CYCLE"] = "100";

    // SpaceMap grid dimensions. X and Y directions
    table["MAP_XDIM"] = "1024";
    table["MAP_YDIM"] = "1024";
    
    table["IMPORTANCE_DECAY_ENABLED"]      = "1";
    table["ACTION_SELECTION_ENABLED"]      = "1";
    table["COMBO_INTERPRETER_ENABLED"]     = "1";
    table["SCHEMA_GOAL_MINING_ENABLED"]    = "1";
    table["PROCEDURE_INTERPRETER_ENABLED"] = "1";

    table["DISABLE_LOG_OF_PVP_MESSAGES"] = "1";

    table["CHECK_OPC_MEMORY_LEAKS"] = "0";
    table["CHECK_OPC_MEMORY_USAGE"] = "0";
    table["VALGRIND_PATH"] = "/usr/local/bin/valgrind";
    table["MASSIF_DEPTH"] = "30";
    table["MASSIF_DETAILED_FREQ"] = "10";

    table["RUN_OPC_DEBUGGER"] = "0";
    table["OPC_DEBUGGER_PATH"] = "/usr/bin/gdb";

    // NetworkElement's message reading parameters
    table["UNREAD_MESSAGES_CHECK_INTERVAL"] = "10";
    table["UNREAD_MESSAGES_RETRIEVAL_LIMIT"] = "1"; // -1 for unlimited number of retrieved messages
    table["NO_ACK_MESSAGES"] = "0"; 
    table["WAIT_LISTENER_READY_TIMEOUT"] = "60";  // time (in seconds) to wait for socket Listener to be ready 

    //------------------
    //for LearningServer
    //------------------
    //entropy threshold for the perception filter,
    //between 0 [completely open] and 1 [completely closed]
    table["ENTROPY_PERCEPTION_FILTER_THRESHOLD"] = "0.01";
    //similarity threshold for the action filter
    //between 0 [completely open] and 1 [completely closed]
    table["SIMILARITY_ACTION_FILTER_THRESHOLD"] = "0.01";
    //the max size of the action subsequence generated by the ActionFilter
    //if -1 then the max size is maximum
    table["ACTION_FILTER_SEQ_MAX"] = "2";
    //choose the imitation learning algorithm
    //see static strings defined in the header for the various options
    table["IMITATION_LEARNING_ALGORITHM"] = ImitationLearningAlgo::HillClimbing;
    //table["IMITATION_LEARNING_ALGORITHM"] = ImitationLearningAlgo::MOSES;

    //evanescence delay of has_said perception in time unit
    table["HAS_SAID_DELAY"] = "200";

	//lower bound (high age) of atoms in atomTable
    table["ATOM_TABLE_LOWER_STI_VALUE"] = "-400";

    //0 with no random operator optimization for NoSpaceLife
    //1 with optimization (to avoid Monte Carlos simulations)
    table["RANDOM_OPERATOR_OPTIMIZATION"] = "1";

    //that flag indicates to the neighborhood expension of
    //hillclimbing whether (1) or not (0) both branches of
    //a conditional action_boolean_if should be filled at once
    //instead of only one
    table["ACTION_BOOLEAN_IF_BOTH_BRANCHES_HC_EXPENSION"] = "1";

    //depending on the option, in hillclimbing when a new exemplar comes
    //we either get restarted from the best program so far (value 0)
    //or from the start (the empty combo_tree) (value 1)
    table["HC_NEW_EXEMPLAR_INITIALIZES_CENTER"] = "1";

    //signed integer that indicates the size of
    //1) while operators
    //2) conditional operators
    //3) contin constant
    //that is to favor (little size or negative) or unfavor (large size)
    //in the search process
    table["WHILE_OPERATOR_SIZE"] = "3";
    table["CONDITIONAL_SIZE"] = "1";
    table["CONTIN_SIZE"] = "0";

    //SizePenalty coef A and B, determined using SPCTools
    table["SIZE_PENALTY_COEF_A"] = "0.03";
    table["SIZE_PENALTY_COEF_B"] = "0.34";

    //Defines the distance (in percentage of the diagonal of the SpaceMap)
    //under which an avatar is considered approaching something
    table["DIST_PERCENTAGE_THRESHOLD_WALK_TO_GOTO"] = "1.0";
    
    // print logs message in standard io
    table["PRINT_LOG_TO_STDOUT"] = "0";

    //perform type checking after loading procedures
    table["TYPE_CHECK_LOADING_PROCEDURES"] = "1";
    table["TYPE_CHECK_GENERATED_COMBO"] = "1";

    table["MANUAL_OPC_LAUNCH"] = "0";
}

void SystemParameters::loadFromFile(const std::string &fileName) {
	
	char line[256];
    std::ifstream fin(fileName.c_str());

    std::string pName;
    std::string pValue;
    
	while (!fin.eof()){
    	fin.getline(line, 256);
        
        // not a comentary or an empty line
        if(line[0] != '#' && line[0] != 0x00){
        	std::istringstream in ((std::string)line);
            
            in >> pName;
            in >> pValue;
           	
            std::map<std::string,std::string>::iterator iter1 = table.find(pName);
        	if (iter1 == table.end()) {
            	printf("Unknown parameter name <%s>. Discarding it.\n", pName.c_str());
        	} else {
            	table[pName] = pValue;
        	}           
        }
    }

    fin.close();    
/*    
    while (fin >> pname) {
        //printf("pname = <%s>\n", pname.c_str());
        fin >> pvalue;
        //printf("pvalue = <%s>\n", pvalue.c_str());
        std::map<std::string,std::string>::iterator iter1 = table.find(pname);
        if (iter1 == table.end()) {
            printf("Unknown parameter name <%s>. Discarding it.\n", pname.c_str());
        } else {
            table[pname] = pvalue;
        }
    }
*/    
}

const std::string &SystemParameters::get(const std::string &paramName) const {

    std::map<std::string,std::string>::const_iterator iter1 = table.find(paramName);
    if (iter1 == table.end()) {
        return emptyString;
    } else {
        return iter1->second;
    }
}

