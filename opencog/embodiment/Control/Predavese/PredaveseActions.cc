/*
 * opencog/embodiment/Control/Predavese/PredaveseActions.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#include <opencog/util/Logger.h>
#include <opencog/util/StringManipulator.h>

#include "PredaveseActions.h"
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h>
#include "PredaveseStdafx.h"
#include <opencog/spatial/LocalSpaceMap2DUtil.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include <iostream>
#include <sstream>

using namespace std;
using namespace predavese;
using namespace Control;

/*
 * Reward action:
 */
reward_action::reward_action(PetInterface& petInterface) : action(petInterface)
{
}
bool reward_action::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{
    logger().info("reward_action - Got Reward Instruction");
    logger().debug("reward_action - Args: \n%s", arg->toString().c_str());
    logger().debug("reward_action - petId = %s, timestamp = %lu", petInterface.getPetId().c_str(), timestamp);

    //    AtomSpaceUtil::addRewardPredicate(petInterface.getAtomSpace(), toString(petInterface.getPetId()).c_str(), timestamp);

    petInterface.reward(timestamp);
    return true;
}

/*
 * Punish action:
 */
punish_action::punish_action(PetInterface& petInterface) : action(petInterface)
{
}
bool punish_action::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{
    logger().info("punish_action - Got punish instruction.");
    logger().debug("punish_action - Args: \n%s", arg->toString().c_str());
    logger().debug("punish_action - petId = %s, timestamp = %lu", petInterface.getPetId().c_str(), timestamp);

    //    AtomSpaceUtil::addPunishmentPredicate(petInterface.getAtomSpace(), toString(petInterface.getPetId()).c_str(), timestamp);

    petInterface.punish(timestamp);
    return true;
}

/*
 * Predicate:
 */
predicate_action::predicate_action(PetInterface& petInterface, int _arity ) : action(petInterface), arity(_arity)
{
}
bool predicate_action::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("predicate_action - Got predicate instruction.");
    logger().debug("predicate_action - Args:\n%s", arg->toString().c_str());
    logger().debug("predicate_action - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    // TODO: When we know what this would generate...

    vector<string> f_args;
    FormatArgs(arg->out, 0, back_inserter(f_args), arity);

    if ( f_args[0] == "GO" && ( f_args[1] == "FIND" || f_args[1] == "AFTER" ) && ( f_args[2] == "TREASURE" || f_args[2] == "TREASURES" ) ) {
        petInterface.getCurrentModeHandler( ).handleCommand( "goFindTreasure", std::vector<std::string>( ) );
    } else if ( f_args[0] == "THIS" && ( f_args[1] == "TREASURE" ||
                                         ( f_args[1] == "IS" && f_args[2] == "THE" && f_args[3] == "TREASURE" ) ) ) {
        // get the holded object to setup as the treaure on playing scavenger hunt
        petInterface.getCurrentModeHandler( ).handleCommand( "setupTreasure", std::vector<std::string>() );
    } // if

    return true;
}

/*
 * Developer Meta Commandos
 */
pet_dev_meta_command::pet_dev_meta_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity)
{
}
bool pet_dev_meta_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("pet_dev_meta_command - Got Meta Command.");
    logger().debug("pet_dev_meta_command - Args: %s", arg->toString().c_str());
    logger().debug("pet_dev_meta_command - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    vector<string> f_args;
    FormatArgs(arg->out, 0, back_inserter(f_args), arity);

    string meta_command = f_args.front();
    logger().info("Meta Command: '%s'.", meta_command.c_str());

    if (meta_command == "SAVE_MAP") {
        petInterface.saveSpaceMapFile();
    } else if (meta_command == "SAVE_VISMAP") {
        std::vector<std::string> arguments;
        petInterface.getCurrentModeHandler( ).handleCommand( "saveVisMap", arguments );
    } else {
        logger().warn("Unrecognized meta command: '%s'", meta_command.c_str());
    } // else

    // TODO: When we know what this would generate...
    return true;
}


/*
 * Pet command
 */
pet_command::pet_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity)
{
}
bool pet_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("pet_command - Got pet instruction.");
    logger().debug("pet_command - Args: \n%s", arg->toString().c_str());
    logger().debug("pet_command - petId = %s, timestamp = %lu", petInterface.getPetId().c_str(), timestamp);


    // TODO: Only handles intransitive and transitive verbs. Ditransitive is not addressed yet
    vector<string> f_args;
    FormatArgs(arg->out, 0, back_inserter(f_args), arity);
    opencog::cassert(TRACE_INFO, !f_args.empty(), "pet_command - Unable to format arguments.");

    { // block used to handle playing
        std::vector<std::string> arguments;
        bool playingMode = false;
        unsigned int i;
        for ( i = 0; i < f_args.size( ); ++i ) {
            if ( ( f_args[i] == "LETS" || f_args[i] == "PLAY" ) && i < 2 ) {
                playingMode = true;
            } else if ( playingMode ) {
                arguments.push_back( f_args[i] );
            } // else if
        } // for

        if ( !arguments.empty( ) ) {
            logger().debug("PredaveseActions - User requested to play something. # of arguments: %d", arguments.size( ) );
            if ( ( arguments[0] == "SCAVENGER" && arguments[1] == "HUNT" ) || ( arguments[0] == "SH" ) ) {
                std::vector<std::string> arguments;
                arguments.push_back( "lets_play_scavenger_hunt" ); // PetMode::SCAVENGER_HUNT
                petInterface.getCurrentModeHandler( ).handleCommand( "receivedOwnerCommand", arguments );
                logger().debug("PredaveseActions - Starting to play scavenger hunt" );
                return true;
            } // if

            logger().warn("PredaveseActions - Don't know how to play %s", arguments[0].c_str( ) );
            return false;
        } // if
    } // end block

    if ( f_args[0] == "GO" && ( f_args[1] == "FIND" || f_args[1] == "AFTER" ) && ( f_args[2] == "TREASURE" || f_args[2] == "TREASURES" ) && f_args.size( ) == 4 ) {
        std::vector<std::string> arguments;
        arguments.push_back( f_args[3] );
        arguments.push_back( agentIdWhichSentTheCommand );
        petInterface.getCurrentModeHandler( ).handleCommand( "goFindTreasure", arguments );
    } else if ( f_args[0] == "FOLLOW" && f_args[1] == "ME" && ( f_args.size( ) == 3 || f_args.size( ) == 4 ) ) {
        if ( f_args.size( ) == 3 ) {
            std::vector<std::string> arguments;
            arguments.push_back( f_args[2] );
            arguments.push_back( agentIdWhichSentTheCommand );
            petInterface.getCurrentModeHandler( ).handleCommand( "followMe", arguments );
            return true;
        } else if ( f_args.size( ) == 4 ) {
            std::vector<std::string> arguments;
            arguments.push_back( f_args[3] );
            arguments.push_back( agentIdWhichSentTheCommand );
            petInterface.getCurrentModeHandler( ).handleCommand( "followMeAndCollectTreasures", arguments );
            return true;
        } // else if
    } else if ( f_args[0] == "COME" && f_args[1] == "HERE" && f_args.size( ) == 3 ) {
        std::vector<std::string> arguments;
        arguments.push_back( f_args[2] );
        arguments.push_back( agentIdWhichSentTheCommand );
        petInterface.getCurrentModeHandler( ).handleCommand( "comeHere", arguments );
        return true;
    } else if ( f_args[0] == "EXPLORE" && f_args[1] == "AREA" && f_args.size( ) == 4  ) {
        std::vector<std::string> arguments;
        arguments.push_back( f_args[2] );
        arguments.push_back( f_args[3] );
        arguments.push_back( agentIdWhichSentTheCommand );
        petInterface.getCurrentModeHandler( ).handleCommand( "exploreArea", arguments );
        return true;
    } else if ( f_args[0] == "WAIT" && f_args.size( ) == 2  ) {
        std::vector<std::string> arguments;
        arguments.push_back( f_args[1] );
        arguments.push_back( agentIdWhichSentTheCommand );
        petInterface.getCurrentModeHandler( ).handleCommand( "wait", arguments );
        return true;
    } else if ( f_args[0] == "REGROUP" && f_args[1] == "TEAM" && f_args.size( ) == 3 ) {
        std::vector<std::string> arguments;
        arguments.push_back( f_args[2] );
        arguments.push_back( agentIdWhichSentTheCommand );
        petInterface.getCurrentModeHandler( ).handleCommand( "regroupTeam", arguments );
        return true;
    } else if ( f_args[0] == "SPY" && f_args.size( ) == 3 ) {
        std::vector<std::string> arguments;
        arguments.push_back( f_args[1] );
        arguments.push_back( f_args[2] );
        arguments.push_back( agentIdWhichSentTheCommand );
        petInterface.getCurrentModeHandler( ).handleCommand( "spy", arguments );
        return true;
    } // else if


    string trickName;
    vector<string>::iterator arguments;

    if (f_args[0] != "DO") {
        trickName = f_args[0];
        arguments = f_args.begin() + 1;
    } else {
        trickName = f_args[1];
        arguments = f_args.begin() + 2;
    }

    vector<string> resolvedNames;

    logger().debug("pet_command - trick: %s", trickName.c_str());
    Handle S = petInterface.getAtomSpace().getHandle(GROUNDED_SCHEMA_NODE, trickName);

    if (!CoreUtils::compare(S, Handle::UNDEFINED)) {
        logger().warn("pet_command - Found no schema node: %s", trickName.c_str());
        // Grounded schema S does not exist. Gets the built-in "unknowTrick" schema instead.
        // (i.e., the schema where pet makes an interrogation gesture or something like that)
        S = petInterface.getAtomSpace().getHandle(GROUNDED_SCHEMA_NODE, UNKNOWN_TRICK_SCHEMA_NAME);
        if (!CoreUtils::compare(S, Handle::UNDEFINED)) {
            logger().warn("pet_command - Found no schema node: %s", UNKNOWN_TRICK_SCHEMA_NAME);
        } else {
            // the given command will feed RuleEngine with the unknow trick
            petInterface.setRequestedCommand( UNKNOWN_TRICK_SCHEMA_NAME, resolvedNames);
        }
        // even if the "unknown trick" is exercuted the return of
        // the pet command should be false since the actual commmand wasn't exectuted
        return false;
    }
    while ( arguments != f_args.end() ) {
        // TODO: get speakerId from message
        resolvedNames.push_back(nameResolver->solve(*arguments, petInterface.getOwnerId(), timestamp));
        arguments++;
    }
    // the given command will feed RuleEngine with the requested trick
    petInterface.setRequestedCommand( trickName, resolvedNames );
    return true;
}


/*
 * Pet learn with the owner
 */
pet_learn_command::pet_learn_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity)
{
}
bool pet_learn_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("pet_learn_command - Got learn instruction.");
    logger().debug("pet_learn_command - Args: \n%s",
                 arg->toString().c_str());
    logger().debug("pet_learn_command - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    vector<string> f_args;
    FormatArgs(arg->out, arity - 1, back_inserter(f_args));
    //    foreach(string a, f_args)
    //        printf("\t%s\n", a.c_str());

    if (f_args.size() >= 3 && f_args[f_args.size()-2] == "WITH") {
        // Get the real avatar id, since the received argument is actually the name
        string avatar_id = AtomSpaceUtil::getObjIdFromName(petInterface.getAtomSpace(), f_args.back());
        if (avatar_id.empty()) {
            logger().debug("pet_learn_command - found no avatar with name %s", f_args.back().c_str());
            // There is no object/avatar with such name
            return false;
        }
        petInterface.setExemplarAvatarId( avatar_id );
        f_args.pop_back(); // remove the exemplar avatar id
        f_args.pop_back(); // remove the keyword WITH
    } else  petInterface.setExemplarAvatarId( petInterface.getOwnerId() );

    if (f_args.size() > 0) {
        vector<string>::iterator args = f_args.begin() + 1;
        while (args != f_args.end()) {
            *args = nameResolver->solve(*args, petInterface.getOwnerId(), timestamp);
            args++;
        }
    }

    petInterface.startLearning(f_args, timestamp);
    // TODO: What if there are additional arguments, in addition to the schema name

    return true;
}


/*
 * Pet Stop Learn
 */
pet_stop_learn_command::pet_stop_learn_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity)
{
}
bool pet_stop_learn_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("pet_stop_learn_command - Got stop learn instruction.");
    logger().debug("pet_stop_learn_command - Args: \n%s",
                 arg->toString().c_str());
    logger().debug("pet_stop_learn_command - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    vector<string> f_args;
    FormatArgs(arg->out, arity - 1, back_inserter(f_args));
    //foreach(string a, f_args)
    //    printf("\t%s\n", a.c_str());

    if (f_args.empty()) {
        logger().warn("pet_stop_learn_command - Schema name should be provided.");
        return false;
    }

    if (f_args.size() > 0) {
        vector<string>::iterator args = f_args.begin() + 1;
        while (args != f_args.end()) {
            *args = nameResolver->solve(*args, petInterface.getOwnerId(), timestamp);
            args++;
        }
    }

    petInterface.stopLearning(f_args, timestamp);

    return true;
}


/*
 * Pet stop command
 */
pet_stop_command::pet_stop_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity)
{
}
bool pet_stop_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{
    logger().info("pet_stop_command - Got stop instruction.");
    logger().debug("pet_stop_command - Args: \n%s",
                 arg->toString().c_str());
    logger().debug("pet_stop_command - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    vector<string> f_args;
    FormatArgs(arg->out, arity - 1, back_inserter(f_args));
    //foreach(string a, f_args)
    //    printf("\t%s\n", a.c_str());

    if (f_args.empty()) {
        logger().warn("pet_stop_command - Schema name should be provided.");
        return false;
    }

    if (f_args.size() > 0) {
        vector<string>::iterator args = f_args.begin() + 1;
        while (args != f_args.end()) {
            *args = nameResolver->solve(*args, petInterface.getOwnerId(), timestamp);
            args++;
        }
    }


    petInterface.stopExecuting(f_args, timestamp);

    return true;
}


/*
 * Examplar start
 */
exemplar_start_command::exemplar_start_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity)
{
}
bool exemplar_start_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("exemplar_start - Got exemplar start instruction.");
    logger().debug("exemplar_start - Args: \n%s",
                 arg->toString().c_str());
    logger().debug("exemplar_start - petId = %s, timestamp = %lu, owner id = %s .",
                 petInterface.getPetId().c_str(), timestamp, petInterface.getOwnerId().c_str());

    vector<string> f_args;

    FormatArgs(arg->out, 0, back_inserter(f_args));
    std::string exemplarAvatarId = f_args.front();
    string avatar_id = AtomSpaceUtil::getObjIdFromName(petInterface.getAtomSpace(), f_args.front());
    if (avatar_id.empty()) {
        logger().debug("pet_learn_command - found no avatar with name %s", f_args.front().c_str());
    } else {
        exemplarAvatarId = avatar_id;
    }
    bool isSpecialCase = false;
    if (f_args[1] == "WILL") {
        isSpecialCase = true;
        f_args.clear();
    } else f_args.erase(f_args.begin());

    //foreach(string a, f_args)
    //    printf("\t%s\n", a.c_str());

    if ( f_args.size( ) == 2 && ( f_args[0] == "BLUE" || f_args[0] == "RED" ) && f_args[1] == "TEAM"  ) {
        // an avatar wants to enter in a specific scavenger hunt team

        std::vector<std::string> arguments;
        arguments.push_back( agentIdWhichSentTheCommand );
        arguments.push_back( "lets_play_scavenger_hunt" );
        arguments.push_back( (f_args[0] == "BLUE") ? "1" : "0" ); // 0 = RED, 1 = BLUE
        arguments.push_back( "1.0" ); // greatest rand double
        petInterface.getCurrentModeHandler( ).handleCommand( "receivedGroupCommand", arguments );
        return true;

    } else if (  ( petInterface.getExemplarAvatarId() != petInterface.getOwnerId() && exemplarAvatarId == "I" )
                 || (petInterface.getExemplarAvatarId() != exemplarAvatarId && exemplarAvatarId != "I" ) ) {
        logger().warn("exemplar_start_command - '%s' is an invalid exemplar avatar id. The exemplar id registered is '%s' .", f_args.front().c_str(), petInterface.getExemplarAvatarId().c_str());
        return false;
    }

    if (f_args.size() == 0 && !isSpecialCase) {
        logger().warn("exemplar_start_command - Schema name should be provided.");
        return false;
    }

    if (f_args.size() > 0) {
        vector<string>::iterator args = f_args.begin() + 1;
        while (args != f_args.end()) {
            *args = nameResolver->solve(*args, petInterface.getOwnerId(), timestamp);
            args++;
        }
    }

    petInterface.startExemplar(f_args, timestamp);
    return true;
}


/*
 * Exemplar end
 */
exemplar_end_command::exemplar_end_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity)
{
}
bool exemplar_end_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("exemplar_end - Got exemplar end instruction.");
    logger().debug("exemplar_end - Args: \n%s",
                 arg->toString().c_str());
    logger().debug("exemplar_end - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    vector<string> f_args;
    FormatArgs(arg->out, arity - 1, back_inserter(f_args));
    //foreach(string a, f_args)
    //    printf("\t%s\n", a.c_str());

    /*
      if (f_args.empty()) {
      logger().warn("exemplar_end_command - Schema name should be provided.");
      return false;
      }
    */

    if (f_args.size() > 0) {
        vector<string>::iterator args = f_args.begin() + 1;
        while (args != f_args.end()) {
            *args = nameResolver->solve(*args, petInterface.getOwnerId(), timestamp);
            args++;
        }
    }

    petInterface.endExemplar(f_args, timestamp);

    return true;
}

/*
 * Try schema
 */
try_schema_command::try_schema_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity)
{
}
bool try_schema_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("try_schema_command - Got try schema instruction.");
    logger().debug("try_schema_command - Args: \n%s",
                 arg->toString().c_str());
    logger().debug("try_schema_command - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    vector<string> f_args;
    FormatArgs(arg->out, arity - 1, back_inserter(f_args));

    if (f_args.empty() && !petInterface.isInLearningMode()) {
        logger().warn("try_schema_command - Schema name should be provided.");
        return false;
    }
    //printf("Try schema: %s\n", f_args[0].c_str());
    if (f_args.size() > 0) {
        vector<string>::iterator args = f_args.begin() + 1;
        while (args != f_args.end()) {
            *args = nameResolver->solve(*args, petInterface.getOwnerId(), timestamp);
            args++;
        }
    }


    petInterface.trySchema(f_args, timestamp);

    return true;
}

pet_meta_command::pet_meta_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity) {}
bool pet_meta_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("pet_meta_command - meta command instruction.");
    logger().debug("pet_meta_command- Args: \n%s",
                 arg->toString().c_str());
    logger().debug("pet_meta_command - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    return true;
}

pet_interrogative_command::pet_interrogative_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity) {}
bool pet_interrogative_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("pet_interrogative_command - question instruction.");
    logger().debug("pet_interrogative_command- Args: \n%s",
                 arg->toString().c_str());
    logger().debug("pet_interrogative_command - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    return true;
}

pet_declarative_command::pet_declarative_command(PetInterface& petInterface, int _arity) : action(petInterface), arity(_arity) {}
bool pet_declarative_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("pet_declarative_command - declarative instruction.");
    logger().debug("pet_declarative_command- Args: \n%s",
                 arg->toString().c_str());
    logger().debug("pet_declarative_command - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    return true;
}

any_command::any_command(PetInterface& petInterface) : action(petInterface) {}
bool any_command::operator()(const pAtom& arg, unsigned long timestamp, const std::string& agentIdWhichSentTheCommand ) const
{

    logger().info("any_command - Dummy instruction (should not be invoked).");
    logger().debug("any_command - Args: \n%s",
                 arg->toString().c_str());
    logger().debug("any_command - petId = %s, timestamp = %lu",
                 petInterface.getPetId().c_str(), timestamp);

    return true;
}
