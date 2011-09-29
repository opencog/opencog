/*
 * opencog/embodiment/Control/OperationalAvatarController/RuleEngineLearnedTricksHandler.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
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

#include "RuleEngineLearnedTricksHandler.h"
#include <opencog/util/mt19937ar.h>

#include <limits>

using namespace opencog::oac;

const std::string RuleEngineLearnedTricksHandler::LEARNED_TRICK_NODE_NAME = "learnedTrick";
const int RuleEngineLearnedTricksHandler::REWARD_VALUE = 80;
const int RuleEngineLearnedTricksHandler::PUNISHMENT_VALUE = -50;
const int RuleEngineLearnedTricksHandler::NOT_SELECTED_VALUE = 2;

RuleEngineLearnedTricksHandler::RuleEngineLearnedTricksHandler( OAC* oac ) : oac( oac ),
        atomSpace( oac->getAtomSpace( ) ), latestSelectedTrick( "" ), numberOfLearnedTricks( 0 )
{

    unsigned long randSeed;
    if (opencog::config().get_bool("AUTOMATED_SYSTEM_TESTS")) {
        randSeed = 0;
    } else {
        randSeed = time(NULL);
    } // else
    this->randGen = new opencog::MT19937RandGen( randSeed );

    // create a concept node for LEARNED_TRICK
    this->learnedTrickNode = AtomSpaceUtil::addNode(*atomSpace, CONCEPT_NODE, LEARNED_TRICK_NODE_NAME, true);
}

RuleEngineLearnedTricksHandler::~RuleEngineLearnedTricksHandler( )
{
    delete this->randGen;
}

void RuleEngineLearnedTricksHandler::addLearnedSchema( const std::string& schemaName )
{
    Handle schema = AtomSpaceUtil::addNode(*atomSpace, GROUNDED_SCHEMA_NODE, schemaName, true);

    HandleSeq inheritance;
    inheritance.push_back( this->learnedTrickNode );
    inheritance.push_back( schema );

    Handle inheritanceLink = AtomSpaceUtil::addLink(*atomSpace, INHERITANCE_LINK, inheritance, true);

    // TODO: Since LTI is greater than 0, setting STI to a big value is not needed anymore. Anyway, this must be reviewed later, in case learned tricks can be forgotten.
    // 32767 = max short value
    //this->atomSpace->setSTI( inheritanceLink, std::numeric_limits<short>::max() );
}

void RuleEngineLearnedTricksHandler::selectLearnedTrick( std::string& schemaName, std::set<std::string>& arguments ) throw(opencog::NotFoundException)
{

    HandleSeq inheritance;
    inheritance.push_back( this->learnedTrickNode );
    inheritance.push_back( Handle::UNDEFINED );


    // find all learned tricks and put it in order of STI values
    std::vector<Handle> allSchemaHandles;
    this->atomSpace->getHandleSet( back_inserter( allSchemaHandles ), inheritance, NULL, NULL, 2, INHERITANCE_LINK, false );

    std::multimap<AttentionValue::sti_t, Handle> stiHandles;
    foreach( Handle linkHandle, allSchemaHandles ) {
        AttentionValue::sti_t sti = this->atomSpace->getSTI( linkHandle );
        stiHandles.insert( std::pair<AttentionValue::sti_t, Handle>( sti, this->atomSpace->getOutgoing(linkHandle, 1) ) );
    } // foreach

    if ( stiHandles.size( ) == 0 ) {
        this->latestSelectedTrick = "";
        throw opencog::NotFoundException( TRACE_INFO,
                                          "RuleEngineLearnedTricksHandler - There is no learned tricks at this moment" );
    } // if

    // select a random trick of the top 5
    int index = stiHandles.size( ) > 0 ? ( randGen->randint( ) % std::min( (size_t)5, stiHandles.size( ) ) ) : 0;

    std::multimap<AttentionValue::sti_t, Handle>::iterator it = stiHandles.begin( );
    while ( index > 0 ) {
        ++it;
        --index;
    } // while

    schemaName = this->atomSpace->getName(it->second);
    this->latestSelectedTrick = schemaName;
    logger().debug("RuleEngineLearnedTricksHandler - selected schema %s ", schemaName.c_str( ) );
    // TODO: restore parameters
}

void RuleEngineLearnedTricksHandler::rewardSchema( std::string& schemaName )
{
    Handle schema = this->atomSpace->getHandle( GROUNDED_SCHEMA_NODE, schemaName );
    if ( schema != Handle::UNDEFINED ) {
        HandleSeq inheritance;
        inheritance.push_back( this->learnedTrickNode );
        inheritance.push_back( schema );

        Handle link = this->atomSpace->getHandle( INHERITANCE_LINK, inheritance );

        if ( link != Handle::UNDEFINED ) {
            addToSTIValue( link, REWARD_VALUE );
            // now the given schema isn't a punished trick
            std::set<Handle>::iterator it = this->punishedTricks.find( link );
            if ( it != punishedTricks.end( ) ) {
                punishedTricks.erase( it );
            } // if
            logger().debug("RuleEngineLearnedTricksHandler - rewarded schema: %s | current sti: %d ", schemaName.c_str( ), this->atomSpace->getSTI( link ) );
            return;
        } // if
    } // if
    logger().error("RuleEngineLearnedTricksHandler - there is no schema named %s to reward", schemaName.c_str( ) );

}

void RuleEngineLearnedTricksHandler::punishSchema( std::string& schemaName )
{
    Handle schema = this->atomSpace->getHandle( GROUNDED_SCHEMA_NODE, schemaName );
    if ( schema != Handle::UNDEFINED ) {
        HandleSeq inheritance;
        inheritance.push_back( this->learnedTrickNode );
        inheritance.push_back( schema );

        Handle link = this->atomSpace->getHandle( INHERITANCE_LINK, inheritance );

        if ( link != Handle::UNDEFINED ) {
            addToSTIValue( link, PUNISHMENT_VALUE );

            punishedTricks.insert( link );

            logger().debug("RuleEngineLearnedTricksHandler - punished schema: %s | current sti: %d ", schemaName.c_str( ), this->atomSpace->getSTI( link ) );
            return;
        } // if
    } // if
    logger().error("RuleEngineLearnedTricksHandler - there is no schema named %s to punish", schemaName.c_str( ) );
}

void RuleEngineLearnedTricksHandler::update( void )
{


    // reward or punish the latest selected trick if it was executed
    // until 4 seconds passed and the owner gave a feedback
    if ( this->latestSelectedTrick.length( ) > 0 ) {
        unsigned long rewardTimestamp = this->oac->getPet( ).getLatestRewardTimestamp( );
        unsigned long punishmentTimestamp = this->oac->getPet( ).getLatestPunishmentTimestamp( );
        unsigned long currentTimestamp = this->oac->getPAI( ).getLatestSimWorldTimestamp( );
        bool reward = false;
        bool punish = false;

        logger().debug("RuleEngineLearnedTricksHandler - Timestamps: current(%ld) reward(%ld) punishment(%ld)", currentTimestamp, rewardTimestamp, punishmentTimestamp );

        if ( rewardTimestamp != Pet::UNDEFINED_TIMESTAMP &&
                ( currentTimestamp - rewardTimestamp ) < 5000 ) { // 5 seconds
            reward = true;
        } // if
        if ( punishmentTimestamp != Pet::UNDEFINED_TIMESTAMP &&
                ( currentTimestamp - punishmentTimestamp ) < 5000 ) { // 5 seconds
            if ( !reward || rewardTimestamp < punishmentTimestamp ) {
                punish = true;
            } // if
        } // if

        if ( reward ) {
            rewardSchema( this->latestSelectedTrick );
        } else if ( punish ) {
            punishSchema( this->latestSelectedTrick );
        } // else if

        this->latestSelectedTrick = "";
    } // if

    HandleSeq inheritance;
    inheritance.push_back( this->learnedTrickNode );
    inheritance.push_back( Handle::UNDEFINED );

    // find all learned tricks
    std::vector<Handle> allLinkHandles;
    this->atomSpace->getHandleSet( back_inserter( allLinkHandles ), inheritance, NULL, NULL, 2, INHERITANCE_LINK, false );

    this->numberOfLearnedTricks = 0;
    foreach( Handle linkHandle, allLinkHandles ) {

        Handle schemaHandle = atomSpace->getOutgoing(linkHandle, 1);
        if ( schemaHandle == Handle::UNDEFINED ) {
            logger().error("RuleEngineLearnedTricksHandler - undefined schema" );
            continue;
        } // if

        std::set<Handle>::iterator it = this->punishedTricks.find( schemaHandle );
        if ( it == punishedTricks.end( ) &&  this->atomSpace->getName(schemaHandle) != latestSelectedTrick ) {
            addToSTIValue( linkHandle, NOT_SELECTED_VALUE );

            logger().debug("RuleEngineLearnedTricksHandler - updated schema: %s | current sti: %d ",  this->atomSpace->getName(schemaHandle).c_str( ), this->atomSpace->getSTI( linkHandle ) );
        } // if
        if ( this->atomSpace->getSTI( linkHandle ) > opencog::config().get_int("MIN_STI") ) {
            ++this->numberOfLearnedTricks;
        } // if
    } // foreach
}

bool RuleEngineLearnedTricksHandler::hasLearnedTricks( void )
{
    return ( this->numberOfLearnedTricks > 0 );
}

void RuleEngineLearnedTricksHandler::addToSTIValue( Handle link, short value )
{
    int currentSTIValue = static_cast<int>( this->atomSpace->getSTI( link ) );
    currentSTIValue += REWARD_VALUE;

    short newSTIValue = static_cast<short>( std::max( std::min( currentSTIValue, static_cast<int>( std::numeric_limits<short>::max( ) ) ), static_cast<int>( std::numeric_limits<short>::min( ) ) ) );
    this->atomSpace->setSTI( link, newSTIValue );
}
