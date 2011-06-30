/*
 * opencog/embodiment/Control/OperationalAvatarController/RuleEngineUtil.cc
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

#include <lua.hpp>
#include <luabind/luabind.hpp>

#include "OAC.h"
#include "SchemaRunner.h"
#include "RuleEngine.h"
#include "RuleEngineUtil.h"
#include "RuleEngineLearnedTricksHandler.h"

using namespace opencog::oac;

RuleEngineUtil::RuleEngineUtil( RuleEngine* ruleEngine )
        : ruleEngine( ruleEngine )
{
    this->cyclesDuringNovelty = config().get_int("RE_CYCLES_DURING_NOVELTY");
}

RuleEngineUtil::HandleContainer RuleEngineUtil::getNovelEntityHandleSet( void )
{
    HandleContainer nohs = getNovelObjectHandleSet();
    HandleContainer nahs = getNovelAgentHandleSet();

    std::set_union(nohs.begin(), nohs.end(), nahs.begin(), nahs.end(),
                   insert_iterator<HandleContainer>(nahs, nahs.begin()));
    return nahs;
}

RuleEngineUtil::HandleContainer RuleEngineUtil::getNovelObjectHandleSet( void )
{
    HandleContainer res;
    const AtomSpace& as = *(ruleEngine->opc->getAtomSpace());
    RuleEngine::Id_EntityPerception_Map_Const_It it;
    for ( it = this->ruleEngine->objects.begin( );
            it != this->ruleEngine->objects.end( ); ++it ) {
        //check that the object has not been seen or not for long
        if ( it->second.getFirstSeenCycle( )
                >= (this->ruleEngine->cycle - cyclesDuringNovelty) ) {
            //insert if so
            res.insert(AtomSpaceUtil::getObjectHandle(as, it->first));
        } // if
    } // for
    return res;
}

RuleEngineUtil::HandleContainer RuleEngineUtil::getNovelAgentHandleSet( void )
{
    HandleContainer res;
    const AtomSpace& as = *(ruleEngine->opc->getAtomSpace());
    RuleEngine::Id_EntityPerception_Map_Const_It it;
    for ( it = this->ruleEngine->avatars.begin( );
            it != this->ruleEngine->avatars.end( ); ++it ) {
        //check that the object has not been seen or not for long
        if ( it->second.getFirstSeenCycle( )
                >= (this->ruleEngine->cycle - cyclesDuringNovelty) ) {
            //insert if so
            res.insert(AtomSpaceUtil::getAgentHandle(as, it->first));
        } // if
    } // for
    return res;
}

bool RuleEngineUtil::isNovelty( void )
{
    return ( isAvatarNovelty( ) || isObjectNovelty( ) );
}

bool RuleEngineUtil::isAvatarNovelty( void )
{
    RuleEngine::Id_EntityPerception_Map_Const_It it;
    for ( it = this->ruleEngine->avatars.begin( );
            it != this->ruleEngine->avatars.end( ); ++it ) {
        if ( it->second.getFirstSeenCycle( )
                >= (this->ruleEngine->cycle - cyclesDuringNovelty) ) {
            return true;
        } // if
    } // for
    return false;
}

bool RuleEngineUtil::isObjectNovelty( void )
{

    RuleEngine::Id_EntityPerception_Map_Const_It it;
    for ( it = this->ruleEngine->objects.begin( );
            it != this->ruleEngine->objects.end( ); ++it ) {
        if ( it->second.getFirstSeenCycle( )
                >= (this->ruleEngine->cycle - cyclesDuringNovelty) ) {
            return true;
        } // if
    } // for
    return false;
}

bool RuleEngineUtil::isThereARequestedSchema( void )
{

    if ( this->ruleEngine->opc->getPet( ).isRequestedCommandNotReaded( ) ) {
        this->ruleEngine->lastRequestedCommandCycles = config().get_int("RE_CYCLES_FOR_REQUESTED_SCHEMA");
        return true;
    } else if ( this->ruleEngine->lastRequestedCommandCycles > 0 ) {
        --this->ruleEngine->lastRequestedCommandCycles;
        return true;
    } // else
    return false;
}
