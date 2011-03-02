/*
 * opencog/embodiment/Control/PredicateUpdaters/NearPredicateUpdater.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari Heljakka, Welter Luigi, Samir Araujo
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


#include <opencog/atomspace/SimpleTruthValue.h>

#include "NearPredicateUpdater.h"
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>

using namespace OperationalAvatarController;
using namespace opencog;

NearPredicateUpdater::NearPredicateUpdater(AtomSpace &_atomSpace) :
        BasicPredicateUpdater(_atomSpace) {}

NearPredicateUpdater::~NearPredicateUpdater()
{
}

void NearPredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp )
{
    // there is no map, no update is possible
    Handle spaceMapHandle = atomSpace.getSpaceServer().getLatestMapHandle();
    if (spaceMapHandle == Handle::UNDEFINED) {
        logger().warn( "NearPredicateUpdater::%s - No space map handle found!", __FUNCTION__);
        return;
    }
    const SpaceServer::SpaceMap& spaceMap = atomSpace.getSpaceServer().getLatestMap();

    std::vector<std::string> entities;
    spaceMap.findAllEntities(back_inserter(entities));

    logger().debug( "NearPredicateUpdater::%s - Processing timestamp '%lu'",
            __FUNCTION__, timestamp );
    if ( lastTimestamp != timestamp ) {
        lastTimestamp = timestamp;
        processedEntities.clear( );
    } // if
    
    const std::string& entityAId = atomSpace.getName( object );
    if ( processedEntities.find( entityAId ) != processedEntities.end( ) ) {
        return;
    } // if
    processedEntities.insert( entityAId );

    try {

        const spatial::EntityPtr& entityA = spaceMap.getEntity( entityAId );
    
        bool mapContainsEntity = spaceMap.containsObject( entityAId );

        for( unsigned int i = 0; i < entities.size( ); ++i ) {        
            const std::string& entityBId = entities[i];
            if ( processedEntities.find( entityBId ) != processedEntities.end( ) ) {
                continue;
            } // if
            Handle entityBHandle = getHandle( entityBId );

            if ( !mapContainsEntity ) {
                logger().debug( "NearPredicateUpdater::%s - Removing predicates "
                        "from '%s' and '%s'", __FUNCTION__,
                        entityAId.c_str( ), entityBId.c_str( ) );
                setPredicate( object, entityBHandle, "near", 0.0f );
                setPredicate( object, entityBHandle, "next", 0.0f );
            } else {
                const spatial::EntityPtr& entityB = spaceMap.getEntity( entityBId );
                double distance = entityA->distanceTo( *entityB );
                logger().debug( "NearPredicateUpdater::%s - Adding predicates "
                        "for '%s' and '%s'. distance '%f'", __FUNCTION__,
                        entityAId.c_str( ), entityBId.c_str( ), distance );

                spatial::math::Vector3 minCorner( spaceMap.xMin( ), spaceMap.yMin( ) );
                spatial::math::Vector3 maxCorner( spaceMap.xMax( ), spaceMap.yMax( ) );
            
                double mapDiagonal = ( maxCorner - minCorner ).length( );
            
                double nearDistance = spaceMap.getNearDistance( );
                double nextDistance = spaceMap.getNextDistance( );
            
                logger().debug( "NearPredicateUpdater::%s - nearDistance '%f'",
                                __FUNCTION__, nearDistance );

                setPredicate( object, entityBHandle, "near",
                        ( distance < nearDistance ) ? 1.0 : 0.0f );
                setPredicate( object, entityBHandle, "next",
                        ( distance < nextDistance ) ? 1.0 - (distance/nextDistance) : 0.0f );

                SimpleTruthValue tv(1.0 - (distance/mapDiagonal), 1);
                AtomSpaceUtil::setPredicateValue( atomSpace, "proximity", tv,
                        object, entityBHandle );
                AtomSpaceUtil::setPredicateValue( atomSpace, "proximity", tv,
                        entityBHandle, object );
            
            } // else
        } // for
    } catch( const opencog::NotFoundException& ex ) {
        logger().error( "NearPredicateUpdater::%s - Entity not found '%s'",
                        __FUNCTION__, ex.getMessage( ) );
    } // catch
        
}

void NearPredicateUpdater::setPredicate( const Handle& entityA, const Handle& entityB, const std::string& predicateName, float mean )
{
    SimpleTruthValue tv( mean, 1 );
    AtomSpaceUtil::setPredicateValue( atomSpace, predicateName, tv, entityA, entityB );
    AtomSpaceUtil::setPredicateValue( atomSpace, predicateName, tv, entityB, entityA );
}
