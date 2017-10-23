/*
 * opencog/spatial/SuperEntity.cc
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

#include <map>

#include <opencog/spatial/SuperEntity.h>
#include <opencog/util/Logger.h>

using namespace opencog;
using namespace opencog::spatial;

SuperEntity::SuperEntity( const EntityPtr& entity1, const EntityPtr& entity2 )
{
    boost::shared_ptr<SubEntity> subEntity1( createSubEntity(entity1) );
    boost::shared_ptr<SubEntity> subEntity2( createSubEntity(entity2) );

    this->subEntities.insert( LongSubEntityPtrHashMap::value_type( subEntity1->id, subEntity1 ) );
    this->subEntities.insert( LongSubEntityPtrHashMap::value_type( subEntity2->id, subEntity2 ) );

    if ( !rebuild( ) ) {
        this->segments.clear( );
        this->subEntities.clear( );
        logger().error("Entities doesn't intersect: Entity1(%s) Entity2(%s). An exception will be thrown", entity1->toString( ).c_str( ), entity2->toString( ).c_str( ) );
        throw opencog::InvalidParamException( TRACE_INFO, "Given entities does not intersect" );
    } // if
}

SuperEntity::~SuperEntity( void )
{
}

bool SuperEntity::splitEdges( const SubEntityPtr& subEntity1, const SubEntityPtr& subEntity2 )
{

    bool intersects = false;

    // check if there are edges of each entities that overlaps
    unsigned int i;
    unsigned int j;
    for ( i = 0; i < 4; ++i ) {
        std::list<math::LineSegment>& edges1 = subEntity1->splitEdges[i];
        std::list<math::LineSegment>::iterator it1 = edges1.begin( );

        while ( it1 != edges1.end( ) ) {
            bool edgeSplit = false;

            for ( j = 0; j < 4; ++j ) {
                std::list<math::LineSegment>& edges2 = subEntity2->splitEdges[j];
                std::list<math::LineSegment>::iterator it2 = edges2.begin( );
                while ( it2 != edges2.end( ) ) {

                    math::Vector3 pointInA;
                    math::Vector3 pointInB;
                    double distance = it1->distanceTo( *it2, &pointInA, &pointInB);

                    if ( distance <= math::LineSegment::TOLERANCE_DISTANCE ) {
                        // distance lesser than threshold, so two edges intersects each other

                        // check if the intersection is just the begin or end points intersection
                        bool needSplitEdge1 = !it1->sharePoint( pointInA );
                        bool needSplitEdge2 = !it2->sharePoint( pointInB );

                        if ( needSplitEdge1 ) {
                            // remove the original edge and insert the two fragments into the edges list
                            std::list<math::LineSegment>::iterator it3 = it1; ++it3;

                            math::LineSegment segA( it1->pointA, pointInA );
                            math::LineSegment segB( pointInA, it1->pointB );

                            edges1.erase( it1 ); it1 = it3;

                            if ( !subEntity2->rectangle.isInside( segB.getMidPoint( ) ) ) {
                                it1 = edges1.insert( it1, segB );
                            } // if

                            if ( !subEntity2->rectangle.isInside( segA.getMidPoint( ) ) ) {
                                it1 = edges1.insert( it1, segA );
                            } // if

                            intersects = true;
                            edgeSplit = true;
                        } // if

                        if ( needSplitEdge2 ) {
                            // remove the original edge and insert the two fragments into the edges list
                            std::list<math::LineSegment>::iterator it3 = it2; ++it3;

                            math::LineSegment segA( it2->pointA, pointInB );
                            math::LineSegment segB( pointInB, it2->pointB );

                            edges2.erase( it2 ); it2 = it3;

                            if ( !subEntity1->rectangle.isInside( segB.getMidPoint( ) ) ) {
                                it3 = edges2.insert( it3, segB );
                            } // if

                            if ( !subEntity1->rectangle.isInside( segA.getMidPoint( ) ) ) {
                                it3 = edges2.insert( it3, segA );
                            } // if

                            intersects = true;
                            continue;
                        } // if

                    } else if ( subEntity1->rectangle.isInside( it2->pointA) &&
                                subEntity1->rectangle.isInside( it2->pointB) ) {
                        // if the edge of the second entity is inside of the first entity remove it from the list
                        std::list<math::LineSegment>::iterator it3 = it2; ++it3;
                        edges2.erase( it2 );
                        it2 = it3;
                        intersects = true;
                        continue;
                    } // if

                    ++it2;
                } // while
            } // for

            if ( edgeSplit ) {
                continue;
            } else if ( subEntity2->rectangle.isInside( it1->pointA ) &&
                        subEntity2->rectangle.isInside( it1->pointB ) ) {
                // if the edge of the first entity is inside of the second entity remove it from the list
                std::list<math::LineSegment>::iterator it3 = it1; ++it3;
                edges1.erase( it1 );
                it1 = it3;
                intersects = true;
                continue;
            } // if

            ++it1;
        } // while
    } // for
    return intersects;
}

bool SuperEntity::rebuild( void )
{

    // remove all the computed segments
    this->segments.clear( );

    LongSubEntityPtrHashMap::iterator it1;
    LongSubEntityPtrHashMap::iterator it2;

    std::map<long, bool> intersectionMap;

    // reseting split edges from all sub entities
    for ( it1 = subEntities.begin( ); it1 != subEntities.end( ); ++it1 ) {
        it1->second->reset( );
        intersectionMap[it1->first] = false;
    } // for

    // compute the intersection between each of the entities of this superentity
    for ( it1 = subEntities.begin( ); it1 != subEntities.end( ); ++it1 ) {
        for ( it2 = it1, ++it2; it2 != subEntities.end( ); ++it2 ) {
            if ( splitEdges( it1->second, it2->second ) ) {
                // the entities have their edges split
                intersectionMap[ it1->first ] = true;
                intersectionMap[ it2->first ] = true;
            } else {
                // check if one entity is inside another
                std::list<math::LineSegment>::iterator it3;

                bool firstInsideSecond = true;
                bool secondInsideFirst = true;

                for ( it3 = it2->second->edges.begin( ); it3 != it2->second->edges.end( ); ++it3 ) {
                    if ( !it1->second->rectangle.isInside( it3->pointA ) ) {
                        secondInsideFirst = false;
                        break;
                    } // if
                } // if

                if ( secondInsideFirst ) {
                    // ok, second entity is completely inside the first
                    unsigned int i;
                    for ( i = 0; i < 4; ++i ) {
                        it2->second->splitEdges[i].clear( );
                    } // for
                    intersectionMap[ it1->first ] = true;
                    intersectionMap[ it2->first ] = true;
                } else {
                    // no, the second entity is'n inside the first, so check the opposite
                    for ( it3 = it1->second->edges.begin( ); it3 != it1->second->edges.end( ); ++it3 ) {
                        if ( !it2->second->rectangle.isInside( it3->pointA ) ) {
                            firstInsideSecond = false;
                            break;
                        } // if
                    } // if

                    if ( firstInsideSecond ) {
                        // ok, first entity is completely inside the second
                        unsigned int i;
                        for ( i = 0; i < 4; ++i ) {
                            it1->second->splitEdges[i].clear( );
                        } // for
                        intersectionMap[ it1->first ] = true;
                        intersectionMap[ it2->first ] = true;
                    } else {
                        // no, the first entity is'n inside the second, so check whether at least one corner
                        // overlaps
                        std::list<math::LineSegment>::iterator it4;

                        int overlapCorners = 0;
                        // corner overlap counter
                        for ( it3 = it2->second->edges.begin( ); it3 != it2->second->edges.end( ); ++it3 ) {
                            for ( it4 = it1->second->edges.begin( ); it4 != it1->second->edges.end( ); ++it4 ) {
                                // if ( it3->pointA == it4->pointA ) {
                                if ( (it3->pointA - it4->pointA).length() < math::LineSegment::TOLERANCE_DISTANCE ) {
                                    ++overlapCorners;
                                } // if
                            } // for
                        } // for

                        if ( overlapCorners == 4 ) { // they completely overlap
                            unsigned int i;
                            for ( i = 0; i < 4; ++i ) {
                                it1->second->splitEdges[i].clear( );
                            } // for
                            intersectionMap[ it1->first ] = true;
                            intersectionMap[ it2->first ] = true;
                        } else if ( overlapCorners == 1 ) { // just one corner overlap
                            intersectionMap[ it1->first ] = true;
                            intersectionMap[ it2->first ] = true;
                        } // else if

                    } // else
                } // else

            } // if

        } // for
    } // for

    // copying all edges to the main segment list
    std::back_insert_iterator<std::list<math::LineSegment> > ii( this->segments );
    for ( it1 = subEntities.begin( ); it1 != subEntities.end( ); ++it1 ) {
        std::list<math::LineSegment> splitEdges = it1->second->getSplitEdges( );
        std::copy( splitEdges.begin( ), splitEdges.end( ), ii );
    } // for


    std::map<long, bool>::iterator it;
    for ( it = intersectionMap.begin( ); it != intersectionMap.end( ); ++it ) {
        if ( !it->second ) {
            return false;
        } // if
    } // for
    return true;

} // rebuild

std::list<math::Vector3> SuperEntity::getCorners( void ) const
{
    std::list<math::Vector3> points;
    std::list<math::LineSegment> edges = this->segments;
    std::list<math::LineSegment>::iterator it = edges.begin( );

    points.push_back( it->pointA );
    points.push_back( it->pointB );

    math::Vector3 current = points.back( );

    while ( it != edges.end( ) ) {
        if ( ( current - it->pointA ).length( ) <= math::LineSegment::TOLERANCE_DISTANCE ) {
            points.push_back( it->pointB );
            current = points.back( );
            edges.erase( it );
            it = edges.begin( );
        } else if ( ( current - it->pointB ).length( ) <= math::LineSegment::TOLERANCE_DISTANCE ) {
            points.push_back( it->pointA );
            current = points.back( );
            edges.erase( it );
            it = edges.begin( );
        } else {
            ++it;
        } // else
    } // while
    return points;
}

bool SuperEntity::merge( const EntityPtr& entity )
{
    if ( containsEntity( entity->getId( ) ) ) {
        return true;
    } // if

    this->subEntities[ entity->getId( ) ] = SubEntityPtr( createSubEntity( entity ) );
    bool result = rebuild( );
    if ( !result ) {
        this->subEntities.erase( entity->getId( ) );
        rebuild( );
    } // if
    return result;
}

bool SuperEntity::merge( const SuperEntityPtr& entity )
{
    LongSubEntityPtrHashMap::const_iterator it;
    for ( it = entity->subEntities.begin( ); it != entity->subEntities.end( ); ++it ) {
        if ( !containsEntity( it->first ) ) {
            subEntities[it->first] = it->second;
        } // if
    } // for

    return rebuild( );
}

bool SuperEntity::containsEntity( long id ) const
{
    return ( this->subEntities.find( id ) != this->subEntities.end( ) );
}

void SuperEntity::removeEntity( long id )
{
    LongSubEntityPtrHashMap::iterator it = this->subEntities.find( id );
    if ( it != this->subEntities.end( ) ) {
        if ( this->subEntities.size( ) == 2 ) {
            throw opencog::InvalidParamException( TRACE_INFO, "SuperEntity::removeEntity - Cannot remove a subEntity from a superEntity containing just two entities" );
        } // if

        this->subEntities.erase( it );

        if ( !rebuild( ) ) {
            throw opencog::InvalidParamException( TRACE_INFO, "SuperEntity::removeEntity - The removed subEntity invalidate this superEntity" );
        } // if
    } // if
}

SuperEntityPtr SuperEntity::clone( void ) const
{
    SuperEntityPtr clone( new SuperEntity( ) );
    clone->segments = this->segments;
    LongSubEntityPtrHashMap::const_iterator it;
    for ( it = this->subEntities.begin( ); it != this->subEntities.end( ); ++it ) {
        clone->subEntities.insert( LongSubEntityPtrHashMap::value_type( it->first, SubEntityPtr( it->second ) ) );
    } // for
    return clone;
}

SuperEntity::SubEntityPtr SuperEntity::createSubEntity( const EntityPtr& entity )
{
    // compute the edges of the entities bases
    const math::Vector3& p11 = entity->getBoundingBox().getCorner( math::BoundingBox::NEAR_LEFT_BOTTOM );
    const math::Vector3& p12 = entity->getBoundingBox().getCorner( math::BoundingBox::FAR_LEFT_BOTTOM );
    const math::Vector3& p13 = entity->getBoundingBox().getCorner( math::BoundingBox::FAR_RIGHT_BOTTOM );
    const math::Vector3& p14 = entity->getBoundingBox().getCorner( math::BoundingBox::NEAR_RIGHT_BOTTOM );


    std::list<math::LineSegment> edges1;
    edges1.push_back( math::LineSegment(p11, p12) );
    edges1.push_back( math::LineSegment(p12, p13) );
    edges1.push_back( math::LineSegment(p13, p14) );
    edges1.push_back( math::LineSegment(p14, p11) );
    return SubEntityPtr( new SubEntity( entity->getId(), math::Rectangle( p11, p12, p13 ), edges1 ) );
}

bool SuperEntity::isInside( const math::Vector3& point ) const
{
    LongSubEntityPtrHashMap::const_iterator it;
    for ( it = this->subEntities.begin(); it != this->subEntities.end( ); ++it ) {
        if ( it->second->rectangle.isInside(point)) {
            return true;
        } // if
    } // for
    return false;
}

std::vector<long> SuperEntity::getSubEntitiesIds( void ) const
{
    std::vector<long> ids;
    LongSubEntityPtrHashMap::const_iterator it;
    for ( it = this->subEntities.begin(); it != this->subEntities.end( ); ++it ) {
        ids.push_back( it->first );
    } // for
    return ids;
}
