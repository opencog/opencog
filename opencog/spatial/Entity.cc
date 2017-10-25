/*
 * opencog/spatial/Entity.cc
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

#include <sstream>
#include <cstring>
#include <cassert>

#include <boost/type_traits/remove_cv.hpp>
#include <boost/variant/get.hpp>

#include <opencog/spatial/Entity.h>
#include <opencog/spatial/LocalSpaceMap2D.h>
#include <opencog/util/Logger.h>

using namespace opencog;
using namespace opencog::spatial;

/**
 * Helper classes used to compare two points
 * by using an specific axis
 */
class SortByAxisX {
public:
    inline bool operator()( const math::Vector3& p1, const math::Vector3& p2 ) const {
        return p1.x < p2.x;
    }
};
class SortByAxisY {
public:
    inline bool operator()( const math::Vector3& p1, const math::Vector3& p2 ) const {
        return p1.y < p2.y;
    }
};
class SortByAxisZ {
public:
    inline bool operator()( const math::Vector3& p1, const math::Vector3& p2 ) const {
        return p1.z < p2.z;
    }
};

Entity::Entity( const EntityPtr& entity ) : 
    id(entity->id), name(entity->name), dimension(entity->dimension),
    position(entity->position), orientation(entity->orientation), 
    expansionRadius(entity->expansionRadius), boundingBox(this)
{
    //this->properties.resize( Entity::NUMBER_OF_PROPERTIES );
}

Entity::Entity( long id, const std::string& name, const math::Vector3& position, 
                const math::Dimension3& dimension, const math::Quaternion& orientation, 
                double radius ) :
                    id(id), name(name), dimension(dimension), position(position),
                    orientation(orientation), expansionRadius( radius ), boundingBox(this) {}

math::Vector3 Entity::getDirection( void ) const
{
    return orientation.rotate( math::Vector3::X_UNIT );
}

double Entity::distanceTo( const Entity& entity,
                           math::Vector3* pointInA,
                           math::Vector3* pointInB,
                           LimitRelation* status ) const
{

    /* TODO: This method is not precise. It does an approximation of the real
     * distance between the entities comparing the distance of the edges of the
     * objects. It would be better if the three cases bellow were handle by this method:
     * 1) corner vs corner
     * 2) corner vs face
     * 3) face vs face
     */

    const math::BoundingBox& bb1 = getBoundingBox( );
    const math::BoundingBox& bb2 = entity.getBoundingBox( );

    LimitRelation localStatus = computeObjectsLimits( entity );

    if ( status != NULL ) {
        *status = localStatus;
    }
    
    std::list<unsigned int> bordersOfA;
    std::list<unsigned int> bordersOfB;

    bool completelyOverlap = true;
    if ( ( localStatus.relations[LimitRelation::X] & (1|4|16) ) > 0 ) {
        // get the most right point of A and most left point of B
        bordersOfA.push_back( XMAX );
        bordersOfB.push_back( XMIN );
        completelyOverlap = false;
    }
    else if ( ( localStatus.relations[LimitRelation::X] & (2|8|32) ) > 0 ) {
        // get the most left point of A and most right point of B
        bordersOfA.push_back( XMIN );
        bordersOfB.push_back( XMAX );
        completelyOverlap = false;
    }

    if ( ( localStatus.relations[LimitRelation::Y] & (1|4|16) ) > 0 ) {
        // get the most right point of A and most left point of B
        bordersOfA.push_back( YMAX );
        bordersOfB.push_back( YMIN );
        completelyOverlap = false;
    }
    else if ( ( localStatus.relations[LimitRelation::Y] & (2|8|32) ) > 0 ) {
        // get the most left point of A and most right point of B
        bordersOfA.push_back( YMIN );
        bordersOfB.push_back( YMAX );
        completelyOverlap = false;
    }

    if ( ( localStatus.relations[LimitRelation::Z] & (1|4|16) ) > 0 ) {
        // get the most right point of A and most left point of B
        bordersOfA.push_back( ZMAX );
        bordersOfB.push_back( ZMIN );
        completelyOverlap = false;
    } 
    else if ( ( localStatus.relations[LimitRelation::Z] & (2|8|32) ) > 0 ) {
        // get the most left point of A and most right point of B
        bordersOfA.push_back( ZMIN );
        bordersOfB.push_back( ZMAX );
        completelyOverlap = false;
    }

    // If objects contains each other in all dimensions, the distance is zero
    if ( completelyOverlap ) {
        return 0.0;
    }
    
    std::set<math::Vector3> entityPointsA, entityPointsB;
    { // get the points in object which has a greatest score
        std::list<unsigned int>::const_iterator it;
        std::map<math::Vector3, unsigned int> pointCounterA, pointCounterB;

        unsigned int counter = 0;
        for( it = bordersOfA.begin( ); it != bordersOfA.end( ); ++it ) {
            std::set<math::Vector3>::const_iterator itPoints;
            for( itPoints = localStatus.limitsA[*it].begin( );
                 itPoints != localStatus.limitsA[*it].end( ); ++itPoints ) {
                unsigned int& counterA = pointCounterA[*itPoints];
                ++counterA;
                if ( counterA > counter ) {
                    counter = counterA;
                    entityPointsA.clear( );
                    entityPointsA.insert(*itPoints);
                } else if ( counterA == counter ) {
                    entityPointsA.insert(*itPoints);
                } // else if 
            } // for
        } // for

        counter = 0;
        for( it = bordersOfB.begin( ); it != bordersOfB.end( ); ++it ) {
            std::set<math::Vector3>::const_iterator itPoints;
            for( itPoints = localStatus.limitsB[*it].begin( ); 
                 itPoints != localStatus.limitsB[*it].end( ); ++itPoints ) {
                unsigned int& counterB = pointCounterB[*itPoints];
                ++counterB;
                if ( counterB > counter ) {
                    counter = counterB;
                    entityPointsB.clear( );
                    entityPointsB.insert(*itPoints);
                } else if ( counterB == counter ) {
                    entityPointsB.insert(*itPoints);
                } // else if 

            } // for
        } // for        
    } // end block

    std::map<math::LineSegment, unsigned int > segmentsInACounter;
    std::map<math::LineSegment, unsigned int > segmentsInBCounter;

    std::set<math::LineSegment> segmentsInA;
    std::set<math::LineSegment> segmentsInB;
    unsigned int currentSegmentAStrength = 0;
    unsigned int currentSegmentBStrength = 0;
    { // get nearest segments
        std::set<math::Vector3>::const_iterator itPoints;
        for( itPoints = entityPointsA.begin( ); itPoints != entityPointsA.end( ); ++itPoints ) {
            const std::list<math::LineSegment*>& nearestEdges = bb1.getEdges( *itPoints );
            std::list<math::LineSegment*>::const_iterator it2;
            for( it2 = nearestEdges.begin( ); it2 != nearestEdges.end( ); ++it2 ) {
                if ( segmentsInACounter.find( **it2 ) == segmentsInACounter.end( ) ) {
                    segmentsInACounter[ **it2 ] = 0;
                } // if
                unsigned int& counter = segmentsInACounter[ **it2 ];
                ++counter;
                if ( counter > currentSegmentAStrength ) {
                    segmentsInA.clear( );
                    currentSegmentAStrength = counter;
                    segmentsInA.insert(**it2);
                } else if ( counter == currentSegmentAStrength ) {
                    segmentsInA.insert(**it2);
                } // else if
            } // for
        } // for

        for( itPoints = entityPointsB.begin( ); itPoints != entityPointsB.end( ); ++itPoints ) {
            const std::list<math::LineSegment*>& nearestEdges = bb2.getEdges( *itPoints );
            std::list<math::LineSegment*>::const_iterator it2;
            for( it2 = nearestEdges.begin( ); it2 != nearestEdges.end( ); ++it2 ) {
                if ( segmentsInBCounter.find( **it2 ) == segmentsInBCounter.end( ) ) {
                    segmentsInBCounter[ **it2 ] = 0;
                } // if
                
                unsigned int& counter = segmentsInBCounter[ **it2 ];
                ++counter;
                if ( counter > currentSegmentBStrength ) {
                    segmentsInB.clear( );
                    currentSegmentBStrength = counter;
                    segmentsInB.insert( **it2 );
                } else if ( counter == currentSegmentBStrength ) {
                    segmentsInB.insert( **it2 );
                } // else if
            } // for
        } // for
    } // end block
    
    math::LineSegment chosenSegmentA( *segmentsInA.begin( ) );
    math::LineSegment chosenSegmentB( *segmentsInB.begin( ) );
    // special case where both entities are parallel each other
    if ( segmentsInA.size( ) > 1 || segmentsInB.size( ) > 1 ) {
        std::set<math::LineSegment>::const_iterator segInA, segInB;
        double minDistance = std::numeric_limits<double>::max( );
        for( segInA = segmentsInA.begin( ); segInA != segmentsInA.end( ); ++segInA ) {
            for( segInB = segmentsInB.begin( ); segInB != segmentsInB.end( ); ++segInB ) {
                double candidateDistance = segInA->distanceTo( *segInB );
                if ( candidateDistance < minDistance ) {
                    chosenSegmentA = *segInA;
                    chosenSegmentB = *segInB;
                    minDistance = candidateDistance;
                } // if
            } // for
        } // for
    } // if

    // only if the objects intersects in all three axis they can be tested to
    // be inside another
    if ( ( localStatus.relations[LimitRelation::X] & (1|2) ) == 0 && 
         ( localStatus.relations[LimitRelation::Y] & (1|2) ) == 0 && 
         ( localStatus.relations[LimitRelation::Z] & (1|2) ) == 0 ) {
        if ( bb1.isInside( chosenSegmentB.pointA ) || bb1.isInside( chosenSegmentB.pointB ) ||
             bb2.isInside( chosenSegmentA.pointA ) || bb2.isInside( chosenSegmentA.pointB ) ) {
            return 0.0;
        } 
    } // if

    return chosenSegmentA.distanceTo( chosenSegmentB, pointInA, pointInB );
}

void Entity::setProperty( Entity::PROPERTY property, PropertyValueType value )
{
    this->properties.insert( PropertyHashMap::value_type( property, value ) );
}

bool Entity::getBooleanProperty( Entity::PROPERTY property ) const
{
    PropertyHashMap::const_iterator it = this->properties.find( property );
    const bool* result = NULL;
    if ( it == this->properties.end( ) || ( result = boost::get<bool>( &it->second ) ) == NULL ) {
        return false;
    } 
    return *result;
}

std::string Entity::getStringProperty( PROPERTY property ) const
{
    PropertyHashMap::const_iterator it = this->properties.find( property );
    const std::string* result = NULL;
    if ( it == this->properties.end( ) || ( result = boost::get<std::string>( &it->second ) ) == NULL ) {
        return "";
    }
    return *result;
}

double Entity::getDoubleProperty( PROPERTY property ) const
{
    PropertyHashMap::const_iterator it = this->properties.find( property );
    const double* result = NULL;
    if ( it == this->properties.end( ) || ( result = boost::get<double>( &it->second ) ) == NULL ) {
        return 0.0;
    }
    return *result;
}

int Entity::getIntProperty( PROPERTY property ) const
{
    PropertyHashMap::const_iterator it = this->properties.find( property );
    const int* result = NULL;
    if ( it == this->properties.end( ) || ( result = boost::get<int>( &it->second ) ) == NULL ) {
        return 0;
    }
    return *result;
}

bool Entity::operator==( const EntityPtr& entity ) const
{
    return (*this == *entity); 
}

bool Entity::operator!=( const EntityPtr& entity ) const
{
    return !(*this == entity);
}

#define DOUBLE_DIFF_TOLERANCE 0.0001
bool Entity::operator==( const Entity& entity ) const
{
    // TODO: Review usage of Quaternion in opencog::spatial classes because
    // a comparison between 2 Quaternions generated from a same (pitch,roll,yaw)
    // set may be different each other depending on which constructor is used
    // (due to double values rounded off after conversions) 
    // For now, comparing pitch, roll and yaw with a specific rounding error tolerance
    const math::Quaternion& q = getOrientation();
    const math::Quaternion& entity_q = entity.getOrientation();
    double pitch_diff = fabs(entity_q.getPitch() - q.getPitch());
    double roll_diff = fabs(entity_q.getRoll() - q.getRoll());
    double yaw_diff = fabs(entity_q.getYaw() - q.getYaw());
    return ( entity.getPosition() == getPosition() &&
             pitch_diff <= DOUBLE_DIFF_TOLERANCE &&
             roll_diff <= DOUBLE_DIFF_TOLERANCE &&
             yaw_diff <= DOUBLE_DIFF_TOLERANCE &&
             entity.dimension == getDimension() );
}

bool Entity::operator!=( const Entity& entity ) const
{
    return !(*this == entity);
}

bool Entity::intersects( const Entity& other ) const
{
    return ( distanceTo( other ) == 0 );
}

std::string Entity::toString( ) const
{
    std::stringstream description;

    description << "id[";
    description << this->id;
    description << "] name[";
    description << this->name;
    description << "] position[";
    description << this->position.toString( );
    description << "] dim[";
    description << this->getDimension( ).toString( );
    description << "] orient[";
    description << this->orientation.toString( );
    description << "] radius[";
    description << this->expansionRadius;
    description << "]";

    return description.str();
}

Entity::LimitRelation Entity::computeObjectsLimits( const Entity& entityB ) const {

    const Entity& entityA = *this;

    LimitRelation status( &entityA, &entityB );

    const math::BoundingBox& bb1 = entityA.getBoundingBox( );
    const math::BoundingBox& bb2 = entityB.getBoundingBox( );
    
    const std::vector<math::Vector3>& corners1 = bb1.getAllCorners( );
    const std::vector<math::Vector3>& corners2 = bb2.getAllCorners( );

    // tolerance is used to determine if a coord
    // sufficiently near another to be considered relevant
    // for the algorithm
    double tolerance = 0.01;

    std::multiset<math::Vector3, SortByAxisX > sortedByXInA;
    std::multiset<math::Vector3, SortByAxisY > sortedByYInA;
    std::multiset<math::Vector3, SortByAxisZ > sortedByZInA;

    std::multiset<math::Vector3, SortByAxisX > sortedByXInB;
    std::multiset<math::Vector3, SortByAxisY > sortedByYInB;
    std::multiset<math::Vector3, SortByAxisZ > sortedByZInB;

    // sort all corners of both objects by X, Y and Z
    unsigned int i;
    for( i = 0; i < corners1.size( ); ++i ) {
        sortedByXInA.insert( corners1[i] );
        sortedByYInA.insert( corners1[i] );
        sortedByZInA.insert( corners1[i] );

        sortedByXInB.insert( corners2[i] );
        sortedByYInB.insert( corners2[i] );
        sortedByZInB.insert( corners2[i] );
    } // for
    
    std::multiset<math::Vector3, SortByAxisX >::const_iterator
        itFrontXA, itFrontXB, itFrontYA, itFrontYB, itFrontZA, itFrontZB;
    std::multiset<math::Vector3, SortByAxisX >::const_reverse_iterator
        itBackXA, itBackXB, itBackYA, itBackYB, itBackZA, itBackZB;

    // build a bunch of iterators that will be used to navigate
    // throught the sorted points
    itFrontXA = sortedByXInA.begin( ); itBackXA = sortedByXInA.rbegin( );
    itFrontYA = sortedByYInA.begin( ); itBackYA = sortedByYInA.rbegin( );
    itFrontZA = sortedByZInA.begin( ); itBackZA = sortedByZInA.rbegin( );

    itFrontXB = sortedByXInB.begin( ); itBackXB = sortedByXInB.rbegin( );
    itFrontYB = sortedByYInB.begin( ); itBackYB = sortedByYInB.rbegin( );
    itFrontZB = sortedByZInB.begin( ); itBackZB = sortedByZInB.rbegin( );

    math::Vector3 minA(itFrontXA->x, itFrontYA->y, itFrontZA->z );
    math::Vector3 maxA(itBackXA->x, itBackYA->y, itBackZA->z);

    math::Vector3 minB(itFrontXB->x, itFrontYB->y, itFrontZB->z );
    math::Vector3 maxB(itBackXB->x, itBackYB->y, itBackZB->z);

    // prepare a cache that will store the points sorted
    // by the insertion ordering
    std::vector<std::list<math::Vector3> > validPoints[2];
    validPoints[0].resize(6); validPoints[1].resize(6); 
    
    // get the first and the last point of the sorted sets.
    // these points will be considered the min and max ones
    status.limitsA[XMIN].insert( *itFrontXA );
    validPoints[0][XMIN].push_back( *itFrontXA++ );
    status.limitsA[XMAX].insert( *itBackXA );
    validPoints[0][XMAX].push_back( *itBackXA++ );
    status.limitsA[YMIN].insert( *itFrontYA );  
    validPoints[0][YMIN].push_back( *itFrontYA++ );
    status.limitsA[YMAX].insert( *itBackYA );   
    validPoints[0][YMAX].push_back( *itBackYA++ ); 
    status.limitsA[ZMIN].insert( *itFrontZA );  
    validPoints[0][ZMIN].push_back( *itFrontZA++ );
    status.limitsA[ZMAX].insert( *itBackZA ); 
    validPoints[0][ZMAX].push_back( *itBackZA++ ); 

    status.limitsB[XMIN].insert( *itFrontXB );
    validPoints[1][XMIN].push_back( *itFrontXB++ );
    status.limitsB[XMAX].insert( *itBackXB );
    validPoints[1][XMAX].push_back( *itBackXB++ );
    status.limitsB[YMIN].insert( *itFrontYB );  
    validPoints[1][YMIN].push_back( *itFrontYB++ );
    status.limitsB[YMAX].insert( *itBackYB );   
    validPoints[1][YMAX].push_back( *itBackYB++ ); 
    status.limitsB[ZMIN].insert( *itFrontZB );  
    validPoints[1][ZMIN].push_back( *itFrontZB++ );
    status.limitsB[ZMAX].insert( *itBackZB ); 
    validPoints[1][ZMAX].push_back( *itBackZB++ );

    // there are 6 types of limits. we will use a vector
    // of status to manage the set points of each limit
    // for both objects
    std::vector<bool> elementStatus(12);
    std::fill( elementStatus.begin( ), elementStatus.end( ), true );

    // now, traverse the sorted points sets and collect 
    // each point that is sufficiently near of the
    // latest cached point (using tolerance). This step
    // will build a set of the points that belongs to a
    // specific limit (XMIN, ZMAX, etc).
    unsigned int counter = 12;
    while( counter > 0 && itFrontXA != sortedByXInA.end( ) ) {        
        int objectId = 0;
        int axisId = 0;
        unsigned int i;
        for( i = 0; i < 12; ++i ) {
            if ( !elementStatus[i] ) {
                continue;
            } // if

            std::vector<std::set<math::Vector3> >* limits = NULL;
            double coordA = 0, coordB = 0;
            math::Vector3 referencePoint[3];
            math::Vector3 point;
            bool min = (i%2) == 0;
            unsigned int coordId = (i%6);

            if ( objectId == 0 ) {
                limits = &status.limitsA;
                referencePoint[0] = min ? *itFrontXA : *itBackXA;
                referencePoint[1] = min ? *itFrontYA : *itBackYA;
                referencePoint[2] = min ? *itFrontZA : *itBackZA;
            } else {
                limits = &status.limitsB;
                referencePoint[0] = min ? *itFrontXB : *itBackXB;
                referencePoint[1] = min ? *itFrontYB : *itBackYB;
                referencePoint[2] = min ? *itFrontZB : *itBackZB;
            } // else

            point = referencePoint[axisId];
            if ( axisId == 0 ) {
                coordA = std::max(validPoints[objectId][coordId].back( ).x, point.x);
                coordB = std::min(validPoints[objectId][coordId].back( ).x, point.x);
            } else if ( axisId == 1 ) {
                coordA = std::max(validPoints[objectId][coordId].back( ).y, point.y);
                coordB = std::min(validPoints[objectId][coordId].back( ).y, point.y);
            } else {
                coordA = std::max(validPoints[objectId][coordId].back( ).z, point.z);
                coordB = std::min(validPoints[objectId][coordId].back( ).z, point.z);
            } // else
            
            if ( std::abs(coordA - coordB) < tolerance ) {
                (*limits)[coordId].insert( point );
                validPoints[objectId][coordId].push_back( point );
            } else {
                elementStatus[i] = false;
                --counter;
            } // else

            axisId += ((i+1)%2 == 0 ) ? 1 : 0;
            if (coordId+1 == 6) {
                objectId = 1;
                axisId = 0;
            } // if
        } // else

        ++itFrontXA; ++itBackXA; ++itFrontXB; ++itBackXB;
        ++itFrontYA; ++itBackYA; ++itFrontYB; ++itBackYB;
        ++itFrontZA; ++itBackZA; ++itFrontZB; ++itBackZB;
    } // while
    // finally, classify the relation between the given objects
    // by using an algebra based on Region connection calculus (RCC)
    status.relations[LimitRelation::X] = (maxA.x < minB.x ) ? 1 : 
        ( maxB.x < minA.x ) ? 2 : 
        ( minA.x < minB.x && maxA.x < maxB.x ) ? 4 : 
        ( minB.x < minA.x && maxB.x < maxA.x ) ? 8 : 
        ( maxA.x == minB.x ) ? 16 :
        ( maxB.x == minA.x ) ? 32 :
        ( minA.x == minB.x && maxA.x == maxB.x ) ? 64 : // perfect overlap
        ( minA.x > minB.x && maxA.x < maxB.x ) ? 128 : // non perfect B overlaps A
        ( minB.x > minA.x && maxB.x < maxA.x ) ? 256 : // non perfect A overlaps B
        ( (minA.x == minB.x && maxA.x < maxB.x) || (minA.x > minB.x && maxA.x == maxB.x ) ) ? 512 : 1024;

    status.relations[LimitRelation::Y] = (maxA.y < minB.y ) ? 1 : 
        ( maxB.y < minA.y ) ? 2 : 
        ( minA.y < minB.y && maxA.y < maxB.y ) ? 4 : 
        ( minB.y < minA.y && maxB.y < maxA.y ) ? 8 : 
        ( maxA.y == minB.y ) ? 16 :
        ( maxB.y == minA.y ) ? 32 :
        ( minA.y == minB.y && maxA.y == maxB.y ) ? 64 : // perfect overlap
        ( minA.y > minB.y && maxA.y < maxB.y ) ? 128 : // non perfect B overlaps A
        ( minB.y > minA.y && maxB.y < maxA.y ) ? 256 : // non perfect A overlaps B
        ( (minA.y == minB.y && maxA.y < maxB.y) || (minA.y > minB.y && maxA.y == maxB.y ) ) ? 512 : 1024;


    status.relations[LimitRelation::Z] = (maxA.z < minB.z ) ? 1 : 
        ( maxB.z < minA.z ) ? 2 : 
        ( minA.z < minB.z && maxA.z < maxB.z ) ? 4 : 
        ( minB.z < minA.z && maxB.z < maxA.z ) ? 8 : 
        ( maxA.z == minB.z ) ? 16 :
        ( maxB.z == minA.z ) ? 32 :
        ( minA.z == minB.z && maxA.z == maxB.z ) ? 64 : // perfect overlap
        ( minA.z > minB.z && maxA.z < maxB.z ) ? 128 : // non perfect B overlaps A
        ( minB.z > minA.z && maxB.z < maxA.z ) ? 256 : // non perfect A overlaps B
        ( (minA.z == minB.z && maxA.z < maxB.z) || (minA.z > minB.z && maxA.z == maxB.z ) ) ? 512 : 1024;
    return status;
}

//std::vector<Entity::SPATIAL_RELATION>
//Entity::computeSpatialRelations( const Entity & observer,
//                                 double besideDistance,
//                                 const Entity & entityB,
//                                 const Entity & entityC ) const {

//    std::vector<SPATIAL_RELATION> spatialRelationsAB =
//        computeSpatialRelations( observer, besideDistance, entityB );

//    std::vector<SPATIAL_RELATION> spatialRelationsAC =
//        computeSpatialRelations( observer, besideDistance, entityC );
    
//    std::vector<SPATIAL_RELATION> relations;
    
//    std::vector<bool> activeRelationsAB(TOTAL_RELATIONS);
//    unsigned int i;
//    for( i = 0; i < activeRelationsAB.size( ); ++i ) {
//        activeRelationsAB[i] = false;
//    }

//    std::vector<bool> relationsAB(6);

//    std::vector<SPATIAL_RELATION>::const_iterator it;
//    for( it = spatialRelationsAB.begin( ); it != spatialRelationsAB.end( ); ++it ) {
//        if ( *it == RIGHT_OF ) {
//            relationsAB[0] = true;
//        }
//        else if ( *it == LEFT_OF ) {
//            relationsAB[1] = true;
//        }
//        else if ( *it == BEHIND ) {
//            relationsAB[2] = true;
//        }
//        else if ( *it == IN_FRONT_OF ) {
//            relationsAB[3] = true;
//        }
//        else if ( *it == ABOVE ) {
//            relationsAB[4] = true;
//        }
//        else if ( *it == BELOW ) {
//            relationsAB[5] = true;
//        }
//        else
//            activeRelationsAB[*it] = true;
//    }// for

//    for( it = spatialRelationsAC.begin( ); it != spatialRelationsAC.end( ); ++it ) {
//        if ( ( *it == LEFT_OF && relationsAB[0] ) ||
//             ( *it == RIGHT_OF && relationsAB[1] ) ||
//             ( *it == IN_FRONT_OF && relationsAB[2] ) ||
//             ( *it == BEHIND && relationsAB[3] ) ||
//             ( *it == BELOW && relationsAB[4] ) ||
//             ( *it == ABOVE && relationsAB[5] ) ) {
//            relations.push_back( BETWEEN );
//        }// if
//    }// for

//    return relations;
//}

//std::vector<Entity::SPATIAL_RELATION>
//Entity::computeSpatialRelations( const Entity & observer,
//                                 double besideDistance,
//                                 const Entity & entityB ) const
//{
//    const Entity & entityA = *this;

//    std::vector<SPATIAL_RELATION> spatialRelations;

//    math::Vector3 pointInA;
//    math::Vector3 pointInB;

//    LimitRelation status;
//    double distance = entityA.distanceTo( entityB, & pointInA, & pointInB, & status );

//    bool computeAsideRelations = false;
//    if ( ( status.relations[0] & 64 ) > 0 &&
//         ( status.relations[1] & 64 ) > 0 &&
//         ( status.relations[2] & 64 ) > 0 ) {
//        // A overlaps B and vice-versa
//        spatialRelations.push_back(INSIDE);
//        spatialRelations.push_back(TOUCHING);
//        spatialRelations.push_back(NEAR);
//        return spatialRelations;
//    }
//    else if ( ( status.relations[0] & 128 ) > 0 &&
//              ( status.relations[1] & 128 ) > 0 &&
//              ( status.relations[2] & 128 ) > 0 ) {
//        // A is inside B
//        spatialRelations.push_back(INSIDE);
//        spatialRelations.push_back(NEAR);
//        return spatialRelations;
//    }
//    else if ( ( status.relations[0] & 256 ) > 0 &&
//              ( status.relations[1] & 256 ) > 0 &&
//              ( status.relations[2] & 256 ) > 0 ) {
//        // A is outside B
//        spatialRelations.push_back(OUTSIDE);
//        spatialRelations.push_back(NEAR);
//    }
//    else if ( ( status.relations[0] & (64|128|512) ) > 0 &&
//              ( status.relations[1] & (64|128|512) ) > 0 &&
//              ( status.relations[2] & (64|128|512) ) > 0 ) {
//        // A is inside B and touching it
//        spatialRelations.push_back(INSIDE);
//        spatialRelations.push_back(TOUCHING);
//        spatialRelations.push_back(NEAR);
//        return spatialRelations;
//    }
//    else if ( ( status.relations[0] & (64|256|1024) ) > 0 &&
//              ( status.relations[1] & (64|256|1024) ) > 0 &&
//              ( status.relations[2] & (64|256|1024) ) > 0 ) {
//        // A is outside B but touching it
//        spatialRelations.push_back(OUTSIDE);
//        spatialRelations.push_back(TOUCHING);
//        spatialRelations.push_back(NEAR);
//    }
//    else if ( ( status.relations[0] & (1|2) ) == 0 &&
//              ( status.relations[1] & (1|2) ) == 0 &&
//              ( status.relations[2] & (1|2) ) == 0 ) {
//        // A is not completely inside B or vice-versa, but they intersect
//        spatialRelations.push_back(TOUCHING);
//        spatialRelations.push_back(NEAR);
//    }
//    else if ( ( status.relations[0] & (1|2|16|32) ) == 0 &&
//              ( status.relations[1] & (1|2|16|32) ) == 0 &&
//              ( status.relations[2] & 32 ) > 0 ) {
//        // A is on top of B
//        spatialRelations.push_back(ON_TOP_OF);
//        spatialRelations.push_back(TOUCHING);
//        spatialRelations.push_back(NEAR);
//    }
//    else if ( ( ( ( status.relations[0] & (16|32) ) > 0 &&
//                  ( status.relations[1] & (1|2) ) == 0
//                ) ||
//                ( ( status.relations[0] & (1|2) ) == 0 &&
//                  ( status.relations[1] & (16|32) ) > 0
//                )
//              ) &&
//              ( status.relations[2] & (1|2) ) == 0
//            ) {
//        // A is adjacent to B
//        spatialRelations.push_back(ADJACENT);
//        spatialRelations.push_back(TOUCHING);
//        spatialRelations.push_back(NEAR);
//    }
//    else {
//        computeAsideRelations = true;
//    }// if

//    ///*************************** WARNING *********************************////
//    // TODO: UP AXIS = Y (TODO: customize it)
//    //       an intersection must occur at X and Y besides
//    if ( ( status.relations[0] & (1|2) ) == 0 &&
//         ( status.relations[1] & (1|2) ) == 0 ) {
//        if ( ( status.relations[2] & (1|4|16) ) > 0 ) {
//            spatialRelations.push_back(BELOW);
//        }
//        else if ( ( status.relations[2] & (2|8|32) ) > 0 ) {
//            spatialRelations.push_back(ABOVE);
//        }
//    }// if
//    ///*************************** WARNING *********************************////

//    if ( distance > besideDistance ) {
//        spatialRelations.push_back(FAR_);
//        return spatialRelations;
//    }
//    else if ( distance < besideDistance * (LocalSpaceMap2D::NEAR_FACTOR/LocalSpaceMap2D::NEXT_FACTOR) ) {
//        spatialRelations.push_back(NEAR);
//    }
//    else {
//        spatialRelations.push_back(BESIDE);
//    }// if

//    if ( !computeAsideRelations ) {
//        return spatialRelations;
//    }

//    const math::Vector3& observerPosition = observer.getPosition( );

//    math::Vector3 observerDirection;
//    math::Vector3 objectDirection( pointInB - pointInA ); // direction vector from A (this) to B

//    bool observerBetweenObjects = false;

//    if ( observer.get_name( ) == entityA.get_name( ) ||
//         entityA.getBoundingBox( ).isInside( observerPosition ) ) {
//        observerDirection = (observer.getDirection( ) * objectDirection.length( )+1.0);
//    }
//    else if ( observer.get_name( ) == entityB.get_name( ) ||
//              entityB.getBoundingBox( ).isInside( observerPosition ) ) {
//        observerDirection = -(observer.getDirection( ) * objectDirection.length( )+1.0);
//    }
//    else {
//        math::Vector3 observerToEntityA, observerToEntityB;
//        {
//            math::Vector3 observerPoint, entityPoint;
//            observer.distanceTo( entityA, &observerPoint, &entityPoint );
//            observerToEntityA = entityPoint - observerPoint; // direction vector from observer to A (this)
//            observerDirection = observerPoint - entityPoint; // direction vector from A (this) to observer
//        }
//        {
//            math::Vector3 observerPoint, entityPoint;
//            observer.distanceTo( entityB, &observerPoint, &entityPoint );
//            observerToEntityB = entityPoint - observerPoint; // direction vector from observer to B (this)
//        }
//        observerToEntityA.normalise( );
//        observerToEntityB.normalise( );

//        double angle = std::acos( observerToEntityA.dotProduct( observerToEntityB ) );
//        observerBetweenObjects = ( std::abs(angle) > 150.0/180.0*M_PI );
//    }// if

//    double distanceToA = observerDirection.length( );
//    double distanceBetweenAandB = objectDirection.length( );

//    observerDirection.normalise(); // direction vector from A (this) to observer
//    objectDirection.normalise();   // direction vector from A (this) to B

//    double angle;
//    {
//        ///*************************** WARNING *********************************////
//        // TODO: UP AXIS = Z (TODO: customize it)

//        // Angle from observerDirection (A to observer) to objectDirection (A to B)
//        angle = std::atan2( objectDirection.y, objectDirection.x ) -
//                std::atan2( observerDirection.y, observerDirection.x );

//        if ( angle > M_PI ) {
//            angle -= M_PI*2.0;
//        } else if ( angle < -M_PI ) {
//            angle += M_PI*2.0;
//        }
//        ///*************************** WARNING *********************************////
//    }
//    angle *= 180.0/M_PI;
    
//    double lowerLimit = 20.0;
//    double upperLimit = 110.0;

//    if ( angle > lowerLimit && angle <= upperLimit ) {
//        spatialRelations.push_back( LEFT_OF );
//    }
//    else if ( ( angle > upperLimit && angle <= 180.0 ) ||
//              ( angle >= -180.0 && angle <= -upperLimit ) ) {
//        spatialRelations.push_back( observerBetweenObjects ? BEHIND : IN_FRONT_OF );
//    }
//    else if ( angle > -upperLimit && angle <= -lowerLimit ) {
//        spatialRelations.push_back( RIGHT_OF );
//    }
//    else {
//        if ( distanceToA > distanceBetweenAandB ) {
//            spatialRelations.push_back( observerBetweenObjects ? IN_FRONT_OF : BEHIND );
//        }
//        else {
//            spatialRelations.push_back( angle > 0 ? RIGHT_OF : LEFT_OF );
//        }
//    }// if

//    // BESIDE = next
//    // NEAR = near
    
//    return spatialRelations;
//}

//std::string Entity::spatialRelationToString( Entity::SPATIAL_RELATION relation ) {
//    switch( relation ) {
//    case LEFT_OF: return "left_of";
//    case RIGHT_OF: return "right_of";
//    case ABOVE: return "above";
//    case BELOW: return "below";
//    case BEHIND: return "behind";
//    case IN_FRONT_OF: return "in_front_of";
//    case BESIDE: return "beside";
//    case NEAR: return "near";
//    case FAR_: return "far";
//    case TOUCHING: return "touching";
//    case BETWEEN: return "between";
//    case INSIDE: return "inside";
//    case OUTSIDE: return "outside";
//    default:
//    case TOTAL_RELATIONS:
//        return " invalid relation ";
//    }
//}
