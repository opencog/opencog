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

#include <boost/variant/get.hpp>

#include <opencog/spatial/Entity.h>
#include <opencog/spatial/LocalSpaceMap2D.h>
#include <opencog/util/Logger.h>

using namespace Spatial;

Entity::Entity( const EntityPtr& entity ) : id(entity->id), name(entity->name), dimension(entity->dimension), position(entity->position), orientation(entity->orientation), expansionRadius(entity->expansionRadius), boundingBox(this)
{
    //this->properties.resize( Entity::NUMBER_OF_PROPERTIES );
}

Entity::Entity( long id, const std::string& name, const Math::Vector3& position, const Math::Dimension3& dimension, const Math::Quaternion& orientation, double radius ) : id(id), name(name), dimension(dimension), position(position), orientation(orientation), expansionRadius( radius ), boundingBox(this) {}

Math::Vector3 Entity::getDirection( void ) const
{
    return orientation.rotate( Math::Vector3::X_UNIT );
}

double Entity::distanceTo( const Entity& entity,
                           Math::Vector3* pointInA,
                           Math::Vector3* pointInB,
                           LimitRelation* status ) const
{

    /* TODO: This method is not precise. It does an aproximation of the real
     * distance between the entities comparing the distance of the edges of the
     * objects. It would be better if the three cases bellow were handle by this method:
     * 1) corner vs corner
     * 2) corner vs face
     * 3) face vs face
     */

    const Math::BoundingBox& bb1 = getBoundingBox( );
    const Math::BoundingBox& bb2 = entity.getBoundingBox( );

    LimitRelation localStatus = LocalSpaceMap2D::computeObjectsLimits( *this, entity );

    if ( status != NULL ) {
        *status = localStatus;
    } // if
    
    std::list<unsigned int> bordersOfA;
    std::list<unsigned int> bordersOfB;

    bool completelyOverlap = true;
    if ( ( localStatus.relations[LimitRelation::X] & (1|4|16) ) > 0 ) {
        // get the most right point of A and most left point of B
        bordersOfA.push_back( XMAX );
        bordersOfB.push_back( XMIN );
        completelyOverlap = false;
    } else if ( ( localStatus.relations[LimitRelation::X] & (2|8|32) ) > 0 ) {
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
    } else if ( ( localStatus.relations[LimitRelation::Y] & (2|8|32) ) > 0 ) {
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
    } else if ( ( localStatus.relations[LimitRelation::Z] & (2|8|32) ) > 0 ) {
        // get the most left point of A and most right point of B
        bordersOfA.push_back( ZMIN );
        bordersOfB.push_back( ZMAX );
        completelyOverlap = false;
    }

    // If objects contains each other in all dimensions, the distance is zero
    if ( completelyOverlap ) {
        return 0.0;
    }
    
    std::set<Math::Vector3> entityPointsA, entityPointsB;
    { // get the points in object which has a greatest score
        std::list<unsigned int>::const_iterator it;
        std::map<Math::Vector3, unsigned int> pointCounterA, pointCounterB;

        unsigned int counter = 0;
        for( it = bordersOfA.begin( ); it != bordersOfA.end( ); ++it ) {
            std::set<Math::Vector3>::const_iterator itPoints;
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
            std::set<Math::Vector3>::const_iterator itPoints;
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

    std::map<Math::LineSegment, unsigned int > segmentsInACounter;
    std::map<Math::LineSegment, unsigned int > segmentsInBCounter;

    std::set<Math::LineSegment> segmentsInA;
    std::set<Math::LineSegment> segmentsInB;
    unsigned int currentSegmentAStrength = 0;
    unsigned int currentSegmentBStrength = 0;
    { // get nearest segments
        std::set<Math::Vector3>::const_iterator itPoints;
        for( itPoints = entityPointsA.begin( ); itPoints != entityPointsA.end( ); ++itPoints ) {
            const std::list<Math::LineSegment*>& nearestEdges = bb1.getEdges( *itPoints );
            std::list<Math::LineSegment*>::const_iterator it2;
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
            const std::list<Math::LineSegment*>& nearestEdges = bb2.getEdges( *itPoints );
            std::list<Math::LineSegment*>::const_iterator it2;
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
    
    const Math::LineSegment& chosenSegmentA = *segmentsInA.begin( );
    const Math::LineSegment& chosenSegmentB = *segmentsInB.begin( );

    // only if the objects intersects in all three axis they can be tested to
    // be inside another
    if ( ( localStatus.relations[LimitRelation::X] & (1|2) ) == 0 && 
         ( localStatus.relations[LimitRelation::Y] & (1|2) ) == 0 && 
         ( localStatus.relations[LimitRelation::Z] & (1|2) ) == 0 ) {
        if ( bb1.isInside( chosenSegmentB.pointA ) || bb1.isInside( chosenSegmentB.pointB ) ||
             bb2.isInside( chosenSegmentA.pointA ) || bb2.isInside( chosenSegmentA.pointB ) ) {
            return 0.0;
        } // if
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
    } // if
    return *result;
}

std::string Entity::getStringProperty( PROPERTY property ) const
{
    PropertyHashMap::const_iterator it = this->properties.find( property );
    const std::string* result = NULL;
    if ( it == this->properties.end( ) && ( result = boost::get<std::string>( &it->second ) ) == NULL ) {
        return "";
    } // if
    return *result;
}

double Entity::getDoubleProperty( PROPERTY property ) const
{
    PropertyHashMap::const_iterator it = this->properties.find( property );
    const double* result = NULL;
    if ( it == this->properties.end( ) && ( result = boost::get<double>( &it->second ) ) == NULL ) {
        return 0.0;
    } // if
    return *result;
}

int Entity::getIntProperty( PROPERTY property ) const
{
    PropertyHashMap::const_iterator it = this->properties.find( property );
    const int* result = NULL;
    if ( it == this->properties.end( ) && ( result = boost::get<int>( &it->second ) ) == NULL ) {
        return 0;
    } // if
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
    // TODO: Review usage of Quaternion in opencog::Spatial classes because
    // a comparison between 2 Quaternions generated from a same (pitch,roll,yaw)
    // set may be different each other depending on which constructor is used
    // (due to double values rounded off after conversions) 
    // For now, comparing pitch, roll and yaw with a specific rounding error tolerance
    const Math::Quaternion& q = getOrientation();
    const Math::Quaternion& entity_q = entity.getOrientation();
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
