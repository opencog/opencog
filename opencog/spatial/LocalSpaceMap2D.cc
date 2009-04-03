/*
 * opencog/spatial/LocalSpaceMap2D.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
 * All Rights Reserved
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
#include "LocalSpaceMap2D.h"

#include "TB_ASSERT.h"
#include "util/Logger.h"
#include "util/exceptions.h"

#include "Vector3.h"
#include "StaticEntity.h"

#include <boost/lexical_cast.hpp>

#define HUGE_DISTANCE 999999.9

using namespace Spatial;
using namespace opencog;

bool LocalSpaceMap2D::addToSuperEntity( const EntityPtr& entity )
{
    bool merged = false;
    LongEntityPtrHashMap::iterator it2;
    SuperEntityPtr previousSuperEntity;

    logger().log(opencog::Logger::DEBUG, "LocalSpaceMap2D::addToSuperEntity Verifying entity[%s - %ld]", entity->getName( ).c_str( ), entity->getId( ) );

    std::map< long, SuperEntityPtr > intersection;

    for ( it2 = this->entities.begin( ); it2 != this->entities.end( ); ++it2 ) {
        if ( it2->first == entity->getId( ) ) {
            continue;
        } // if

        //std::cout << "inspecting " << it2->first << std::endl;

        if ( it2->second->intersects( entity ) ) {
            //std::cout << it2->first << " intersects " << entity->getId( ) << std::endl;

            if ( intersection[ it2->first ].get( ) == NULL ) {
                //std::cout << it2->first << " was not defined as a superentity" << std::endl;
                std::list<SuperEntityPtr>::iterator it3;
                for ( it3 = this->superEntities.begin( ); it3 != this->superEntities.end( ); ++it3 ) {
                    if ( (*it3)->containsEntity( it2->first ) ) {
                        //std::cout << it2->first << " super entity found" << std::endl;
                        intersection[it2->first] = *it3;
                    } // if
                } // for
            } // if


            if ( intersection[ entity->getId( ) ].get( ) == NULL ) {
                if ( intersection[it2->first].get( ) == NULL ) {
                    //std::cout << "Merging two free entities " << it2->first << " " << entity->getId( ) << std::endl;
                    intersection[it2->first] = SuperEntityPtr( new SuperEntity( it2->second, entity ) );
                    intersection[entity->getId( )] = intersection[it2->first];
                    this->superEntities.push_back( intersection[it2->first] );
                } else {
                    //unsigned int subEntities =
                    intersection[it2->first]->getSubEntitiesIds( ).size( );
                    if ( intersection[it2->first]->merge( entity ) ) {
                        //std::cout << "Merging free entity to superentity "
                        //     << (result?"t":"f")
                        //     << " before: " << subEntities
                        //     << " after: " << intersection[it2->first]->getSubEntitiesIds( ).size( )
                        //     << std::endl;
                        intersection[ entity->getId( ) ] = intersection[it2->first];
                    } // if
                } // else
            } else {
                if ( intersection[it2->first].get( ) == NULL ) {
                    //unsigned int subEntities = intersection[entity->getId( )]->getSubEntitiesIds( ).size( );
                    if ( intersection[entity->getId( )]->merge( it2->second ) ) {
                        //std::cout << "Merging superentity into free entity["
                        //     << it2->first
                        //     << "] "
                        //     << (result?"t":"f")
                        //     << " before: " << subEntities
                        //     << " after: " << intersection[entity->getId( )]->getSubEntitiesIds( ).size( )
                        //     << std::endl;
                        intersection[it2->first] = intersection[entity->getId( )];
                    } // if
                } else if ( intersection[entity->getId( )] != intersection[it2->first] ) {

                    //unsigned int subEntities1 = intersection[entity->getId( )]->getSubEntitiesIds( ).size( );
                    //unsigned int subEntities2 = intersection[it2->first]->getSubEntitiesIds( ).size( );
                    if ( intersection[entity->getId( )]->merge( intersection[it2->first] ) ) {
                        //std::cout << "Merging two superenties ["
                        //     << it2->first
                        //     << "] "
                        //     << (result?"t":"f")
                        //     << " before1: " << subEntities1 << " before2: " << subEntities2
                        //     << " after: " << intersection[entity->getId( )]->getSubEntitiesIds( ).size( )
                        //     << std::endl;

                        std::list<SuperEntityPtr>::iterator it4 =
                            std::find( this->superEntities.begin( ), this->superEntities.end( ), intersection[ it2->first] );
                        this->superEntities.erase( it4 );

                        intersection[it2->first] = intersection[entity->getId( )];
                    } // if
                } // else
            } // else
            merged = true;

        } // if
    } // for

    return merged;
}

/** ---------------------------------------------------------------------------
 * Private Functions
 * ----------------------------------------------------------------------------
 */
bool LocalSpaceMap2D::outsideMap( const std::vector<Spatial::Math::LineSegment>& segments )
{
    bool outside = true;

    unsigned int i;
    for ( i = 0; outside && i < segments.size( ); ++i ) {
        const Math::Vector3& point = segments[i].pointA;

        if ( point.x  >= _xMin && point.x  <= _xMax && point.y >= _yMin && point.y <= _yMax ) {
            outside = false;
        } // if
    } // for

    return outside;
}


/** ---------------------------------------------------------------------------
 * Public Functions
 * ----------------------------------------------------------------------------
 */
LocalSpaceMap2D::LocalSpaceMap2D(Spatial::Distance xMin, Spatial::Distance xMax, unsigned int xDim,
                                 Spatial::Distance yMin, Spatial::Distance yMax, unsigned int yDim,
                                 Spatial::Distance radius) :
        _xMin(xMin), _xMax(xMax), _xDim(xDim),
        _yMin(yMin), _yMax(yMax), _yDim(yDim),
        _radius(radius)
{
    Distance xDelta = xMax - xMin;
    Distance yDelta = yMax - yMin;
    _diagonalSize = sqrt(xDelta * xDelta + yDelta * yDelta);
}

LocalSpaceMap2D::LocalSpaceMap2D( const LocalSpaceMap2D& map )
{
    this->_xMin = map._xMin;
    this->_xMax = map._xMax;
    this->_yMin = map._yMin;
    this->_yMax = map._yMax;

    this->_xDim = map._xDim;
    this->_yDim = map._yDim;
    this->_radius = map._radius;

    copyObjects( map );
}

LocalSpaceMap2D::~LocalSpaceMap2D()
{
}

LocalSpaceMap2D* LocalSpaceMap2D::clone() const
{
    LocalSpaceMap2D* clonedMap = new LocalSpaceMap2D(_xMin, _xMax, _xDim,
            _yMin, _yMax, _xDim,
            _radius);
    //clonedMap->objects = objects;
    LongEntityPtrHashMap::const_iterator it1;
    for ( it1 = this->entities.begin( ); it1 != this->entities.end( ); ++it1 ) {
        clonedMap->entities.insert( LongEntityPtrHashMap::value_type( it1->first, it1->second->clone( ) ) );
    } // for

    std::list<SuperEntityPtr>::const_iterator it;
    for ( it = this->superEntities.begin( ); it != this->superEntities.end( ); ++it ) {
        clonedMap->superEntities.push_back( (*it)->clone( ) );
    } // for

    clonedMap->gridPoints = this->gridPoints;
    clonedMap->_grid = _grid;
    clonedMap->_grid_nonObstacle = _grid_nonObstacle;

    return clonedMap;
}

bool LocalSpaceMap2D::operator==(const LocalSpaceMap2D& other) const
{
    if (other._xMin != _xMin ||
            other._yMin != _yMin ||
            other._xMax != _xMax ||
            other._yMax != _yMax ||
            other._xDim != _xDim ||
            other._yDim != _yDim ||
            other._radius != _radius ||
            //other.objects.size( ) != objects.size( ) ||
            other.superEntities.size( ) != other.superEntities.size( ) ||
            other.entities.size( ) != entities.size( ) ||
            other._grid.size() != _grid.size() ||
            other._grid_nonObstacle.size() != _grid_nonObstacle.size() ) {
        //std::cout << other.entities.size( ) << " <1> " <<  entities.size( ) << std::endl;
        //std::cout << other.superEntities.size( ) << " <2> " <<  other.superEntities.size( ) << std::endl;
        return false;
    }

    LongEntityPtrHashMap::const_iterator it;
    for ( it = this->entities.begin( ); it != this->entities.end( ); ++it ) {
        if ( other.entities.find( it->first ) == other.entities.end( ) ) {
            //std::cout << it->first << " WAS NOT FOUND ON SECOND MAP" << std::endl;
            return false;
        } // if
    } // if
    return true;

}

void LocalSpaceMap2D::save(FILE* fp) const
{

    //unsigned int numberOfObjects = this->objects.size();
    unsigned int numberOfObjects = this->entities.size();
    fwrite( &numberOfObjects, sizeof(unsigned int), 1, fp );

    //ObjectHashMap::const_iterator it;
    LongEntityPtrHashMap::const_iterator it;
    //for( it = this->objects.begin( ); it != this->objects.end( ); ++it ) {
    for ( it = this->entities.begin( ); it != this->entities.end( ); ++it ) {

        //std::string id = it->first;
        std::string id = it->second->getName( );
        unsigned int length = id.size();

        fwrite(&length, sizeof(unsigned int), 1, fp);
        fwrite(id.c_str(), sizeof(char), length, fp);


        Spatial::ObjectMetaData metaData( it->second->getPosition( ).x,
                                          it->second->getPosition( ).y,
                                          it->second->getLength( ),
                                          it->second->getWidth( ),
                                          it->second->getHeight( ),
                                          it->second->getOrientation( ).getRoll( ) );

        bool isObstacle = it->second->getBooleanProperty( Entity::OBSTACLE );

        fwrite(&metaData, sizeof(Spatial::ObjectMetaData), 1, fp);
        fwrite(&isObstacle, sizeof(bool), 1, fp );

        //fwrite(&it->second.metaData, sizeof(Spatial::ObjectMetaData), 1, fp);
        //fwrite(&it->second.isObstacle, sizeof(bool), 1, fp );

    } // for

}

void LocalSpaceMap2D::load(FILE* fp)
{
    unsigned int numberOfObjects;
    fread(&numberOfObjects, sizeof(unsigned int), 1, fp);

    for (unsigned int i = 0; i < numberOfObjects; ++i) {

        unsigned int length;
        fread(&length, sizeof(unsigned int), 1, fp);

        char* id = new char[length+1];
        fread(id, sizeof(char), length, fp);
        id[length] = '\0';

        Spatial::ObjectMetaData metadata;
        fread(&metadata, sizeof(Spatial::ObjectMetaData), 1, fp);

        bool isObstacle;
        fread(&isObstacle, sizeof(bool), 1, fp);

        addObject(std::string(id), metadata, isObstacle );

        delete id;
    } // for

}

Spatial::Distance LocalSpaceMap2D::xGridWidth() const
{
    return (_xMax -_xMin) / Distance(_xDim);
}

Spatial::Distance LocalSpaceMap2D::yGridWidth() const
{
    return (_yMax -_yMin) / Distance(_yDim);
}

Spatial::Distance LocalSpaceMap2D::xMin() const
{
    return _xMin;
}

Spatial::Distance LocalSpaceMap2D::xMax() const
{
    return _xMax;
}

unsigned int LocalSpaceMap2D::xDim() const
{
    return _xDim;
}

Spatial::Distance LocalSpaceMap2D::yMin() const
{
    return _yMin;
}

Spatial::Distance LocalSpaceMap2D::yMax() const
{
    return _yMax;
}

unsigned int LocalSpaceMap2D::yDim() const
{
    return _yDim;
}

Spatial::Distance LocalSpaceMap2D::diagonalSize() const
{
    return _diagonalSize;
}

Spatial::Distance LocalSpaceMap2D::radius() const
{
    return _radius;
}

bool LocalSpaceMap2D::illegal(const Spatial::Point& pt) const
{
    if (pt.first < _xMin || pt.first > _xMax || pt.second < _yMin || pt.second > _yMax) {
        logger().log(opencog::Logger::FINE, "LocalSpaceMap - illegal(pt = (%f, %f)): out of the map!", pt.first, pt.second);
        return true;
    }
    return gridIllegal(snap(pt));
}

Spatial::Point LocalSpaceMap2D::getNearestFreePoint(const Spatial::Point& pt) const throw (opencog::RuntimeException, std::bad_exception)
{
    GridPoint gp = snap(pt);
    if (!gridIllegal(gp)) {
        return pt;
    }
    bool foundFreePoint = false;
    Point nearestFreePoint;
    double nearestDistance = 0.0;
    unsigned int limit = std::max(_xDim, _yDim);
    for (unsigned int step = 1; !foundFreePoint && (step < limit); step++) {
        bool valid_lower_x = step <= gp.first;
        bool valid_lower_y = step <= gp.second;
        bool valid_upper_x = step < (_xDim - gp.first);
        bool valid_upper_y = step < (_yDim - gp.second);
        if (!valid_lower_x && !valid_lower_y && !valid_upper_x && !valid_upper_y) {
            break;
        }
        unsigned int lower_x = valid_lower_x ? (gp.first - step) : 0;
        unsigned int lower_y = valid_lower_y ? (gp.second - step) : 0;
        unsigned int upper_x = valid_upper_x ? (gp.first + step) : _xDim - 1;
        unsigned int upper_y = valid_upper_y ? (gp.second + step) : _yDim - 1;
        if (valid_lower_x) {
            for (unsigned int y = lower_y; y <= upper_y; y++) {
                if (!gridIllegal(lower_x, y)) {
                    Point freePoint = unsnap(GridPoint(lower_x, y));
                    double distance = sqrt(pow(freePoint.first - pt.first, 2) + pow(freePoint.second - pt.second, 2));
                    if (!foundFreePoint || (distance < nearestDistance)) {
                        nearestFreePoint = freePoint;
                        nearestDistance = distance;
                        foundFreePoint = true;
                    }
                }
            }
        }
        if (valid_lower_y) {
            for (unsigned int x = lower_x; x <= upper_x; x++) {
                if (!gridIllegal(x, lower_y)) {
                    Point freePoint = unsnap(GridPoint(x, lower_y));
                    double distance = sqrt(pow(freePoint.first - pt.first, 2) + pow(freePoint.second - pt.second, 2));
                    if (!foundFreePoint || (distance < nearestDistance)) {
                        nearestFreePoint = freePoint;
                        nearestDistance = distance;
                        foundFreePoint = true;
                    }
                }
            }
        }
        if (valid_upper_x) {
            for (unsigned int y = lower_y; y <= upper_y; y++) {
                if (!gridIllegal(upper_x, y)) {
                    Point freePoint = unsnap(GridPoint(upper_x, y));
                    double distance = sqrt(pow(freePoint.first - pt.first, 2) + pow(freePoint.second - pt.second, 2));
                    if (!foundFreePoint || (distance < nearestDistance)) {
                        nearestFreePoint = freePoint;
                        nearestDistance = distance;
                        foundFreePoint = true;
                    }
                }
            }
        }
        if (valid_upper_y) {
            for (unsigned int x = lower_x; x <= upper_x; x++) {
                if (!gridIllegal(x, upper_y)) {
                    Point freePoint = unsnap(GridPoint(x, upper_y));
                    double distance = sqrt(pow(freePoint.first - pt.first, 2) + pow(freePoint.second - pt.second, 2));
                    if (!foundFreePoint || (distance < nearestDistance)) {
                        nearestFreePoint = freePoint;
                        nearestDistance = distance;
                        foundFreePoint = true;
                    }
                }
            }
        }
    }
    if (!foundFreePoint) {
        throw opencog::RuntimeException(TRACE_INFO,
                                        "LocalSpaceMap2D - Could not find the nearest valid grid point from (%u,%u)",
                                        pt.first, pt.second, gp.first, gp.second);
    }
    return nearestFreePoint;
}

bool LocalSpaceMap2D::gridIllegal(const Spatial::GridPoint& gp) const
{
    return gridIllegal(gp.first, gp.second);
}

bool LocalSpaceMap2D::gridIllegal(unsigned int i, unsigned int j) const
{
    bool result;
    if ((result = gridOccupied(i, j))) {
        logger().log(opencog::Logger::FINE, "LocalSpaceMap - gridIllegal(%u, %u): grid occupied by an object!", i, j);
    } else if ((result = !coordinatesAreOnGrid(i, j))) {
        logger().log(opencog::Logger::FINE, "LocalSpaceMap - gridIllegal(%u, %u): coordinates not on grid!", i, j);
    }
    return result;
}

bool LocalSpaceMap2D::gridOccupied(const Spatial::GridPoint& gp) const
{
    Spatial::GridMap::const_iterator itr = _grid.find(gp);
    return (itr != _grid.end() && !(itr->second.empty()));
}

bool LocalSpaceMap2D::gridOccupied(unsigned int i, unsigned int j) const
{
    return (gridOccupied(GridPoint(i, j)));
}

bool LocalSpaceMap2D::gridOccupied_nonObstacle(const Spatial::GridPoint& gp) const
{
    Spatial::GridMap::const_iterator itr = _grid_nonObstacle.find(gp);
    return (itr != _grid_nonObstacle.end() && !(itr->second.empty()));
}

bool LocalSpaceMap2D::gridOccupied_nonObstacle(unsigned int i, unsigned int j) const
{
    return (gridOccupied_nonObstacle(GridPoint(i, j)));
}

bool LocalSpaceMap2D::containsObject(const Spatial::ObjectID& id) const
{
    try {
        getEntity( id );
        return true;
    } catch ( opencog::NotFoundException& ex ) {
        return false;
    } // catch
}

bool LocalSpaceMap2D::isObstacle(const Spatial::ObjectID& id) const
{
    try {
        return getEntity( id )->getBooleanProperty( Entity::OBSTACLE );
    } catch ( opencog::NotFoundException& ex ) {
        return false;
    } // catch
}

bool LocalSpaceMap2D::isNonObstacle(const Spatial::ObjectID& id) const
{
    return !isObstacle( id );
}

const std::vector<Spatial::GridPoint>& LocalSpaceMap2D::getObjectPoints(const Spatial::ObjectID& id) const  throw(opencog::NotFoundException)
{
    long idHash = boost::hash<std::string>()( id );
    LongGridPointVectorHashMap::const_iterator it = this->gridPoints.find( idHash );
    if ( it == this->gridPoints.end( ) ) {
        throw opencog::NotFoundException( TRACE_INFO, "LocalSpaceMap2D - There is no object %s", id.c_str() );
    } // if
    return it->second;
}

Spatial::Point LocalSpaceMap2D::getNearestObjectPoint( const Spatial::Point& referencePoint, const Spatial::ObjectID& objectID ) const throw (opencog::NotFoundException)
{

    if ( !containsObject( objectID ) ) {
        throw opencog::NotFoundException( TRACE_INFO, "LocalSpaceMap2D - There is no object %s", objectID.c_str() );
    } // if

    const EntityPtr& entity = getEntity( objectID );
    const Math::BoundingBox& bb = entity->getBoundingBox( );

    std::vector<Math::LineSegment> bottomSegments;
    bottomSegments.push_back( Math::LineSegment( bb.getCorner( Math::BoundingBox::FAR_LEFT_BOTTOM ), bb.getCorner( Math::BoundingBox::FAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( Math::LineSegment( bb.getCorner( Math::BoundingBox::FAR_RIGHT_BOTTOM ), bb.getCorner( Math::BoundingBox::NEAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( Math::LineSegment( bb.getCorner( Math::BoundingBox::NEAR_RIGHT_BOTTOM ), bb.getCorner( Math::BoundingBox::NEAR_LEFT_BOTTOM ) ) );
    bottomSegments.push_back( Math::LineSegment( bb.getCorner( Math::BoundingBox::NEAR_LEFT_BOTTOM ), bb.getCorner( Math::BoundingBox::FAR_LEFT_BOTTOM ) ) );




    Math::Vector3 point( referencePoint.first, referencePoint.second );
    //const ObjectMetaData& md = getMetaData(objectID);
    //Math::Vector3 nearestPoint( md.centerX, md.centerY);
    Math::Vector3 nearestPoint( entity->getPosition( ) );

    double minDistance = std::numeric_limits<double>::max( );

    //const std::vector<Spatial::Math::LineSegment>& segments = it->second.borderSegments;
    for ( unsigned int i = 0; i < bottomSegments.size( ); ++i ) {
        //Math::Vector3 candidatePoint = getNearestPointInSegment( point, segments[i] );
        Math::Vector3 candidatePoint = bottomSegments[i].nearestPointInSegment( point );
        double candidateDistance = ( candidatePoint - point ).length( );

        if ( candidateDistance < minDistance ) {
            minDistance = candidateDistance;
            nearestPoint = candidatePoint;
        } // if
    } // for
    return Spatial::Point( nearestPoint.x, nearestPoint.y );
}

void LocalSpaceMap2D::copyObjects(const LocalSpaceMap2D& otherMap)
{
    //ObjectHashMap::const_iterator it;
    LongEntityPtrHashMap::const_iterator it;

    for ( it = otherMap.entities.begin( ); it != otherMap.entities.end( ); ++it ) {
        Spatial::ObjectMetaData metaData( it->second->getPosition( ).x,
                                          it->second->getPosition( ).y,
                                          it->second->getLength( ),
                                          it->second->getWidth( ),
                                          it->second->getHeight( ),
                                          it->second->getOrientation( ).getRoll( ) );
        addObject( it->second->getName( ), metaData, it->second->getBooleanProperty( Entity::OBSTACLE ) );
    } // for
}

void LocalSpaceMap2D::removeObject(const Spatial::ObjectID& id)
{
    logger().log(opencog::Logger::FINE, "LocalSpaceMap2D::removeObject(%s)", id.c_str());

    //ObjectHashMap::iterator it = this->objects.find( id );
    long idHash = boost::hash<std::string>()( id );
    //if ( it == this->objects.end( ) ) {
    LongEntityPtrHashMap::iterator it = this->entities.find( idHash );
    if ( it == this->entities.end( ) ) {
        logger().log(opencog::Logger::FINE, "LocalSpaceMap2D - There is no object %s on this map", id.c_str( ) );
        return;
    } // if

    //const char* internalId = it->first.c_str( );
    const char* internalId = it->second->getName( ).c_str( );
    bool isObstacle = it->second->getBooleanProperty( Entity::OBSTACLE );

    // removing object grid points
    //std::vector<GridPoint>::const_iterator itPoints;
    //for( itPoints = it->second.gridPoints.begin( ); itPoints != it->second.gridPoints.end( ); ++itPoints ) {
    LongGridPointVectorHashMap::const_iterator it2 = gridPoints.find( it->first );
    const std::vector<GridPoint>& entityGridPoints = it2->second;

    unsigned int i;
    for ( i = 0; i < entityGridPoints.size( ); ++i ) {

        //if ( it->second.isObstacle ) {
        if ( isObstacle ) {

            _grid[ entityGridPoints[i] ].erase( internalId );
            if ( _grid[ entityGridPoints[i] ].size( ) == 0 ) {
                _grid.erase( entityGridPoints[i] );
            } // if
        } else {

            _grid_nonObstacle[ entityGridPoints[i] ].erase( internalId );
            if ( _grid_nonObstacle[ entityGridPoints[i] ].size( ) == 0 ) {
                _grid_nonObstacle.erase( entityGridPoints[i] );
            } // if
        } // else
    } // for

    this->gridPoints.erase( idHash );
    this->entities.erase( it );

    std::list<SuperEntityPtr>::iterator it3;
    bool superEntityFound = false;
    for ( it3 = this->superEntities.begin( ); !superEntityFound && it3 != this->superEntities.end( ); ++it3 ) {
        if ( (*it3)->containsEntity( idHash ) ) {
            std::vector<long> ids = (*it3)->getSubEntitiesIds( );

            try {
                //std::cout << "Removing " << idHash << " from: " << *it3 << " # of elements: " << ids.size( ) << std::endl;

                (*it3)->removeEntity( idHash );
            } catch ( opencog::InvalidParamException& ex ) {
                //std::cout << "Removing invalid " << idHash << std::endl;
                //delete *it3;
                this->superEntities.erase( it3 );

                unsigned int j;
                for ( j = 0; j < ids.size( ); ++j ) {
                    LongEntityPtrHashMap::iterator it4 = this->entities.find( ids[j] );
                    if ( it4 != this->entities.end( ) ) {
                        addToSuperEntity( it4->second );
                    } // if
                } // for

            } // catch
            superEntityFound = true;
        } // if
    } // for

}

Distance LocalSpaceMap2D::minDist(const Spatial::ObjectID& id, const Spatial::Point& p) const
{
    std::vector<Spatial::Point> all;
    allPoints(id, back_inserter(all));
    if (all.empty()) {
        logger().log(opencog::Logger::ERROR,
                     "LocalSpaceMap2D::minDist(): No point associated to obj '%s'.",
                     id.c_str());

        // return a huge distance soh this object is discarded
        return (HUGE_DISTANCE);
    }
    //TB_ASSERT(!all.empty());
    Distance d = eucDist(p, all.front());
    for (std::vector<Spatial::Point>::const_iterator it = ++all.begin();it != all.end();++it) {
        d = std::min(d, eucDist(p, *it));
    }
    return d;
}

bool LocalSpaceMap2D::coordinatesAreOnGrid(unsigned int x, unsigned int y) const
{
    return (x < _xDim && y < _yDim);
}

Spatial::Point LocalSpaceMap2D::nearbyPoint(const Spatial::Point& src, const Spatial::ObjectID& id) const
{

    // getNearestObjectPoint will throw an exception if the object isn't inside the map
    Spatial::Point closestPoint = getNearestObjectPoint( src, id );

    return getNearestFreePoint(closestPoint); //Suggested by Welter
    //return findFree(closestPoint, Point(src.first - closestPoint.first,
    //         src.second - closestPoint.second));
}

Spatial::Point LocalSpaceMap2D::behindPoint(const Spatial::Point& src, const Spatial::ObjectID& id) const
{
    Point loc = centerOf(id);

    //check for the degenerate case
    if (snap(loc) == snap(src))
        return loc;

    return findFree(loc, Point(loc.first - src.first, loc.second - src.second));
}

Spatial::Point LocalSpaceMap2D::findFree(const Spatial::Point& p, const Spatial::Point& direction) const
{
    //normalize the step
    Spatial::Distance d = 2.0 * eucDist(Point(0, 0), direction) / (xGridWidth() + yGridWidth());
    Spatial::Point normDirection(direction.first / d, direction.second / d);
    Spatial::Point result = p;

    Spatial::GridPoint g = snap(result);
    while (gridIllegal(g)) {
        result.first += normDirection.first;
        result.second += normDirection.second;
        g = snap(result);
        if (!coordinatesAreOnGrid(g.first, g.second)) {
            result.first -= normDirection.first;
            result.second -= normDirection.second;
            break;
        }
    }
    return result;
}

Spatial::Point LocalSpaceMap2D::centerOf(const Spatial::ObjectID& id) const
{
    Spatial::Point center(0, 0);
    try {
        const EntityPtr& entity = getEntity( id );
        center.first = entity->getPosition( ).x;
        center.second = entity->getPosition( ).y;
    } catch ( opencog::NotFoundException& ex ) {
        // ignore
    } // catch
    return center;
}

Spatial::GridPoint LocalSpaceMap2D::snap(const Spatial::Point& p) const
{
    Spatial::GridPoint gp =
        Spatial::GridPoint(p.first <= _xMin ? 0 : p.first >= _xMax ? _xDim - 1 :
                           (unsigned int)((p.first - _xMin) / xGridWidth()),
                           p.second <= _yMin ? 0 : p.second >= _yMax ? _yDim - 1 :
                           (unsigned int)((p.second - _yMin) / yGridWidth()));
    return gp;
}

Spatial::Point LocalSpaceMap2D::unsnap(const Spatial::GridPoint& g) const
{
    return Spatial::Point(Spatial::Distance(g.first) * xGridWidth() + _xMin + xGridWidth() / 2,
                          Spatial::Distance(g.second) * yGridWidth() + _yMin + yGridWidth() / 2);
}

Spatial::Point LocalSpaceMap2D::getNearFreePointAtDistance( const Spatial::Point& position, float distance, const Spatial::Point& startDirection ) const throw (opencog::NotFoundException)
{
    // do a raytrace to
    unsigned int step = 5;
    Math::Quaternion rotationStep( Math::Vector3::Z_UNIT, step * M_PI / 180.0 );

    Spatial::Math::Vector3 rayDirection( startDirection.first, startDirection.second );
    rayDirection.normalise( );
    Spatial::GridPoint freeGridPoint;
    float freeGridPointDistance = 0;
    bool freeGridPointFound = false;
    float minimalDistanceThreshold = (distance * 0.90); // 90% of the distance is enough

    unsigned int i;
    for ( i = 0; i < 360; i += step ) {

        Spatial::Math::Vector3 borderPoint = ( rayDirection * distance ) +
                                             Spatial::Math::Vector3( position.first, position.second );

        Spatial::Point endPosition( borderPoint.x, borderPoint.y );

        // try to find a valid position to start ray tracing
        Spatial::Point startPosition = findFree( position, Spatial::Point( rayDirection.x, rayDirection.y ) );

        logger().log(opencog::Logger::DEBUG, "LocalSpaceMap2D - rayDirection(%f,%f) target original position(%f, %f) target free position(%f, %f)", rayDirection.x, rayDirection.y, borderPoint.x, borderPoint.y, startPosition.first, startPosition.second );

        if ( eucDist( startPosition, position ) > distance ) {
            // there is no valid free point between start and goal positions
            continue;
        } // if

        Spatial::GridPoint collisionGridPoint;

        bool collided = false;
        rayTrace( snap( startPosition ), snap( endPosition ), CollisionDetector( const_cast<LocalSpaceMap2D*>( this ), collisionGridPoint, collided ) );

        if ( !collided ) {
            logger().log(opencog::Logger::DEBUG, "LocalSpaceMap2D - no collision => found free point at end position (%f,%f)", endPosition.first, endPosition.second );
            return endPosition;
        } // if

        if ( !gridIllegal( collisionGridPoint ) ) {
            float candidateGridPointDistance = eucDist( position, unsnap( collisionGridPoint ) );
            if ( freeGridPointFound ) {
                if ( candidateGridPointDistance > freeGridPointDistance ) {
                    freeGridPoint = collisionGridPoint;
                    freeGridPointDistance = candidateGridPointDistance;
                } // if
            } else {
                freeGridPoint = collisionGridPoint;
                freeGridPointFound = true;
                freeGridPointDistance = candidateGridPointDistance;
            } // if
        } // if

        if (freeGridPointFound && freeGridPointDistance > minimalDistanceThreshold) {
            // already found a satisfatory point. Don't need to check other candidates
            break;
        }

        rayDirection = rotationStep.rotate( rayDirection );
    } // for

    if ( !freeGridPointFound ) {
        throw opencog::NotFoundException( TRACE_INFO, "LocalSpaceMap2D - There is no free point near position (%f, %f ) at a distance of %f", position.first, position.second, distance );
    } // if

    return unsnap( freeGridPoint );

}

void LocalSpaceMap2D::saveASCIIMap(const std::string fileName) const
{
    FILE *fp = fopen(fileName.c_str(), "w");

    if (fp != NULL) {
        for (unsigned int i = 0; i < xDim(); i++) {
            if (i == 0) {
                fprintf(fp, "    ");
                for (unsigned int j = 0; j < yDim(); j++) fprintf(fp, " %u", j);
                fprintf(fp, "\n");
            }
            for (unsigned int j = 0; j < yDim(); j++) {
                if (j == 0) fprintf(fp, "%u:  ", i);
                fprintf(fp, " %c", gridOccupied(i, j) ? 'O' : '-');
            }
            fprintf(fp, "\n");
        }
        fclose(fp);
    } else logger().log(opencog::Logger::ERROR, "Error to create the file '%s'", fileName.c_str());
}



void LocalSpaceMap2D::calculateSegmentGridPoints( std::vector<Spatial::GridPoint>& points,  const Math::LineSegment& segment )
{

    Spatial::GridPoint startPoint( snap( Spatial::Point( segment.pointA.x, segment.pointA.y ) ) );
    Spatial::GridPoint endPoint( snap( Spatial::Point( segment.pointB.x, segment.pointB.y ) ) );

    logger().log(opencog::Logger::DEBUG, "LocalSpaceMap - Processing segment points grid points: start[%d %d] end[%d %d]", startPoint.first, startPoint.second, endPoint.first, endPoint.second );

    rayTrace( startPoint, endPoint, GridPointCollector( points ) );

}

void LocalSpaceMap2D::calculateObjectPoints( std::vector<Spatial::GridPoint>& points, const std::vector<Math::LineSegment>& segments )
{

    if ( segments.size( ) != 4 ) {
        logger().log(opencog::Logger::ERROR, "LocalSpaceMap - Unsupported object geometry. Only objects with 4 sides are supported. Object sides: %d", segments.size( ) );
        return;
    } // if

    std::map<int, std::pair<int, int> > limits;

    unsigned int i;
    for ( i = 0; i < segments.size( ); ++i ) {
        if ( segments[i].pointA.x == segments[i].pointB.x ) {
            continue;
        } // if

        std::vector<Spatial::GridPoint> segmentPoints;
        calculateSegmentGridPoints( segmentPoints, segments[i] );

        unsigned int j;
        for ( j = 0; j < segmentPoints.size( ); ++j ) {
            int currentLeftX = segmentPoints[ j ].first;
            int currentTopY = segmentPoints[ j ].second;

            std::map<int, std::pair<int, int> >::iterator it = limits.find( currentLeftX );

            if ( it == limits.end( ) ) {
                limits[ currentLeftX ] = std::pair<int, int>( currentTopY, currentTopY );
            } else if ( currentTopY < limits[ currentLeftX ].first ) {
                limits[ currentLeftX ] = std::pair<int, int>( currentTopY,
                                         limits[ currentLeftX ].first );
            } else if ( currentTopY > limits[ currentLeftX ].second ) {
                limits[ currentLeftX ].second = currentTopY;
            } // else
        } // for


    } // for


    std::map<int, std::pair<int, int> >::iterator it;
    for ( it = limits.begin( ); it != limits.end( ); ++it ) {

        if ( it->second.first != it->second.second ) {
            Spatial::Point pointA = unsnap( GridPoint( it->first, it->second.first ) );
            Spatial::Point pointB = unsnap( GridPoint( it->first, it->second.second ) );
            Math::LineSegment segment( Math::Vector3( pointA.first, pointA.second ),
                                       Math::Vector3( pointB.first, pointB.second ) );

            calculateSegmentGridPoints( points, segment );
        } else {
            points.push_back( Spatial::GridPoint( it->first, it->second.first ) );
        } // else
    } // for

}

void LocalSpaceMap2D::addObject( const Spatial::ObjectID& id, const Spatial::ObjectMetaData& metadata, bool isObstacle )
{

    long idHash = boost::hash<std::string>()( id );
    EntityPtr entity( new StaticEntity( idHash, id, Math::Vector3( metadata.centerX, metadata.centerY ), Math::Dimension3( metadata.width, metadata.height, metadata.length ), Math::Quaternion( Math::Vector3::Z_UNIT, metadata.yaw ), _radius ) );
    entity->setProperty( Entity::OBSTACLE, isObstacle );

    //  std::cout << entity->toString( ) << " MetaData: " << metadata.centerX << " " << metadata.centerY << " " << metadata.length << " " << metadata.width << " " << metadata.height << " " << metadata.yaw << " " << isObstacle << std::endl;
    if ( isObstacle ) {
        addToSuperEntity( entity );
    } // if

    const Math::BoundingBox& bb = entity->getBoundingBox( );

    std::vector<Math::LineSegment> bottomSegments;
    bottomSegments.push_back( Math::LineSegment( bb.getCorner( Math::BoundingBox::FAR_LEFT_BOTTOM ), bb.getCorner( Math::BoundingBox::FAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( Math::LineSegment( bb.getCorner( Math::BoundingBox::FAR_RIGHT_BOTTOM ), bb.getCorner( Math::BoundingBox::NEAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( Math::LineSegment( bb.getCorner( Math::BoundingBox::NEAR_RIGHT_BOTTOM ), bb.getCorner( Math::BoundingBox::NEAR_LEFT_BOTTOM ) ) );
    bottomSegments.push_back( Math::LineSegment( bb.getCorner( Math::BoundingBox::NEAR_LEFT_BOTTOM ), bb.getCorner( Math::BoundingBox::FAR_LEFT_BOTTOM ) ) );

    calculateObjectPoints( gridPoints[idHash], bottomSegments );
    this->entities.insert( LongEntityPtrHashMap::value_type( idHash, entity ) );

    const char* internalId = entity->getName( ).c_str( );
    logger().log(opencog::Logger::DEBUG, "LocalSpaceMap - Adding internal points to grid..." );

    const std::vector<GridPoint>& entityGridPoints = this->gridPoints[idHash];

    unsigned int i;
    for ( i = 0; i < entityGridPoints.size( ); ++i ) {
        if ( isObstacle ) {
            _grid[ entityGridPoints[i] ].insert( internalId );
        } else {
            _grid_nonObstacle[ entityGridPoints[i] ].insert( internalId );
        } // else
    } // for

}

void LocalSpaceMap2D::updateObject( const Spatial::ObjectID& id, const Spatial::ObjectMetaData& metadata, bool isObstacle )
{

    try {
        const EntityPtr& entity = getEntity( id );
        Spatial::ObjectMetaData metaData( entity->getPosition( ).x,
                                          entity->getPosition( ).y,
                                          entity->getLength( ),
                                          entity->getWidth( ),
                                          entity->getHeight( ),
                                          entity->getOrientation( ).getRoll( ) );
        if ( isObstacle == entity->getBooleanProperty( Entity::OBSTACLE ) && metaData == metadata ) {
            return;
        } // if

        removeObject(id);
        addObject(id, metadata, isObstacle );

    } catch ( opencog::NotFoundException& ex ) {
        // ignore
        return;
    } // catch
}

const EntityPtr& LocalSpaceMap2D::getEntity( const std::string& id ) const throw (opencog::NotFoundException)
{
    long idHash = boost::hash<std::string>()( id );
    LongEntityPtrHashMap::const_iterator it = this->entities.find( idHash );
    if ( it != this->entities.end( ) ) {
        return it->second;
    } // if
    throw opencog::NotFoundException( TRACE_INFO, "LocalSpaceMap2D - There is no entity named %s inside map", id.c_str( ) );
}

const EntityPtr& LocalSpaceMap2D::getEntity( long id ) const throw (opencog::NotFoundException)
{
    LongEntityPtrHashMap::const_iterator it = this->entities.find( id );
    if ( it != this->entities.end( ) ) {
        return it->second;
    } // if
    throw opencog::NotFoundException( TRACE_INFO, "LocalSpaceMap2D - There is no entity with id %d inside map", id );
}


bool LocalSpaceMap2D::belongsToSuperEntity( const Spatial::ObjectID& id ) const
{
    std::list<SuperEntityPtr>::const_iterator it;
    long idHash = boost::hash<std::string>()( id );
    for ( it = this->superEntities.begin( ); it != this->superEntities.end( ); ++it ) {
        if ( (*it)->containsEntity( idHash ) ) {
            return true;
        } // if
    } // for

    return false;
}

const SuperEntityPtr& LocalSpaceMap2D::getSuperEntityWhichContains( const Spatial::ObjectID& id ) const throw(opencog::NotFoundException)
{
    std::list<SuperEntityPtr>::const_iterator it;
    long idHash = boost::hash<std::string>()( id );
    for ( it = this->superEntities.begin( ); it != this->superEntities.end( ); ++it ) {
        if ( (*it)->containsEntity( idHash ) ) {
            return *it;
        } // if
    } // for

    throw opencog::NotFoundException( TRACE_INFO, "LocalSpaceMap2D - Given entity[id] does not belongs to a SuperEntity", id.c_str( ) );
}



/** ---------------------------------------------------------------------------
 * Struct rec_find
 * -----------------------------------------------------------------------------
 */
rec_find::rec_find(const Spatial::GridPoint& current,
                   const Spatial::GridMap& grid,
                   Spatial::ObjectIDSet& out,
                   const Spatial::LocalSpaceMap2D& parent) :
        _current(current), _g(current), _grid(grid), _out(out), _parent(&parent)
{

    //initialize walk_arround parameters
    step = 1;
    possible_step = step;
    last_x_difference = 0;
    last_y_difference = 0;

    // old method - buggy
    //rec();
}


bool rec_find::too_far() const
{
    return (LocalSpaceMap2D::eucDist(_g, _current) > 0 || //_radius ||
            !_parent->coordinatesAreOnGrid(_current.first, _current.second));
}

void rec_find::move_left()
{
    _current.first -= 1;
}

void rec_find::move_right()
{
    _current.first += 1;
}

void rec_find::move_up()
{
    _current.second += 1;
}

void rec_find::move_down()
{
    _current.second -= 1;
}

void rec_find::check_grid()
{
    // if there is something in this grid


    Spatial::GridMap::const_iterator it = _grid.find(_current);

    if (it != _grid.end())
        std::copy(it->second.begin(), it->second.end(), std::inserter(_out, _out.begin()));
}

void rec_find::johnnie_walker(Spatial::Distance d)
{

    check_grid();

    unsigned int distance = (unsigned int) (d * ( 2 * sqrt(2))) + 1;
    while (step < distance) {//!too_far()) {

        //    std::cout << "NEW LOOP x: \t" << _current.first << " y: \t" << _current.second << " step: " << step << "\n";

        //calculate next step
        possible_step = step - last_x_difference;
        //check grid bounds. I need save steps on x axis to check next move_left
        //xDim is number of x grids, not the last grid position
        if (_current.first + possible_step >= _parent->xDim()) {
            possible_step = (_parent->xDim() - 1) - _current.first;
            last_x_difference = step - possible_step;
        } else {
            last_x_difference = 0;
        }

        //go right
        if (last_y_difference == 0) {
            for (unsigned int i = 0; i < possible_step; i++) {
                move_right();
                check_grid();
            }
            //add position without check_grid.
        } else {
            _current.first += possible_step;
        }

        //calculate next step
        possible_step = step - last_y_difference;
        //check grid bounds. I need save steps on y axis to check next move_up
        if (_current.second - possible_step > _parent->yDim()) { //need compare with > yDim, because underflow
            possible_step = _current.second;
            last_y_difference = step - possible_step;
        } else {
            last_y_difference = 0;
        }

        //go down
        if (last_x_difference == 0) {
            for (unsigned int i = 0; i < possible_step; i++) {
                move_down();
                check_grid();
            }
            //subtract position without check_grid.
        } else {
            _current.second -= possible_step;
        }

        //increment step before move left
        step++;

        //calculate next step
        possible_step = step - last_x_difference;
        //check grid bounds. I need save steps on x axis to check next move_rigth
        if (_current.first - possible_step > _parent->xDim()) { //need compare with > yDim, because underflow
            possible_step = _current.first;
            last_x_difference = step - possible_step;
        } else {
            last_x_difference = 0;
        }

        //go left
        if (last_y_difference == 0) {
            for (unsigned int i = 0; i < possible_step; i++) {
                move_left();
                check_grid();
            }
            //subtract position without check_grid.
        } else {
            _current.first -= possible_step;
        }

        //calculate next step
        possible_step = step - last_y_difference;
        //check grid bounds. I need save steps on x axis to check next move_left
        //yDim is number of y grids, not the last grid position
        if (_current.second + possible_step >= _parent->yDim()) {
            possible_step = (_parent->yDim() - 1) - _current.second;
            last_y_difference = step - possible_step;
        } else {
            last_y_difference = 0;
        }

        //go up
        if (last_x_difference == 0) {
            for (unsigned int i = 0; i < possible_step; i++) {
                move_up();
                check_grid();
            }
        } else {
            _current.second += possible_step;
        }

        //increment step before move rigth
        step++;
    }
}

