/*
 * opencog/spatial/LocalSpaceMap2D.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Dan Zwell, Samir Araujo, Troy Huang
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

#include <opencog/spatial/LocalSpaceMap2D.h>

#include <opencog/spatial/TB_ASSERT.h>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/macros.h>

#include <opencog/spatial/math/Vector3.h>
#include <opencog/spatial/StaticEntity.h>
#include <opencog/spatial/Block.h>

#include <boost/lexical_cast.hpp>
#include <boost/type_traits/make_signed.hpp>

#include <fstream>
#include <map>
#include <algorithm>

#define HUGE_DISTANCE 999999.9

using namespace opencog;
using namespace opencog::spatial;
using boost::make_signed;

const double LocalSpaceMap2D::NEXT_FACTOR = 0.1;
const double LocalSpaceMap2D::NEAR_FACTOR = 0.003125;

bool LocalSpaceMap2D::addToSuperEntity( const EntityPtr& entity )
{
    bool merged = false;
    LongEntityPtrHashMap::iterator it2;
    SuperEntityPtr previousSuperEntity;

    logger().debug("LocalSpaceMap2D::addToSuperEntity Verifying entity[%s - %ld]", entity->getName( ).c_str( ), entity->getId( ) );

    std::map< long, SuperEntityPtr > intersection;

    for ( it2 = this->entities.begin( ); it2 != this->entities.end( ); ++it2 ) {
        if ( it2->first == entity->getId( ) ) {
            continue;
        } // if

        //std::cout << "inspecting " << it2->first << std::endl;

        if ( it2->second->intersects( *entity ) ) {
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
bool LocalSpaceMap2D::outsideMap( const std::vector<spatial::math::LineSegment>& segments )
{
    bool outside = true;

    unsigned int i;
    for ( i = 0; outside && i < segments.size( ); ++i ) {
        const math::Vector3& point = segments[i].pointA;

        if ( point.x  >= _xMin && point.x  <= _xMax && point.y >= _yMin && point.y <= _yMax ) {
            outside = false;
        } // if
    } // for

    return outside;
}


LocalSpaceMap2D& LocalSpaceMap2D::operator=(const LocalSpaceMap2D& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "LocalSpaceMap2D - Cannot copy an object of this class");
}

LocalSpaceMap2D::LocalSpaceMap2D(const LocalSpaceMap2D& other) 
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "LocalSpaceMap2D - Cannot copy an object of this class");
}

/** ---------------------------------------------------------------------------
 * Public Functions
 * ----------------------------------------------------------------------------
 */
LocalSpaceMap2D::LocalSpaceMap2D(spatial::Distance xMin, spatial::Distance xMax, unsigned int xDim,
                                 spatial::Distance yMin, spatial::Distance yMax, unsigned int yDim,
                                 spatial::Distance radius,spatial::Distance agentHeight,
                                 spatial::Distance floor):
        _xMin(xMin), _xMax(xMax), _xDim(xDim),
        _yMin(yMin), _yMax(yMax), _yDim(yDim),
        _radius(radius)
{
    _floorHeight = floor;
    _agentHeight = agentHeight;
    Distance xDelta = xMax - xMin;
    Distance yDelta = yMax - yMin;
    _diagonalSize = sqrt(xDelta * xDelta + yDelta * yDelta);
}

LocalSpaceMap2D::~LocalSpaceMap2D()
{
}

LocalSpaceMap2D* LocalSpaceMap2D::clone() const
{
    LocalSpaceMap2D* clonedMap = new LocalSpaceMap2D(_xMin, _xMax, _xDim,
            _yMin, _yMax, _xDim, _radius, _agentHeight, _floorHeight);
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
            other._agentHeight != _agentHeight ||
            other._floorHeight != _floorHeight ||
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
        LongEntityPtrHashMap::const_iterator other_it = other.entities.find( it->first );
        if ( other_it == other.entities.end( ) ) {
            //std::cout << it->first << " WAS NOT FOUND IN SECOND MAP" << std::endl;
            return false;
        } else if (*(other_it->second) != *(it->second)) { 
            //std::cout << "ENTITY " << it->first << " IS DIFFERENT FROM WHAT IS IN THE SECOND MAP" << std::endl;
            return false; 
        }
    } 
    return true;

}

void LocalSpaceMap2D::save( FILE* fp ) const
{
    unsigned int numberOfObjects = this->entities.size();
    fwrite( &numberOfObjects, sizeof(unsigned int), 1, fp );

    LongEntityPtrHashMap::const_iterator it;
    for ( it = this->entities.begin( ); it != this->entities.end( ); ++it ) {
        std::string id = it->second->getName( );
        unsigned int length = id.size();

        fwrite(&length, sizeof(unsigned int), 1, fp);
        fwrite(id.c_str(), sizeof(char), length, fp);


        spatial::ObjectMetaData metaData( it->second->getPosition( ).x,
                                          it->second->getPosition( ).y,
                                          it->second->getPosition( ).z,
                                          it->second->getLength( ),
                                          it->second->getWidth( ),
                                          it->second->getHeight( ),
                                          it->second->getOrientation( ).getRoll( ) );

        bool isObstacle = it->second->getBooleanProperty( Entity::OBSTACLE );

        fwrite(&metaData, sizeof(spatial::ObjectMetaData), 1, fp);
        fwrite(&isObstacle, sizeof(bool), 1, fp );
    }
}

void LocalSpaceMap2D::load(FILE* fp)
{
    unsigned int numberOfObjects;
    bool b_read = true;
    FREAD_CK(&numberOfObjects, sizeof(unsigned int), 1, fp);

    for (unsigned int i = 0; i < numberOfObjects; ++i)
    {
        unsigned int length;
        FREAD_CK(&length, sizeof(unsigned int), 1, fp);

        char* id = new char[length+1];
        FREAD_CK(id, sizeof(char), length, fp);
        id[length] = '\0';

        spatial::ObjectMetaData metadata;
        FREAD_CK(&metadata, sizeof(spatial::ObjectMetaData), 1, fp);

        bool isObstacle;
        FREAD_CK(&isObstacle, sizeof(bool), 1, fp);

        addObject(std::string(id), metadata, isObstacle );

        delete [] id;
    }
    CHECK_FREAD;
}

spatial::Distance LocalSpaceMap2D::xGridWidth() const
{
    return (_xMax -_xMin) / Distance(_xDim);
}

spatial::Distance LocalSpaceMap2D::yGridWidth() const
{
    return (_yMax -_yMin) / Distance(_yDim);
}

spatial::Distance LocalSpaceMap2D::xMin() const
{
    return _xMin;
}

spatial::Distance LocalSpaceMap2D::xMax() const
{
    return _xMax;
}

unsigned int LocalSpaceMap2D::xDim() const
{
    return _xDim;
}

spatial::Distance LocalSpaceMap2D::yMin() const
{
    return _yMin;
}

spatial::Distance LocalSpaceMap2D::yMax() const
{
    return _yMax;
}

unsigned int LocalSpaceMap2D::yDim() const
{
    return _yDim;
}

spatial::Distance LocalSpaceMap2D::diagonalSize() const
{
    return _diagonalSize;
}

spatial::Distance LocalSpaceMap2D::radius() const
{
    return _radius;
}

spatial::Distance LocalSpaceMap2D::agentHeight() const
{
    return _agentHeight;
}

spatial::Distance LocalSpaceMap2D::floorHeight() const
{
    return _floorHeight;
}

bool LocalSpaceMap2D::illegal(const spatial::Point& pt) const
{
    if (pt.first < _xMin || pt.first > _xMax || pt.second < _yMin || pt.second > _yMax) {
        logger().fine("LocalSpaceMap - illegal(pt = (%f, %f)): out of the map!", pt.first, pt.second);
        return true;
    }
    return gridIllegal(snap(pt));
}

spatial::Point LocalSpaceMap2D::getNearestFreePoint(const spatial::Point& pt) const throw (opencog::RuntimeException, std::bad_exception)
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

spatial::Point3D LocalSpaceMap2D::getNearestFree3DPoint(const spatial::Point3D& pt, double delta) const throw (opencog::RuntimeException, std::bad_exception)
{
    double destAltitude = pt.get<2>();

    GridPoint gp = snap(spatial::Point(pt.get<0>(), pt.get<1>()));
    if (!gridIllegal(gp)) {
        return spatial::Point3D(pt.get<0>(), pt.get<1>(), _floorHeight);
    } 

    //printf("Original point (%lf, %lf). lower x:%u, lower y:%u, upper x:%u, upper y:%u.\n",
    //       pt.get<0>(), pt.get<1>(), lower_x, lower_y, upper_x, upper_y);

    // Check if there's some point to reach that goal.
    bool foundFreePoint = false;
    spatial::GridPoint nearestFreePoint;
    double nearestAltitude = HUGE_DISTANCE;

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
                double altitude = getProperFreePointAltitude(spatial::GridPoint(lower_x, y), pt, delta);
                if (altitude + _agentHeight >= destAltitude) {
                    if (!foundFreePoint ||
                        std::abs(altitude - destAltitude) < std::abs(nearestAltitude - destAltitude)) {
                        nearestAltitude = altitude;
                        nearestFreePoint = spatial::GridPoint(lower_x, y);
                        foundFreePoint = true;
                    }
                } else {
                    if (!foundFreePoint &&
                        std::abs(altitude - destAltitude) < std::abs(nearestAltitude - destAltitude)) {
                        nearestAltitude = altitude;
                        nearestFreePoint = spatial::GridPoint(lower_x, y);
                    }
                }
            }
        }

        if (valid_lower_y) {
            for (unsigned int x = lower_x; x <= upper_x; x++) {
                double altitude = getProperFreePointAltitude(spatial::GridPoint(x, lower_y), pt, delta);
                if (altitude + _agentHeight >= destAltitude) {
                    if (!foundFreePoint ||
                        std::abs(altitude - destAltitude) < std::abs(nearestAltitude - destAltitude)) {
                        nearestAltitude = altitude;
                        nearestFreePoint = spatial::GridPoint(x, lower_y);
                        foundFreePoint = true;
                    }
                } else {
                    if (!foundFreePoint &&
                        std::abs(altitude - destAltitude) < std::abs(nearestAltitude - destAltitude)) {
                        nearestAltitude = altitude;
                        nearestFreePoint = spatial::GridPoint(x, lower_y);
                    }
                }
            }
        }

        if (valid_upper_x) {
            for (unsigned int y = lower_y; y <= upper_y; y++) {
                double altitude = getProperFreePointAltitude(spatial::GridPoint(upper_x, y), pt, delta);
                if (altitude + _agentHeight >= destAltitude) {
                    if (!foundFreePoint ||
                        std::abs(altitude - destAltitude) < std::abs(nearestAltitude - destAltitude)) {
                        nearestAltitude = altitude;
                        nearestFreePoint = spatial::GridPoint(upper_x, y);
                        foundFreePoint = true;
                    }
                } else {
                    if (!foundFreePoint &&
                        std::abs(altitude - destAltitude) < std::abs(nearestAltitude - destAltitude)) {
                        nearestAltitude = altitude;
                        nearestFreePoint = spatial::GridPoint(upper_x, y);
                    }
                }
            }
        }

        if (valid_upper_y) {
            for (unsigned int x = lower_x; x <= upper_x; x++) {
                double altitude = getProperFreePointAltitude(spatial::GridPoint(x, upper_y), pt, delta);
                if (altitude + _agentHeight >= destAltitude) {
                    if (!foundFreePoint ||
                        std::abs(altitude - destAltitude) < std::abs(nearestAltitude - destAltitude)) {
                        nearestAltitude = altitude;
                        nearestFreePoint = spatial::GridPoint(x, upper_y);
                        foundFreePoint = true;
                    }
                } else {
                    if (!foundFreePoint &&
                        std::abs(altitude - destAltitude) < std::abs(nearestAltitude - destAltitude)) {
                        nearestAltitude = altitude;
                        nearestFreePoint = spatial::GridPoint(x, upper_y);
                    }
                }
            }
        }
    }

    if (!foundFreePoint) { // Goal point can not be reached from free point.
        throw opencog::RuntimeException(TRACE_INFO,
                                        "LocalSpaceMap2D - Could not find the nearest valid grid point from (%u,%u)",
                                        pt.get<0>(), pt.get<1>());
    }


    spatial::Point freePoint2D = unsnap(nearestFreePoint);
    spatial::Point3D freePoint3D = spatial::Point3D(freePoint2D.first, freePoint2D.second, nearestAltitude);

    if (std::abs(nearestAltitude - destAltitude) <= _agentHeight) { // at the same altitude level
        return pt;
    }
    return freePoint3D;
}


double LocalSpaceMap2D::getProperFreePointAltitude(const spatial::GridPoint& gp, const spatial::Point3D& dest, double delta) const
{
    bool is_obstacle = gridOccupied(gp);
    double destHeight = dest.get<2>();
    if (!is_obstacle) {
        // None obstacle, then it has to be the floor.
        return _floorHeight;
    }

    double topSurface = _floorHeight;
    std::vector<spatial::Gradient> grads = getObjectGradientsByGridPoint(gp);
    std::vector<spatial::Gradient>::const_iterator it = grads.begin();
    for (; it != grads.end(); ++it) {
        if (topSurface + _agentHeight > it->first) {
            // The agent is not able to stand on this point because the roof of
            // this point can not cover the agent.
            topSurface = (topSurface > it->second) ? topSurface : it->second;
            continue;
        }
        if (topSurface + _agentHeight > destHeight || topSurface + delta >= destHeight) {
            // Now that from the altitude that it is standing on, the agent is
            // able to reach the goal altitude, we think it is a proper
            // altitude.
            break;
        } else {
            topSurface = (topSurface > it->second) ? topSurface : it->second;
        }
    }

    return topSurface;
}

bool LocalSpaceMap2D::gridIllegal(const spatial::GridPoint& gp) const
{
    return gridIllegal(gp.first, gp.second);
}

bool LocalSpaceMap2D::gridIllegal(unsigned int i, unsigned int j) const
{
    bool result;
    if ((result = gridOccupied(i, j))) {
        logger().fine("LocalSpaceMap - gridIllegal(%u, %u): grid occupied by an object!", i, j);
    } else if ((result = !coordinatesAreOnGrid(i, j))) {
        logger().fine("LocalSpaceMap - gridIllegal(%u, %u): coordinates not on grid!", i, j);
    }
    return result;
}

bool LocalSpaceMap2D::gridOccupied(const spatial::GridPoint& gp) const
{
    spatial::GridMap::const_iterator itr = _grid.find(gp);
    return (itr != _grid.end() && !(itr->second.empty()));
}

bool LocalSpaceMap2D::gridOccupied(unsigned int i, unsigned int j) const
{
    return (gridOccupied(GridPoint(i, j)));
}

bool LocalSpaceMap2D::gridOccupied_nonObstacle(const spatial::GridPoint& gp) const
{
    spatial::GridMap::const_iterator itr = _grid_nonObstacle.find(gp);
    return (itr != _grid_nonObstacle.end() && !(itr->second.empty()));
}

bool LocalSpaceMap2D::gridOccupied_nonObstacle(unsigned int i, unsigned int j) const
{
    return (gridOccupied_nonObstacle(GridPoint(i, j)));
}

bool LocalSpaceMap2D::edgeIllegal(const spatial::GridPoint &src, const spatial::GridPoint &dest, double srcHeight, double delta) const
{
    if (!coordinatesAreOnGrid(dest.first, dest.second)) {
        logger().fine("LocalSpaceMap - gridIllegal(%u, %u): coordinates not on grid!", dest.first, dest.second);
        return true;
    }

    if (isDiagonal(src, dest)) {
        bool illegal = this->edgeIllegal(src, spatial::GridPoint(src.first, dest.second), srcHeight, 0.0001)
                        || this->edgeIllegal(src, spatial::GridPoint(dest.first, src.second), srcHeight, 0.0001);
        if (illegal) return true;
    }
    
    if (getProperDestAltitude(src, dest, srcHeight, delta) > 0) return false;

    return true;
}

double LocalSpaceMap2D::getProperDestAltitude(const spatial::GridPoint &src, const spatial::GridPoint &dest, const double srcHeight, double delta) const
{
    bool is_obstacle = gridOccupied(dest.first, dest.second);
    if (!is_obstacle) {
        // None obstacle, OK
        return _floorHeight;
    } else {
        double topSurface = _floorHeight;
        std::vector<spatial::Gradient> grads = getObjectGradientsByGridPoint(dest);
        std::vector<spatial::Gradient>::const_iterator it = grads.begin();
        for (; it != grads.end(); ++it) {
            if (srcHeight >= it->first) {
                // bottom surface is lower than src altitude, then the altitude of
                // top surface should be no smaller than its opposite surface.
                topSurface = (topSurface > it->second) ? topSurface : it->second;
                continue;
            }
            if (topSurface + _agentHeight <= it->first) {
                break;
            } else {
                topSurface = (topSurface > it->second) ? topSurface : it->second;
            }
        }

        // Check if the agent can reach that height.
        if (srcHeight + delta >= topSurface) {
            return topSurface;
        }

        return -1;
    }
}


bool LocalSpaceMap2D::isDiagonal(const spatial::GridPoint &src, const spatial::GridPoint &dest) const
{
    // Nil: I've added that because otherwise abs(unsigned int) is
    // ambiguious. Using boost::make_signed so the sign-ification is
    // generic in case the type of GridPoint is modified.
    typedef make_signed<GridPoint::first_type>::type SInt;
    return (std::abs((SInt)dest.first - (SInt)src.first) == std::abs((SInt)dest.second - (SInt)src.second));
}


spatial::ObjectID LocalSpaceMap2D::getTallestObjectInGrid(const GridPoint &gp) const
{
    if (!gridOccupied(gp))
        return "";
    double height = 0.0;
    spatial::ObjectID id;
    const spatial::ObjectInfoSet& objInfoSet = _grid.at(gp);
    spatial::ObjectInfoSet::const_iterator it = objInfoSet.begin();
    for (; it != objInfoSet.end(); ++it) {
        if (it->isExtraBoundary) continue;
        spatial::ObjectID entityId = spatial::ObjectID(it->id);
        const EntityPtr& entityPtr = getEntity(entityId);
        double tmp_height = entityPtr->getPosition().z + 0.5 * entityPtr->getHeight();
        if (tmp_height > height) {
            height = tmp_height;
            id = entityId;
        }
    }
    return id;
}

std::vector<spatial::Gradient> LocalSpaceMap2D::getObjectGradientsByGridPoint(const GridPoint& gp) const
{
    std::vector<spatial::Gradient> grads;
    if (!gridOccupied(gp))
        return grads;
    const spatial::ObjectInfoSet& objInfoSet = _grid.at(gp);
    spatial::ObjectInfoSet::const_iterator it = objInfoSet.begin();
    for (; it != objInfoSet.end(); ++it) {
        if (it->isExtraBoundary) {
            continue;
        }
        spatial::ObjectID entityId = spatial::ObjectID(it->id);
        const EntityPtr& entityPtr = getEntity(entityId);
        spatial::Altitude altitude = entityPtr->getPosition().z;
        double semiHeight = 0.5 * entityPtr->getHeight();
        grads.push_back(spatial::Gradient(altitude - semiHeight, altitude + semiHeight));
    }

    // Sort the grads by the altitude of bottom surface
    spatial::gradient_cmp_by_lower_altitude gradient_cmp;
    std::sort(grads.begin(), grads.end(), gradient_cmp);

    return grads;
}

bool LocalSpaceMap2D::containsObject(const spatial::ObjectID& id) const
{
    try {
        getEntity( id );
        return true;
    } catch ( opencog::NotFoundException& ex ) {
        return false;
    } // catch
}

bool LocalSpaceMap2D::isObstacle(const spatial::ObjectID& id) const
{
    try {
        return getEntity( id )->getBooleanProperty( Entity::OBSTACLE );
    } catch ( opencog::NotFoundException& ex ) {
        return false;
    } // catch
}

bool LocalSpaceMap2D::isNonObstacle(const spatial::ObjectID& id) const
{
    return !isObstacle( id );
}

const std::vector<spatial::GridPoint>& LocalSpaceMap2D::getObjectPoints(const spatial::ObjectID& id) const  throw(opencog::NotFoundException)
{
    long idHash = boost::hash<std::string>()( id );
    LongGridPointVectorHashMap::const_iterator it = this->gridPoints.find( idHash );
    if ( it == this->gridPoints.end( ) ) {
        throw opencog::NotFoundException( TRACE_INFO, "LocalSpaceMap2D - There is no object %s", id.c_str() );
    } // if
    return it->second;
}

spatial::Point LocalSpaceMap2D::getNearestObjectPoint( const spatial::Point& referencePoint, const spatial::ObjectID& objectID ) const throw (opencog::NotFoundException)
{

    if ( !containsObject( objectID ) ) {
        throw opencog::NotFoundException( TRACE_INFO, "LocalSpaceMap2D - There is no object %s", objectID.c_str() );
    } // if

    const EntityPtr& entity = getEntity( objectID );
    const math::BoundingBox& bb = entity->getBoundingBox( );

    std::vector<math::LineSegment> bottomSegments;
    bottomSegments.push_back( math::LineSegment( bb.getCorner( math::BoundingBox::FAR_LEFT_BOTTOM ), bb.getCorner( math::BoundingBox::FAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( bb.getCorner( math::BoundingBox::FAR_RIGHT_BOTTOM ), bb.getCorner( math::BoundingBox::NEAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( bb.getCorner( math::BoundingBox::NEAR_RIGHT_BOTTOM ), bb.getCorner( math::BoundingBox::NEAR_LEFT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( bb.getCorner( math::BoundingBox::NEAR_LEFT_BOTTOM ), bb.getCorner( math::BoundingBox::FAR_LEFT_BOTTOM ) ) );


    math::Vector3 point( referencePoint.first, referencePoint.second );
    //const ObjectMetaData& md = getMetaData(objectID);
    //math::Vector3 nearestPoint( md.centerX, md.centerY);
    math::Vector3 nearestPoint( entity->getPosition( ) );

    double minDistance = std::numeric_limits<double>::max( );

    //const std::vector<spatial::math::LineSegment>& segments = it->second.borderSegments;
    for ( unsigned int i = 0; i < bottomSegments.size( ); ++i ) {
        //math::Vector3 candidatePoint = getNearestPointInSegment( point, segments[i] );
        math::Vector3 candidatePoint = bottomSegments[i].nearestPointInSegment( point );
        double candidateDistance = ( candidatePoint - point ).length( );

        if ( candidateDistance < minDistance ) {
            minDistance = candidateDistance;
            nearestPoint = candidatePoint;
        } // if
    } // for
    return spatial::Point( nearestPoint.x, nearestPoint.y );
}

void LocalSpaceMap2D::copyObjects(const LocalSpaceMap2D& otherMap)
{
    //ObjectHashMap::const_iterator it;
    LongEntityPtrHashMap::const_iterator it;

    for ( it = otherMap.entities.begin( ); it != otherMap.entities.end( ); ++it ) {
        spatial::ObjectMetaData metaData( it->second->getPosition( ).x,
                                          it->second->getPosition( ).y,
                                          it->second->getPosition( ).z,
                                          it->second->getLength( ),
                                          it->second->getWidth( ),
                                          it->second->getHeight( ),
                                          it->second->getOrientation( ).getRoll( ),
                                          it->second->getStringProperty( Entity::ENTITY_CLASS ));
        addObject( it->second->getName( ), metaData, it->second->getBooleanProperty( Entity::OBSTACLE ) );
    } // for
}

void LocalSpaceMap2D::removeObject(const spatial::ObjectID& id)
{
    logger().fine("LocalSpaceMap2D::removeObject(%s)", id.c_str());

    //ObjectHashMap::iterator it = this->objects.find( id );
    long idHash = boost::hash<std::string>()( id );
    //if ( it == this->objects.end( ) ) {
    LongEntityPtrHashMap::iterator it = this->entities.find( idHash );
    if ( it == this->entities.end( ) ) {
        logger().fine("LocalSpaceMap2D - There is no object %s on this map", id.c_str( ) );
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

    ObjectInfo info(internalId, true);
    unsigned int i;
    for ( i = 0; i < entityGridPoints.size( ); ++i ) {
        //if ( it->second.isObstacle ) {
        if ( isObstacle ) {
            ObjectInfoSet::iterator obj_info_it = _grid[ entityGridPoints[i] ].find(info);
            if (obj_info_it != _grid[ entityGridPoints[i] ].end()) {
                _grid[ entityGridPoints[i] ].erase(obj_info_it);
            }
            if ( _grid[ entityGridPoints[i] ].size( ) == 0 ) {
                _grid.erase( entityGridPoints[i] );
            } // if
        } else {
            ObjectInfoSet::iterator obj_info_it = _grid_nonObstacle[ entityGridPoints[i] ].find(info);
            if (obj_info_it != _grid_nonObstacle[ entityGridPoints[i] ].end()) {
                _grid_nonObstacle[ entityGridPoints[i] ].erase(obj_info_it);
            }
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

Distance LocalSpaceMap2D::minDist(const spatial::ObjectID& id, const spatial::Point& p) const
{
    std::vector<spatial::Point> all;
    allPoints(id, back_inserter(all));
    if (all.empty()) {
        logger().error(
                     "LocalSpaceMap2D::minDist(): No point associated to obj '%s'.",
                     id.c_str());

        // return a huge distance soh this object is discarded
        return (HUGE_DISTANCE);
    }
    //TB_ASSERT(!all.empty());
    Distance d = eucDist(p, all.front());
    for (std::vector<spatial::Point>::const_iterator it = ++all.begin();it != all.end();++it) {
        d = std::min(d, eucDist(p, *it));
    }
    return d;
}

bool LocalSpaceMap2D::coordinatesAreOnGrid(unsigned int x, unsigned int y) const
{
    return (x < _xDim && y < _yDim);
}

spatial::Point LocalSpaceMap2D::nearbyPoint(const spatial::Point& src, const spatial::ObjectID& id) const
{

    // getNearestObjectPoint will throw an exception if the object isn't inside the map
    spatial::Point closestPoint = getNearestObjectPoint( src, id );

    return closestPoint; // we don't need to invoke getNearestFreePoint here, since that will be triggered
                         // when getting way points.
    //return getNearestFreePoint(closestPoint); //Suggested by Welter
    //return findFree(closestPoint, Point(src.first - closestPoint.first,
    //         src.second - closestPoint.second));
}

spatial::Point LocalSpaceMap2D::behindPoint(const spatial::Point& src, const spatial::ObjectID& id) const
{
    Point loc = centerOf(id);

    //check for the degenerate case
    if (snap(loc) == snap(src))
        return loc;

    return findFree(loc, Point(loc.first - src.first, loc.second - src.second));
}

spatial::Point LocalSpaceMap2D::findFree(const spatial::Point& p, const spatial::Point& direction) const
{
    //normalize the step
    spatial::Distance d = 2.0 * eucDist(Point(0, 0), direction) / (xGridWidth() + yGridWidth());
    spatial::Point normDirection(direction.first / d, direction.second / d);
    spatial::Point result = p;

    spatial::GridPoint g = snap(result);
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

spatial::Point LocalSpaceMap2D::centerOf(const spatial::ObjectID& id) const
{
    spatial::Point center(0, 0);
    try {
        const EntityPtr& entity = getEntity( id );
        center.first = entity->getPosition( ).x;
        center.second = entity->getPosition( ).y;
    } catch ( opencog::NotFoundException& ex ) {
        // ignore
    } // catch
    return center;
}

spatial::GridPoint LocalSpaceMap2D::snap(const spatial::Point& p) const
{
    spatial::GridPoint gp =
        spatial::GridPoint(p.first <= _xMin ? 0 : p.first >= _xMax ? _xDim - 1 :
                           (unsigned int)((p.first - _xMin) / xGridWidth()),
                           p.second <= _yMin ? 0 : p.second >= _yMax ? _yDim - 1 :
                           (unsigned int)((p.second - _yMin) / yGridWidth()));
    return gp;
}

spatial::Point LocalSpaceMap2D::unsnap(const spatial::GridPoint& g) const
{
    return spatial::Point((spatial::Distance(g.first) + 0.5) * xGridWidth() + _xMin,
                          (spatial::Distance(g.second) + 0.5) * yGridWidth() + _yMin);
}

spatial::Point LocalSpaceMap2D::getNearFreePointAtDistance( const spatial::Point& position, float distance, const spatial::Point& startDirection ) const throw (opencog::NotFoundException)
{
    // do a raytrace to
    unsigned int step = 5;
    math::Quaternion rotationStep( math::Vector3::Z_UNIT, step * M_PI / 180.0 );

    spatial::math::Vector3 rayDirection( startDirection.first, startDirection.second );
    rayDirection.normalise( );
    spatial::GridPoint freeGridPoint;
    float freeGridPointDistance = 0;
    bool freeGridPointFound = false;
    float minimalDistanceThreshold = (distance * 0.90); // 90% of the distance is enough

    unsigned int i;
    for ( i = 0; i < 360; i += step ) {

        spatial::math::Vector3 borderPoint = ( rayDirection * distance ) +
                                             spatial::math::Vector3( position.first, position.second );

        spatial::Point endPosition( borderPoint.x, borderPoint.y );

        // try to find a valid position to start ray tracing
        spatial::Point startPosition = findFree( position, spatial::Point( rayDirection.x, rayDirection.y ) );

        logger().debug("LocalSpaceMap2D - rayDirection(%f,%f) target original position(%f, %f) target free position(%f, %f)", rayDirection.x, rayDirection.y, borderPoint.x, borderPoint.y, startPosition.first, startPosition.second );

        if ( eucDist( startPosition, position ) > distance ) {
            // there is no valid free point between start and goal positions
            continue;
        } // if

        spatial::GridPoint collisionGridPoint;

        bool collided = false;
        rayTrace( snap( startPosition ), snap( endPosition ), CollisionDetector( const_cast<LocalSpaceMap2D*>( this ), collisionGridPoint, collided ) );

        if ( !collided ) {
            logger().debug("LocalSpaceMap2D - no collision => found free point at end position (%f,%f)", endPosition.first, endPosition.second );
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

void LocalSpaceMap2D::calculateSegmentGridPoints( std::vector<spatial::GridPoint>& points,  const math::LineSegment& segment )
{

    spatial::GridPoint startPoint( snap( spatial::Point( segment.pointA.x, segment.pointA.y ) ) );
    spatial::GridPoint endPoint( snap( spatial::Point( segment.pointB.x, segment.pointB.y ) ) );

    logger().debug("LocalSpaceMap - Processing segment points grid points: start[%d %d] end[%d %d]", startPoint.first, startPoint.second, endPoint.first, endPoint.second );

    rayTrace( startPoint, endPoint, GridPointCollector( points ) );

}

void LocalSpaceMap2D::calculateObjectPoints( std::vector<spatial::GridPoint>& points, const std::vector<math::LineSegment>& segments )
{

    if ( segments.size( ) != 4 ) {
        logger().error("LocalSpaceMap - Unsupported object geometry. Only objects with 4 sides are supported. Object sides: %d", segments.size( ) );
        return;
    } // if

    std::map<int, std::pair<int, int> > limits;

    unsigned int i;
    for ( i = 0; i < segments.size( ); ++i ) {
        if ( segments[i].pointA.x == segments[i].pointB.x ) {
            continue;
        } // if

        std::vector<spatial::GridPoint> segmentPoints;
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
            spatial::Point pointA = unsnap( GridPoint( it->first, it->second.first ) );
            spatial::Point pointB = unsnap( GridPoint( it->first, it->second.second ) );
            math::LineSegment segment( math::Vector3( pointA.first, pointA.second ),
                                       math::Vector3( pointB.first, pointB.second ) );

            calculateSegmentGridPoints( points, segment );
        } else {
            points.push_back( spatial::GridPoint( it->first, it->second.first ) );
        } // else
    } // for

}

void LocalSpaceMap2D::addObject( const spatial::ObjectID& id, const spatial::ObjectMetaData& metadata, bool isObstacle )
{

    // TODO: Rely on hash code of the ID as primary key for internal structures
    // (entities, gridPoints, etc) may fail eventually, since hash value
    // colision may happen eventually for different IDs. We should review this
    // by using simpler types for IDs (Atom Handles, for example) and replacing
    // idHash by these simpler IDs in these internal structures.
    long idHash = boost::hash<std::string>()( id );
    EntityPtr entity( new StaticEntity( idHash, id, math::Vector3( metadata.centerX, metadata.centerY, metadata.centerZ ), math::Dimension3( metadata.width, metadata.height, metadata.length ), math::Quaternion( math::Vector3::Z_UNIT, metadata.yaw ), _radius ) );
    entity->setProperty( Entity::OBSTACLE, isObstacle );
    entity->setProperty( Entity::ENTITY_CLASS, metadata.entityClass);

    if ( isObstacle ) {
        addToSuperEntity(entity);
    } // if

    const math::BoundingBox& bb = entity->getBoundingBox( );

    std::vector<math::LineSegment> bottomSegments;
    bottomSegments.push_back( math::LineSegment( bb.getCorner( math::BoundingBox::FAR_LEFT_BOTTOM ), bb.getCorner( math::BoundingBox::FAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( bb.getCorner( math::BoundingBox::FAR_RIGHT_BOTTOM ), bb.getCorner( math::BoundingBox::NEAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( bb.getCorner( math::BoundingBox::NEAR_RIGHT_BOTTOM ), bb.getCorner( math::BoundingBox::NEAR_LEFT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( bb.getCorner( math::BoundingBox::NEAR_LEFT_BOTTOM ), bb.getCorner( math::BoundingBox::FAR_LEFT_BOTTOM ) ) );

    calculateObjectPoints( gridPoints[idHash], bottomSegments );
    this->entities.insert( LongEntityPtrHashMap::value_type( idHash, entity ) );

    const char* internalId = entity->getName( ).c_str( );
    logger().debug("LocalSpaceMap - Adding internal points to grid..." );

    const std::vector<GridPoint>& entityGridPoints = this->gridPoints[idHash];

    unsigned int i;
    for ( i = 0; i < entityGridPoints.size( ); ++i ) {
        ObjectInfo info(internalId, false);
        if ( isObstacle ) {
            _grid[ entityGridPoints[i] ].insert(info);
        } else {
            _grid_nonObstacle[ entityGridPoints[i] ].insert(info);
        } // else
    } // for

}

void LocalSpaceMap2D::addBlock(const ObjectID& id, const ObjectMetaData& metadata)
{
    long idHash = boost::hash<std::string>()(id);
    EntityPtr entity(new Block(idHash, id, math::Vector3(metadata.centerX, metadata.centerY, metadata.centerZ), math::Dimension3(metadata.width, metadata.height, metadata.length), math::Quaternion(math::Vector3::Z_UNIT, metadata.yaw), _radius));
    BlockPtr block = boost::static_pointer_cast<Block>(entity);
    entity->setProperty(Entity::OBSTACLE, true);
    entity->setProperty(Entity::ENTITY_CLASS, metadata.entityClass);

    addToSuperEntity(entity);

    // the real area that the object actually occupies.
    std::vector<spatial::GridPoint> solidArea;
    // the expansion area that the extra boundary occupies.
    std::vector<spatial::GridPoint> expansionArea;

    std::vector<math::LineSegment> bottomSegments;

    // get the solid boundingbox bottom segments.
    const math::BoundingBox& innerBounding = block->getSolidBoundingBox();
    bottomSegments.push_back( math::LineSegment( innerBounding.getCorner( math::BoundingBox::FAR_LEFT_BOTTOM ), innerBounding.getCorner( math::BoundingBox::FAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( innerBounding.getCorner( math::BoundingBox::FAR_RIGHT_BOTTOM ), innerBounding.getCorner( math::BoundingBox::NEAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( innerBounding.getCorner( math::BoundingBox::NEAR_RIGHT_BOTTOM ), innerBounding.getCorner( math::BoundingBox::NEAR_LEFT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( innerBounding.getCorner( math::BoundingBox::NEAR_LEFT_BOTTOM ), innerBounding.getCorner( math::BoundingBox::FAR_LEFT_BOTTOM ) ) );

    calculateObjectPoints(solidArea, bottomSegments);

    // get the expanding boundingbox bottom segments.
    const math::BoundingBox& expansionBounding = entity->getBoundingBox( );
    bottomSegments.clear();
    bottomSegments.push_back( math::LineSegment( expansionBounding.getCorner( math::BoundingBox::FAR_LEFT_BOTTOM ), expansionBounding.getCorner( math::BoundingBox::FAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( expansionBounding.getCorner( math::BoundingBox::FAR_RIGHT_BOTTOM ), expansionBounding.getCorner( math::BoundingBox::NEAR_RIGHT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( expansionBounding.getCorner( math::BoundingBox::NEAR_RIGHT_BOTTOM ), expansionBounding.getCorner( math::BoundingBox::NEAR_LEFT_BOTTOM ) ) );
    bottomSegments.push_back( math::LineSegment( expansionBounding.getCorner( math::BoundingBox::NEAR_LEFT_BOTTOM ), expansionBounding.getCorner( math::BoundingBox::FAR_LEFT_BOTTOM ) ) );

    calculateObjectPoints(gridPoints[idHash], bottomSegments);

    // the total area that the object occupies with expansion boundary.
    const std::vector<spatial::GridPoint>& totalArea = this->gridPoints[idHash];
    // Calculate the expansion area.
    std::set_difference(solidArea.begin(), solidArea.end(), totalArea.begin(), totalArea.end(), expansionArea.begin());
    this->entities.insert(LongEntityPtrHashMap::value_type(idHash, entity));

    const char* internalId = entity->getName().c_str();
    logger().debug("LocalSpaceMap - Adding internal points to grid..." );

    unsigned int i;
    for (i = 0; i < solidArea.size(); ++i) {
        ObjectInfo info(internalId, false);
        _grid[solidArea[i]].insert(info);
    } // for
    for (i = 0; i < expansionArea.size(); ++i) {
        ObjectInfo info(internalId, true);
        _grid[expansionArea[i]].insert(info);
    } // for
}

void LocalSpaceMap2D::updateObject( const spatial::ObjectID& id, const spatial::ObjectMetaData& metadata, bool isObstacle )
{

    try {
        const EntityPtr& entity = getEntity( id );
        spatial::ObjectMetaData metaData( entity->getPosition( ).x,
                                          entity->getPosition( ).y,
                                          entity->getPosition( ).z,
                                          entity->getLength( ),
                                          entity->getWidth( ),
                                          entity->getHeight( ),
                                          entity->getOrientation( ).getRoll( ),
                                          entity->getStringProperty( Entity::ENTITY_CLASS ));
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


bool LocalSpaceMap2D::belongsToSuperEntity( const spatial::ObjectID& id ) const
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

const SuperEntityPtr& LocalSpaceMap2D::getSuperEntityWhichContains( const spatial::ObjectID& id ) const throw(opencog::NotFoundException)
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
rec_find::rec_find(const spatial::GridPoint& current,
                   const spatial::GridMap& grid,
                   spatial::ObjectIDSet& out,
                   const spatial::LocalSpaceMap2D& parent) :
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


    spatial::GridMap::const_iterator it = _grid.find(_current);

    if (it != _grid.end()) {
        //std::copy(it->second.begin(), it->second.end(), std::inserter(_out, _out.begin()));
        spatial::ObjectInfoSet::const_iterator obj_info_it = (it->second).begin();
        while (obj_info_it != (it->second).end()) {
            _out.insert(obj_info_it->id);
            ++obj_info_it;
        }
    }
}

void rec_find::johnnie_walker(spatial::Distance d)
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

std::string LocalSpaceMap2D::toString( const LocalSpaceMap2D& map )
{
    std::stringstream out;
    out.precision(25);

    out << map.xMin( ) << " " << map.xMax( ) << " " << map.xDim( ) << " ";
    out << map.yMin( ) << " " << map.yMax( ) << " " << map.yDim( ) << " ";
    out << map.floorHeight() << " ";
    out << map.radius( ) << " " << map.agentHeight() << " ";

    out << map.entities.size() << " ";

    LongEntityPtrHashMap::const_iterator it;
    for ( it = map.entities.begin( ); it != map.entities.end( ); ++it ) {        
        out << it->second->getName( ) << " ";

        out << it->second->getPosition( ).x << " " 
            << it->second->getPosition( ).y << " " 
            << it->second->getPosition( ).z << " ";

        out << it->second->getLength( ) << " " 
            << it->second->getWidth( ) << " "
            << it->second->getHeight( ) << " ";

        out << it->second->getOrientation( ).x << " "
            << it->second->getOrientation( ).y << " " 
            << it->second->getOrientation( ).z << " "
            << it->second->getOrientation( ).w << " ";
        
        out << it->second->getStringProperty( Entity::ENTITY_CLASS ) << " ";
        out << it->second->getBooleanProperty( Entity::OBSTACLE ) << " ";
    } // for
    return out.str( );
}

LocalSpaceMap2D* LocalSpaceMap2D::fromString( const std::string& map )
{
    std::stringstream parser(map);
    parser.precision(25);

    double xMin = 0, xMax = 0;
    double yMin = 0, yMax = 0;
    double floorHeight = 0;
    double agentRadius = 0;
    double agentHeight = 0;
    unsigned int xDim = 0, yDim = 0, numberOfObjects = 0;

    parser >> xMin >> xMax >> xDim;
    parser >> yMin >> yMax >> yDim;
    parser >> floorHeight;
    parser >> agentRadius;
    parser >> agentHeight;
   
    LocalSpaceMap2D* newMap = new LocalSpaceMap2D( xMin, xMax, xDim, yMin, yMax, yDim, floorHeight, agentRadius, agentHeight );

    parser >> numberOfObjects;
    unsigned int i;
    for( i = 0; i < numberOfObjects; ++i ) {
        std::string name;
        math::Vector3 position( 0, 0, 0);
        math::Dimension3 dimension;
        double orientationX = 0, orientationY = 0, orientationZ = 0, orientationW = 0;
        std::string entityClass;
        bool obstacle = false;
        parser >> name 
               >> position.x >> position.y >> position.z
               >> dimension.length >> dimension.width >> dimension.height
               >> orientationX >> orientationY >> orientationZ >> orientationW
               >> entityClass
               >> obstacle;
        math::Quaternion orientation( orientationX, orientationY, orientationZ, orientationW );
        spatial::ObjectMetaData metaData( position.x, 
                                          position.y, 
                                          position.z,
                                          dimension.length,
                                          dimension.width,
                                          dimension.height,
                                          orientation.getRoll( ),
                                          entityClass);
        newMap->addObject( name, metaData, obstacle );
    } // for  

    return newMap;    
}
