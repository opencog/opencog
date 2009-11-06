/*
 * opencog/spatial/LocalSpaceMap2DUtil.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Dan Zwell, Samir Araujo
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

#ifndef _SPATIAL__LOCAL_SPACE_MAP2D_UTIL_H_
#define _SPATIAL__LOCAL_SPACE_MAP2D_UTIL_H_

#include <opencog/util/RandGen.h>
#include <opencog/util/foreach.h>
#include <opencog/util/hash_set.h>
#include <opencog/util/hash_map.h>

#include <map>
#include <iostream>
#include <string>

#include <boost/functional/hash.hpp>

#include "Math/Triangle.h"
#include "Math/LineSegment.h"
#include "Math/Vector3.h"

namespace Spatial
{
class LocalSpaceMap2D;

typedef double Distance;
typedef std::string ObjectID;

typedef std::pair<Distance, Distance> Point;
typedef std::pair<unsigned int, unsigned int> GridPoint;

typedef opencog::hash_set<GridPoint, boost::hash<GridPoint> > GridSet;

struct c_str_compare {
    bool operator()(const char* s1, const char* s2) const {
        return strcmp(s1, s2) < 0;
    }
};
typedef std::set<const char*, c_str_compare> ObjectIDSet;
typedef opencog::hash_map<GridPoint, ObjectIDSet, boost::hash<GridPoint> > GridMap;
typedef opencog::hash_map<long, std::vector<GridPoint>, boost::hash<long> > LongGridPointVectorHashMap;

/**
 * Represents the object geometry
 */
class ObjectMetaData
{
public:
    double centerX;
    double centerY;
    double centerZ;
    double length;
    double width;
    double height;
    double yaw;

    ObjectMetaData();
    ObjectMetaData(double cx, double cy, double cz, double l, double w, double h, double y);

    bool operator==(const ObjectMetaData& rhs) const;
    bool operator!=(const ObjectMetaData& rhs) const;

};


/**
 * Functor class that collects every grid point on a rayTrace execution
 */
class GridPointCollector
{
public:
    GridPointCollector( std::vector<Spatial::GridPoint>& gridPoints );

    bool operator()( const Spatial::GridPoint& gridPoint );

private:
    std::vector<Spatial::GridPoint>& gridPoints;
};

/**
 * Functor class that can be used on rayTrace as Predicate.
 * It stops the execution of the rayTrace method when collided and keep the last
 * free grid point (if not starts at an invalid point)
 */
class CollisionDetector
{
public:
    CollisionDetector( LocalSpaceMap2D* map, Spatial::GridPoint& collisionPoint, bool& collided );
    bool operator()( const Spatial::GridPoint& gridPoint );

private:
    LocalSpaceMap2D* map;
    Spatial::GridPoint& collisionPoint;
    bool& collided;
};

/**
 * Template method that do a ray tracing in a grid space
 *
 * @param startPoint A grid cell point used to start ray tracing
 * @param endPoint A grid cell point used to stop the ray tracing
 * @param predicate A functor object used to handle every point during ray tracing
 */
template <class Predicate> void rayTrace( const Spatial::GridPoint& startPoint, const Spatial::GridPoint& endPoint, Predicate predicate )
{

    // Bresenham line algorithm
    int x1 = startPoint.first;
    int y1 = startPoint.second;
    int x2 = endPoint.first;
    int y2 = endPoint.second;

    int temp;

    bool steep = ( abs( y2 - y1) > abs(x2 - x1) );
    if ( steep ) {
        //   swap(x0, y0)
        temp = x1; x1 = y1; y1 = temp;
        //   swap(x1, y1)
        temp = x2; x2 = y2; y2 = temp;
    } // if
    if ( x1 > x2 ) {
        //   swap(x0, x1)
        temp = x1; x1 = x2; x2 = temp;
        //   swap(y0, y1)
        temp = y1; y1 = y2; y2 = temp;
    } // if

    int deltax = x2 - x1;
    int deltay = abs(y2 - y1);

    float error = -deltax / 2;

    int ystep = ( y1 < y2 ) ? 1 : -1;

    int y = y1;
    int x;

    Spatial::GridPoint currentPosition;

    for ( x = x1; x <= x2; x += 1 ) {
        if ( steep ) {
            currentPosition.first = y;
            currentPosition.second = x;
        } else {
            currentPosition.first = x;
            currentPosition.second = y;
        } // else

        if ( !predicate( currentPosition ) ) {
            return;
        } // if

        error = error + deltay;
        if ( error >= 0.5 ) {
            y = y + ystep;
            error = error - deltax;
        } // if
    } // for

}

/**
 * DEPRECATED METHOD - it must be removed as like TangentBugTestExec and AStarTest
 */
void populateRandom(opencog::RandGen& rng, Spatial::LocalSpaceMap2D& lsm,
                    int obstacles, const Spatial::GridPoint& prob_center, int std_dev = 75);

} // namespace spatial

#endif
