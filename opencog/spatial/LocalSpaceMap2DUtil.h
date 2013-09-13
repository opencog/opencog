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

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/tuple/tuple.hpp>

#include <opencog/spatial/math/Triangle.h>
#include <opencog/spatial/math/LineSegment.h>
#include <opencog/spatial/math/Vector3.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        class LocalSpaceMap2D;

        typedef double Distance;
        typedef double Altitude;
        typedef std::string ObjectID;

        typedef std::pair<Distance, Distance> Point;
        typedef boost::tuple<spatial::Distance, spatial::Distance, spatial::Distance> Point3D;
        typedef std::pair<unsigned int, unsigned int> GridPoint;
        typedef std::pair<Altitude, Altitude> Gradient;

        struct gradient_cmp_by_lower_altitude {
            bool operator() (const Gradient& a, const Gradient& b) const {
                return a.first < b.first;
            }
        };

        typedef boost::unordered_set<GridPoint, boost::hash<GridPoint> > GridSet;

        // An object in the local space map might be seperated into several
        // grid points. Since each object has an extra boundary(with the 
        // length of agent's radius) to detect collision with agent, there 
        // might be chances that the boundary of an object is
        // overlapping with another in grid representation when these two
        // objects are close to each other. We need a flag to tell whether a
        // seperated part of an object in a grid is an extra boundary or not.
        struct ObjectInfo {
            const char* id;
            bool isExtraBoundary;

            ObjectInfo(const char* id_, bool isExtraBoundary_) {
                id = id_;
                isExtraBoundary = isExtraBoundary_;
            }

            ObjectInfo(const std::string& id_, bool isExtraBoundary_) {
                id = id_.c_str();
                isExtraBoundary = isExtraBoundary_;
            }

            bool operator<(const ObjectInfo& rh) const {
                return strcmp(id, rh.id) < 0;
            }
        };

        struct c_str_compare {
            bool operator()(const char* s1, const char* s2) const {
                return strcmp(s1, s2) < 0;
            }
        };

        typedef std::set<const char*, c_str_compare> ObjectIDSet;
        typedef std::set<ObjectInfo> ObjectInfoSet;
        typedef boost::unordered_map<GridPoint, ObjectInfoSet, boost::hash<GridPoint> > GridMap;

        //typedef std::map<const char*, bool, c_str_compare> ObjectInfoSet;
        //typedef boost::unordered_map<GridPoint, ObjectIDSet, boost::hash<GridPoint> > GridMap;
        typedef boost::unordered_map<long, std::vector<GridPoint>, boost::hash<long> > LongGridPointVectorHashMap;

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
            std::string entityClass;

            ObjectMetaData();
            ObjectMetaData(double cx, double cy, double cz, double l, double w, double h, double y);
            ObjectMetaData(double cx, double cy, double cz, double l, double w, double h, double y, const std::string& ec);

            bool operator==(const ObjectMetaData& rhs) const;
            bool operator!=(const ObjectMetaData& rhs) const;

        };


        /**
         * Functor class that collects every grid point on a rayTrace execution
         */
        class GridPointCollector
        {
        public:
            GridPointCollector( std::vector<spatial::GridPoint>& gridPoints );

            bool operator()( const spatial::GridPoint& gridPoint );

        private:
            std::vector<spatial::GridPoint>& gridPoints;
        };

        /**
         * Functor class that can be used on rayTrace as Predicate.
         * It stops the execution of the rayTrace method when collided and keep the last
         * free grid point (if not starts at an invalid point)
         */
        class CollisionDetector
        {
        public:
            CollisionDetector( LocalSpaceMap2D* map, spatial::GridPoint& collisionPoint, bool& collided );
            bool operator()( const spatial::GridPoint& gridPoint );

        private:
            LocalSpaceMap2D* map;
            spatial::GridPoint& collisionPoint;
            bool& collided;
        };

        /**
         * Template method that do a ray tracing in a grid space
         *
         * @param startPoint A grid cell point used to start ray tracing
         * @param endPoint A grid cell point used to stop the ray tracing
         * @param predicate A functor object used to handle every point during ray tracing
         */
        template <class Predicate> void rayTrace( const spatial::GridPoint& startPoint, const spatial::GridPoint& endPoint, Predicate predicate )
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

                spatial::GridPoint currentPosition;

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
        void populateRandom(spatial::LocalSpaceMap2D& lsm,
                            int obstacles, const spatial::GridPoint& prob_center, int std_dev = 75);

    } // spatial
/** @}*/
} // opencog

#endif
