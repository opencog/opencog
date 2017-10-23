/*
 * opencog/spatial/LocalSpaceMap2D.h
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

#ifndef _SPATIAL_LOCAL_SPACE_MAP_2D_H_
#define _SPATIAL_LOCAL_SPACE_MAP_2D_H_

#include <cmath>

#include <opencog/util/Logger.h>
#include <opencog/util/numeric.h>
#include <opencog/util/functional.h>
#include <opencog/util/mt19937ar.h>

#include <opencog/spatial/LocalSpaceMap2DUtil.h>

#include <iostream>
#include <exception>
#include <boost/bind.hpp>
#include <string>

#include <opencog/spatial/SuperEntity.h>

/**
 * Represents a set of 2D object occuring within a given rectangle of space
 *
 * answers queries related to points, regions and trajectories within this
 * rectangle, based on a discretization (i.e. finite grid) of the region
 *
 * behavior is undefined for queries involving points outside this region
 */
namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {

        class LocalSpaceMap2D;

        /**
         * Struct rec_find
         */
        struct rec_find {

        private:

            GridPoint _current;
            const GridPoint& _g;

            const GridMap& _grid;

            ObjectIDSet& _out;

            const LocalSpaceMap2D* _parent;

            bool too_far() const;

            void move_left();
            void move_right();
            void move_up();
            void move_down();

            void rec();
            void check_grid();

            unsigned int step;
            unsigned int possible_step;
            unsigned int last_x_difference;
            unsigned int last_y_difference;

        public:
            rec_find(const GridPoint& current,
                     const GridMap& grid,
                     ObjectIDSet& out,
                     const LocalSpaceMap2D& parent);

            void johnnie_walker(Distance d);

        }; // struct rec_find

        class LocalSpaceMap2D
        {

        protected:

            GridMap _grid;
            GridMap _grid_nonObstacle;

            //    ObjectHashMap objects;

            std::list<SuperEntityPtr> superEntities;
            LongEntityPtrHashMap entities;
            LongGridPointVectorHashMap gridPoints;

            bool addToSuperEntity( const EntityPtr& entity );

        private:
            Distance _xMin;
            Distance _xMax;
            unsigned int _xDim;

            Distance _yMin;
            Distance _yMax;
            unsigned int _yDim;

            Distance _floorHeight;

            Distance _diagonalSize;

            //this is an argument to the constructor and stored so we can
            //cache things for performance
            Distance _radius;
            Distance _agentHeight;

            // used only for test
            const GridSet _empty_set;

            bool outsideMap( const std::vector<math::LineSegment>& segments );

            /**
             * Overrides and declares copy constructor and equals operator as private 
             * for avoiding large object copying by mistake.
             */
            LocalSpaceMap2D& operator=(const LocalSpaceMap2D&);
            LocalSpaceMap2D(const LocalSpaceMap2D&);

        public:

            // constants used to compute the near and next threshold
            static const double NEXT_FACTOR;
            static const double NEAR_FACTOR;



            /**
             * Constructor
             * Creates an empty map with the given dimensions of the rectangle this map
             * represents.
             * xMin: the lower x coordinate of the rectangle
             * xMax: the upper x coordinate of the rectangle
             * xDim: the dimension (in number of grid cells it is divided) of the x side
             *       of the rectangle.
             * yMin: the lower y coordinate of the rectangle
             * yMax: the upper y coordinate of the rectangle
             * yDim: the dimension (in number of grid cells it is divided) of the y side
             *       of the rectangle.
             * radius: the radius of the agent which uses this map (for navigation
             * purposes).
             * agentHeight: the height of the agent which uses this map, zero by
             * default for 2D map.
             * floor: the height of the terrain, zero by default for 2D map.
             */
            LocalSpaceMap2D(Distance xMin, Distance xMax, unsigned int xDim,
                            Distance yMin, Distance yMax, unsigned int yDim,
                            Distance radius, Distance agentHeight = 0.0,
                            Distance floor = 0.0);

            LocalSpaceMap2D* clone() const;
            ~LocalSpaceMap2D();

            /**
             * Operator overloading
             */
            bool operator==(const LocalSpaceMap2D& other) const;

            /**
             * Persistence
             */
            void save(FILE* fp ) const;
            void load(FILE* fp );

            Distance xGridWidth() const;
            Distance yGridWidth() const;

            Distance xMin() const;
            Distance xMax() const;
            unsigned int xDim() const;

            Distance yMin() const;
            Distance yMax() const;
            unsigned int yDim() const;

            Distance diagonalSize() const; // just return a pre-calculated diagonal size

            Distance radius() const;

            Distance agentHeight() const;
            
            Distance floorHeight() const;

            // Is the given point occupied by any obstacle or out of bounds?
            bool illegal(const Point& pt) const;

            // Get the nearest free (not occupied or out of bounds)
            // point from the given point
            Point getNearestFreePoint(const Point& pt) const;
            Point3D getNearestFree3DPoint(const Point3D& pt, double delta) const;

            // Get the proper altitude of a given grid point in order 
            // to reach a target 3D point with a limited delta upwards height.
            double getProperFreePointAltitude(const GridPoint& gp, const Point3D& dest, double delta) const;

            // Is the given point occupied by any obstacle or out of bounds?
            bool gridIllegal(const GridPoint& gp) const;
            bool gridIllegal(unsigned int i, unsigned int j) const;

            bool gridOccupied(const GridPoint& gp) const;
            bool gridOccupied(unsigned int i, unsigned int j) const;

            bool gridOccupied_nonObstacle(const GridPoint& gp) const;
            bool gridOccupied_nonObstacle(unsigned int i, unsigned int j) const;

            // Handle 3D information in the map

            // If point "dest" is an obstacle, we can still use it in pathfinding only when its delta
            // height with point "src" is within a specific value "delta".
            bool edgeIllegal(const GridPoint& src, const GridPoint& dest, double srcHeight, double delta) const;
            double getProperDestAltitude(const GridPoint& src, const GridPoint& dest, const double srcHeight, double delta) const;
            bool isDiagonal(const GridPoint& src, const GridPoint& dest) const;
            ObjectID getTallestObjectInGrid(const GridPoint& gp) const; 

            std::vector<Gradient> getObjectGradientsByGridPoint(const GridPoint& gp) const;
            //const ObjectMetaData& getMetaData(const ObjectID& id) const throw(opencog::NotFoundException);

            bool containsObject(const ObjectID& id) const;
            bool isObstacle(const ObjectID& id) const;
            bool isNonObstacle(const ObjectID& id) const;

            // Gets the grid points occupied by the object with the given id
            const std::vector<GridPoint>& getObjectPoints(const ObjectID& id) const;

            /*
             * Threshold to consider an entity near another
             * @return Near distance
             */
            inline double getNearDistance( void ) const {
                return ( xMax( ) - xMin( ) ) * NEAR_FACTOR;
            }

            /*
             * Threshold to consider an entity next to another
             * @return Next distance
             */
            inline double getNextDistance( void ) const {
                return ( xMax( ) - xMin( ) ) * NEXT_FACTOR;
            }


            // Note: At time of writing, this function is only used for the unit tests.
            // List all objects IDs (obstacles and non-obstacles) on this map
            template<typename Out>
                Out getAllObjects(Out out) const {

                ObjectIDSet localObjects;

                //ObjectHashMap::const_iterator it;
                LongEntityPtrHashMap::const_iterator it;
                //for( it = this->objects.begin( ); it != this->objects.end( ); ++it ) {
                for ( it = this->entities.begin( ); it != this->entities.end( ); ++it ) {
                    //localObjects.insert( it->first.c_str( ) );
                    localObjects.insert( it->second->getName( ).c_str( ) );
                } // for

                return std::copy(localObjects.begin(), localObjects.end(), out);
            }

            // Copy all objects of the given map into this one
            void copyObjects(const LocalSpaceMap2D& otherMap);

            //remove an object from the map entirely
            void removeObject(const ObjectID& id);

            //find the IDs of all objects within distance d of a certain point
            //same code use in findNearestFiltered
            template<typename Out>
                Out findEntities(const GridPoint& g, Distance d, Out out) const {
                ObjectIDSet objs;

                struct rec_find finder(g, _grid, objs, *this);
                struct rec_find finderNonObstacle(g, _grid_nonObstacle, objs, *this);

                finder.johnnie_walker(d);
                finderNonObstacle.johnnie_walker(d);

                return std::copy(objs.begin(), objs.end(), out);

            }
            //find the minimal distance between some point in an object and a reference
            //point
            Distance minDist(const ObjectID& id, const Point& p) const;

            //find the nearest entity to a given point satisfying some predicate
            template<typename Pred>
                ObjectID findNearestFiltered(const Point& p, Pred pred) const {

                std::vector<ObjectID> tmp;
                Distance d = 5;

                //convert to greidPoint
                GridPoint g = snap(p);

                Distance x = fmax(xDim() - g.first - 1, g.first);
                Distance y = fmax(yDim() - g.second - 1, g.second);
                Distance maxD = sqrt((x * x) + (y * y));

                ObjectIDSet objs;

                struct rec_find finder(g, _grid, objs, *this);
                struct rec_find finderNonObstacle(g, _grid_nonObstacle, objs, *this);

                do {
                    finder.johnnie_walker(d);
                    finderNonObstacle.johnnie_walker(d);

                    std::copy(objs.begin(), objs.end(), back_inserter(tmp));

                    tmp.erase(std::partition(tmp.begin(), tmp.end(), pred), tmp.end());

                    if (d >= maxD && tmp.empty()) {
                        opencog::logger().debug("LocalSpaceMap - Found no object that verifies the predicate. Distance %.2f, Max distance %.2f.", d, maxD);
                        //won't find anything
                        return ObjectID();
                    }
                    d *= 1.5; //maybe it shouldn't grow so fast?
                    if (d > maxD ) {
                        d = maxD;
                    }

                } while (tmp.empty());

                //now find the closest
                std::vector<Distance> dists(tmp.size());
                std::transform(tmp.begin(), tmp.end(), dists.begin(),
                               boost::bind(&LocalSpaceMap2D::minDist, this,  ::_1, boost::cref(p)));

                return *(tmp.begin() + distance(dists.begin(),
                                                min_element(dists.begin(), dists.end())));
            }

            //find a random entity satisfying some predicate
            template<typename Pred>
                ObjectID findRandomFiltered(Pred pred) const {

                //filter out all entities that match
                std::vector<ObjectID> tmp;

                //ObjectHashMap::const_iterator it;
                LongEntityPtrHashMap::const_iterator it;
                //for( it = this->objects.begin( ); it != this->objects.end( ); ++it ) {
                for ( it = this->entities.begin( ); it != this->entities.end( ); ++it ) {
                    //if ( pred( it->first ) ){
                    if ( pred( it->second->getName( ) ) ) {
                        //tmp.push_back( it->first );
                        tmp.push_back( it->second->getName( ) );
                    } // if
                } // for

                return tmp.empty() ? ObjectID() : tmp[randGen().randint(tmp.size())];
            }

            // return id's of all objects within the space map
            template<typename Out>
                Out findAllEntities(Out out) const {

                // If one object id is stored as an obstacle and a non-obstacle,
                // only count it once:
                ObjectIDSet localObjects;

                //ObjectHashMap::const_iterator it;
                LongEntityPtrHashMap::const_iterator it;
                //for( it = this->objects.begin( ); it != this->objects.end( ); ++it ) {
                for ( it = this->entities.begin( ); it != this->entities.end( ); ++it ) {
                    //localObjects.insert( it->first.c_str( ) );
                    localObjects.insert( it->second->getName( ).c_str( ) );
                } // for

                return std::copy(localObjects.begin(), localObjects.end(), out);
            }

            // return id's of all objects within the space map
            template<typename Out>
                Out findEntitiesWithClassFilter(Out out, const std::string& filterStr) const {

                // If one object id is stored as an obstacle and a non-obstacle,
                // only count it once:
                ObjectIDSet localObjects;

                LongEntityPtrHashMap::const_iterator it;
                for ( it = this->entities.begin( ); it != this->entities.end( ); ++it ) {
                    const std::string& entity_class = it->second->getStringProperty(Entity::ENTITY_CLASS);
                    if (entity_class != filterStr)
                        localObjects.insert( it->second->getName( ).c_str( ) );
                } // for

                return std::copy(localObjects.begin(), localObjects.end(), out);
            }

            //all of the points associated with a given object
            template<typename Out>
                Out allPoints(const ObjectID& id, Out out) const {
                std::vector<Point> points;

                //ObjectHashMap::const_iterator it = this->objects.find( id );
                long idHash = boost::hash<std::string>()( id );
                LongGridPointVectorHashMap::const_iterator it = this->gridPoints.find( idHash );
                //if ( it != this->objects.end( ) ) {
                if ( it != this->gridPoints.end( ) ) {
                    unsigned int i;
                    //for( i = 0; i < it->second.gridPoints.size( ); ++i ) {
                    for ( i = 0; i < it->second.size( ); ++i ) {
                        //points.push_back( unsnap( it->second.gridPoints[ i ] ) );
                        points.push_back( unsnap( it->second[ i ] ) );
                    } // for
                } // if

                return std::copy(points.begin(), points.end(), out);
            }

            //Euclidean distance between points
            template<typename PointT>
                static Distance eucDist(const PointT& p1, const PointT& p2) {
                return std::sqrt(opencog::sq(p1.first - p2.first) +
                                 opencog::sq(p1.second - p2.second));
            }

            //are the given /grid coordinates/ within the dimensions of this grid?
            bool coordinatesAreOnGrid(unsigned int x, unsigned int y) const;

            //should return a nearby (to the src) unoccupied point (taking _radius into
            //account) on the edge of the object with the given id currently just
            //returns the center
            Point nearbyPoint(const Point& src, const ObjectID& id) const;

            //should return an unoccupied point p (taking _radius into account) on or
            //near the edge of the object 'id' such that the object falls between src
            //and p
            //
            //formally, behindPoint returns a point x such that p, centerOf(id), and x
            //are approximately collinear, centerOf(id) is between p and x, x is
            //unoccupied (taking _radius into account), and the euclidean distance
            //between centerOf(id) and x is minimized
            Point behindPoint(const Point& src, const ObjectID& id) const;

            //returns the first free point traveling from src in direction if no free
            //point is found, will return the last valid point encountered
            Point findFree(const Point& p, const Point& direction) const;

            //the center of a object - currently this is computed as the center of the
            //bounding rectangle, but e.g. and average would be fine to - not sure
            //which will give nicer results in SL or if it will matter
            Point centerOf(const ObjectID& id) const;

            //nearest point on grid
            GridPoint snap(const Point& p) const;

            //back to the original coord system
            Point unsnap(const GridPoint& g) const;

            /**
             * Find a free point near a given position, at a given distance
             * @param position Given position
             * @param distance Maximum distance from the given position to search the free point
             * @param startDirection Vector that points to the direction of the first rayTrace
             */
            Point getNearFreePointAtDistance( const Point& position, float distance, const Point& startDirection ) const;

            /**
             * Get the nearest object point of a given a reference point
             *
             * @param referencePoint Point used as a reference to find the nearest object point
             * @param objectID Object ID
             * @return The nearest object point
             */
            Point getNearestObjectPoint( const Point& referencePoint, const ObjectID& objectID ) const;

            /**
             * Find all points belonging to a given line segment
             *
             * @param points Calculated vector grid points (out)
             * @param segment Segment used to calculate the points
             */
            void calculateSegmentGridPoints( std::vector<GridPoint>& points, const math::LineSegment& segment );

            /**
             * Find all points belonging to an object, represented by it's segments
             *
             * @param points Calculated vector grid points (out)
             * @param segments Vector of segments that represtends an object
             */
            void calculateObjectPoints( std::vector<GridPoint>& points, const std::vector<math::LineSegment>& segments );

            /**
             * Add an object to the map
             *
             * @param id Object id
             * @param metadata Object metadata
             * @param isObstacle If true the object will be considered an obstacle, false an nonObstacle
             */
            void addObject( const ObjectID& id, const ObjectMetaData& metadata, bool isObstacle = false );

            /**
             * Add basic terrain element(block) to the map
             * This method is only served in the minecraft-like world.
             */
            void addBlock( const ObjectID& id, const ObjectMetaData& metadata);

            /**
             * Update the points of an object. If the object metadata has not changed,
             * nothing will be done. If it was changed from object<->nonObject it will be updated too.
             *
             * @param id Object id
             * @param metadata Object metadata
             * @param isObstacle If true the object will be considered an obstacle, false an nonObstacle
             */
            void updateObject( const ObjectID& id, const ObjectMetaData& metadata, bool isObstacle = false );

            //const Object& getObject( const ObjectID& id ) const throw (opencog::NotFoundException);

            const EntityPtr& getEntity( const std::string& id ) const;
            const EntityPtr& getEntity( long id ) const;

            bool belongsToSuperEntity( const ObjectID& id ) const;

            const SuperEntityPtr& getSuperEntityWhichContains( const ObjectID& id ) const;

            inline const std::list<SuperEntityPtr>& getSuperEntities( void ) const {
                return this->superEntities;
            }

            static std::string toString( const LocalSpaceMap2D& map );
            static LocalSpaceMap2D* fromString( const std::string& map );

        }; // struct LocalSpaceMap2D

    } // spatial
/** @}*/
} // opencog

#endif
