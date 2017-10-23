/*
 * opencog/spatial/math/BoundingBox.h
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

#ifndef _SPATIAL_MATH_BOUNDINGBOX_H_
#define _SPATIAL_MATH_BOUNDINGBOX_H_

#include <vector>
#include <list>
#include <map>
#include <boost/functional/hash.hpp>

#include <opencog/spatial/math/Vector3.h>
#include <opencog/spatial/math/LineSegment.h>
#include <opencog/spatial/math/SquareFace.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {

        class Entity;

        namespace math {

            class BoundingBox
            {
            public:

                /**
                 * Corners
                 *
                 *      4+------+5
                 *      /|     /|
                 *     / |    / |
                 *   7+------+6 | --front-->
                 *    | 0+---|--+1
                 *    | /    | /  
                 *    |/     |/
                 *   3+------+2
                 */
                enum CORNER {
                    NEAR_LEFT_BOTTOM,  
                    FAR_LEFT_BOTTOM,
                    FAR_RIGHT_BOTTOM,  
                    NEAR_RIGHT_BOTTOM, 

                    NEAR_LEFT_TOP,     
                    FAR_LEFT_TOP,      
                    FAR_RIGHT_TOP,
                    NEAR_RIGHT_TOP     
                };

                /**
                 * Constructor
                 * @param entity the entity involved by the bounding box
                 */
                BoundingBox( Entity* entity );


                /**
                 * Get the spatial points corresponding to the corners of the bounding box
                 * @return a vector containing all the bounding box corners
                 */
                const std::vector<Vector3>& getAllCorners( void ) const;

                /**
                 * Get the position of one of the corners
                 * @param cornerToGet Specific corner
                 * @return the corner position
                 */
                const Vector3& getCorner(CORNER cornerToGet) const;

                /**
                 * Get all the bounding box edges
                 * @return the edges
                 */
                const std::vector<LineSegment>& getAllEdges( void ) const;

                /**
                 * Check if a given point is inside the bounding box
                 * @param point
                 * @return true if the point is inside or false otherwise
                 */
                bool isInside( const Vector3& point ) const;

                /**
                 * Return the square faces of bounding box
                 * @return all the faces
                 */
                const std::vector<SquareFace>& getAllFaces( void );

                /**
                 * Update the bounding box rebuilding the corners positions
                 */
                void update( void );

                /**
                 * Return the distance between a given point and this bb
                 */
                double distanceTo( const Vector3& point ) const;

                bool operator==( const BoundingBox& bb ) const;

                BoundingBox& operator=( const BoundingBox& bb );

                const std::list<LineSegment*>& getEdges( const Vector3& corner ) const;

            private:
                void buildCorners( void );
                void buildFaces( void );
                void buildEdges( void );


                struct Vector3Hash {
                    inline std::size_t operator()(const Vector3& o) const {
                        std::size_t seed = 0;
                        boost::hash_combine( seed, o.x );
                        boost::hash_combine( seed, o.y );
                        boost::hash_combine( seed, o.z );
                        return seed;
                    }
                };

                Entity* entity;
                std::vector<SquareFace> squareFaces;
                std::vector<LineSegment> edges;
                std::map<Vector3, std::list<LineSegment*> > cornerEdges;
                std::vector<Vector3> corners;

            }; // BoundingBox

        } // math
    } // spatial
/** @}*/
} // opencog

#endif // _SPATIAL_MATH_BOUNDINGBOX_H_
