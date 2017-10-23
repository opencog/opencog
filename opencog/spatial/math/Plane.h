/*
 * opencog/spatial/math/Plane.h
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

#ifndef _SPATIAL_MATH_PLANE_H_
#define _SPATIAL_MATH_PLANE_H_

#include <string>

#include <opencog/util/exceptions.h>

#include <opencog/spatial/math/Vector3.h>
#include <opencog/spatial/math/Matrix4.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        namespace math {

            class Plane
            {
            public:
                /**
                 * Classifies the sides of the plane
                 */
                enum SIDE {
                    NEGATIVE,
                    POSITIVE,
                    INTERSECT
                };

                /**
                 * Simple constructor
                 */
                Plane( void );

                /**
                 * Copy constructor
                 * @param plane
                 */
                Plane( const Plane& plane );

                /**
                 * Use a normal and a distance from origin to build a plane
                 * @param normal
                 * @param distance
                 */
                Plane( const Vector3& normal, double distance );

                /**
                 * Build a plane using three arbitrary points
                 * @param pointA
                 * @param pointB
                 * @param pointC
                 */
                Plane( const Vector3& pointA, const Vector3& pointB, const Vector3& pointC );

                /**
                 * Set new normal and distance values to this plane
                 * @param normal
                 * @param distance
                 */
                void set( const Vector3& normal, double distance );

                /**
                 * Get a 4 dimension vector that represents this plane
                 * @return
                 */
                Vector4 getVector4( void );

                /**
                 * Get the distance between this plane and a given point
                 * @param point
                 * @return
                 */
                double getDistance( const Vector3& point );

                /**
                 * Identifies the side of the plane a given point is positioned
                 * @param point
                 * @return
                 */
                SIDE getSide( const Vector3& point );

                /**
                 * Apply a transformation matrix to this plane
                 * @param transformation
                 */
                void transformSelf( const Matrix4& transformation );

                /**
                 * Get the intersection point between this plane and other two
                 * @param plane2
                 * @param plane3
                 * @return
                 */
                Vector3 getIntersectionPoint( const Plane& plane2, const Plane& plane3 );

                /*
                 *
                 */
                bool operator==( const Plane& other ) const;

                /*
                 *
                 */
                std::string toString(void) const;

                inline virtual ~Plane( void ) { }

                Vector3 normal;
                double distanceFromOrigo;
            }; // Plane

        } // math
    } // spatial
/** @}*/
} // opencog

#endif // _SPATIAL_MATH_PLANE_H_
