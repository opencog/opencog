/*
 * opencog/spatial/math/Triangle.h
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

#ifndef _SPATIAL_MATH_TRIANGLE_H_
#define _SPATIAL_MATH_TRIANGLE_H_

#include <opencog/spatial/math/Vector3.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        namespace math {
            /**
             * This class represents a triangle. A triangle has three points in
             * Cartesian plane.
             */
            class Triangle
            {
            public:

                inline Triangle( void ) 
                { 
                };

                inline Triangle( const Vector3& pointA, const Vector3& pointB, const Vector3& pointC ) :
                    pointA( pointA ), pointB( pointB ), pointC( pointC ) 
                { 
                }

                inline virtual ~Triangle( void ) 
                { 
                }

                inline bool operator==( const Triangle& other ) 
                {
                    return ( pointA == other.pointA && pointB == other.pointB && pointC == other.pointC );
                }

                inline Triangle& operator=( const Triangle& other ) 
                {
                    this->pointA = other.pointA;
                    this->pointB = other.pointB;
                    this->pointC = other.pointC;
                    return *this;
                }

                inline bool isInside( const Vector3& point ) const 
                {
                    // algorithm reference: http://www.blackpawn.com/texts/pointinpoly/default.html

                    // Compute vectors
                    Vector3 v0 = this->pointC - this->pointA;
                    Vector3 v1 = this->pointB - this->pointA;
                    Vector3 v2 = point - this->pointA;

                    // Compute dot products
                    float dot00 = v0.dotProduct(v0);
                    float dot01 = v0.dotProduct(v1);
                    float dot02 = v0.dotProduct(v2);
                    float dot11 = v1.dotProduct(v1);
                    float dot12 = v1.dotProduct(v2);

                    // Compute barycentric coordinates
                    float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
                    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
                    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

                    // Check if point is in triangle
                    return ( (u > 0) && (v > 0) && (u + v < 1) );
                }

                Vector3 pointA;
                Vector3 pointB;
                Vector3 pointC;
            };

        } // math
    } // spatial
} // opencog

#endif // _SPATIAL_MATH_TRIANGLE_H_
