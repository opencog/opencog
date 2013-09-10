/*
 * opencog/spatial/math/Vector4.h
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

#ifndef _SPATIAL_MATH_VECTOR4_H_
#define _SPATIAL_MATH_VECTOR4_H_

#include <opencog/spatial/math/Vector3.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        namespace math {

            class Vector4
            {
            public:
                double x;
                double y;
                double z;
                double w;

                /**
                 * Build a vector from four coords (x,y,z,w)
                 * @param x
                 * @param y
                 * @param z
                 * @param w
                 */
                inline Vector4( double x, double y, double z, double w ) : x(x), y(y), z(z), w(w)
                {
                }

                /**
                 * Simple constructor
                 */
                inline Vector4( void ) : x(0), y(0), z(0), w(0)
                    {
                }

                /**
                 * Copy constructor
                 * @param vector
                 */
                inline Vector4( const Vector4& vector )
                {
                    *this = vector;
                }

                /**
                 * Build a 4 dimensional vector from a 3 dimensional plus another coord
                 * @param vector
                 * @param w
                 */
                inline Vector4( const Vector3& vector, double w ) : x(vector.x), y(vector.y), z(vector.z), w(w)
                {
                }

                /**
                 * Compute the dot product of this and a given 4 dimensional vector
                 * @param vec
                 * @return
                 */
                inline double dot( const Vector4& vec ) const
                {
                    return x * vec.x + y * vec.y + z * vec.z + w * vec.w;
                }

                /**
                 * Compute the dot product of this and a given 3 dimensional vector
                 * @param vec
                 * @return
                 */
                inline double dot( const Vector3& vec ) const
                {
                    return x * vec.x + y * vec.y + z * vec.z + w;
                }

            }; // Vector4

        } // math
    } // spatial
} // opencog

#endif // _SPATIAL_MATH_VECTOR4_H_
