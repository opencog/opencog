/*
 * opencog/spatial/math/Quaternion.h
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

#ifndef _SPATIAL_MATH_QUATERNION_H_
#define _SPATIAL_MATH_QUATERNION_H_

#include <opencog/spatial/math/Vector3.h>
#include <opencog/spatial/math/Matrix3.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        namespace math {

            class Quaternion
            {
            public:

                /**
                 * Simple constructor
                 */
                Quaternion( void );

                /**
                 * Copy constructor
                 * @param q
                 */
                Quaternion(const Quaternion& q);

                /**
                 * Custom constructor
                 * @param x
                 * @param y
                 * @param z
                 * @param w
                 */
                Quaternion(double x, double y, double z, double w);

                /**
                 * Use an axis coordination and a given angle to build the quaternion
                 * @param axis
                 * @param angle
                 */
                Quaternion(const Vector3& axis, double angle);

                inline virtual ~Quaternion( void ) 
                { 
                };

                /**
                 * Return the length of this quaternion
                 * @return
                 */
                double length(void) const;

                /**
                 * Rebuild this quaternion using the given axis and angle
                 * @param axis  rotation axis, unit vector
                 * @param angle the rotation angle (radians)
                 * @return this
                 */
                Quaternion& set(const Vector3& axis, double angle);


                /**
                 * Scale self by a given scale
                 * @param scale
                 * @return
                 */
                Quaternion& operator*=(double scale);

                /**
                 * Divide self by a given scale
                 * @param scale
                 * @return
                 */
                Quaternion& operator/=(double scale);

                /**
                 * Compute the dot product between this quaternion and a given one
                 * @param q
                 * @return
                 */
                double dot(const Quaternion& q) const;

                /**
                 * Multiply self by a given Quaternion
                 * @param q
                 * @return
                 */
                Quaternion& operator*=( const Quaternion& q);

                /**
                 * Compute a interpolated quaternion using this and a given one by a specific step t,
                 * and update self with the new value
                 * @param q
                 * @param t factor / should be between 0-1
                 * @return
                 */
                Quaternion& interpolateThis(const Quaternion& q, double t);

                /**
                 * Normalize self
                 * @return
                 */
                Quaternion& normalize(void);

                /**
                 * Compute a new interpolated Quaternion copy
                 * @param q
                 * @param t factor / should be between 0-1
                 * @return
                 */
                Quaternion interpolate(const Quaternion& q, double t);

                /**
                 * Rotate a vector using the accumulated rotation of this quaternion
                 * @param v
                 * @return
                 */
                Vector3 rotate( const Vector3& v ) const;

                /**
                 * Build a rotation matrix from this quaternion
                 * @return
                 */
                Matrix3 getRotationMatrix( void );

                /**
                 * Add a given quaternion to this one
                 * @param q
                 * @return
                 */
                Quaternion& operator+=( const Quaternion& q );

                /**
                 * Add a given quaternion to a copy of this one
                 * @param q
                 * @return
                 */
                Quaternion operator+( const Quaternion& q );

                /**
                 * Invert the signal of this quaternion
                 * @return
                 */
                Quaternion operator-( void );

                /**
                 * Get roll angle
                 * @return
                 */
                double getRoll( void ) const;

                /**
                 * Get roll angle
                 * @param reprojectAxis
                 * @return
                 */
                double getRoll( bool reprojectAxis) const;

                /**
                 * Get pitch angle
                 * @return
                 */
                double getPitch(void) const;

                /**
                 * Get pitch angle
                 * @param reprojectAxis
                 * @return
                 */
                double getPitch(bool reprojectAxis) const;

                /**
                 * Get yaw angle
                 * @return
                 */
                double getYaw(void) const;

                /**
                 * Get yaw angle
                 * @param reprojectAxis
                 * @return
                 */
                double getYaw(bool reprojectAxis) const;

                /*
                 */
                bool operator==( const Quaternion& q ) const;
                bool operator!=( const Quaternion& q ) const;

                double x;
                double y;
                double z;
                double w;

                inline std::string toString( void ) const 
                {
                    std::stringstream text;
                    text << "x: " << x << " y: " << y << " z: " << z << " w: " << w;
                    text << " roll: " << getRoll( ) << " pitch: " << getPitch( ) << " yaw: " << getYaw( );
                    return text.str( );
                }

            }; // Quaternion

        } // math
    } // spatial
/** @}*/
} // opencog

#endif // _SPATIAL_MATH_QUATERNION_H_
