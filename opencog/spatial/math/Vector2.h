/*
 * opencog/spatial/math/Vector2.h
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

#ifndef _SPATIAL_MATH_VECTOR2_H_
#define _SPATIAL_MATH_VECTOR2_H_

#include <cassert>
#include <cmath>
#include <sstream>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        namespace math {
            class Vector2
            {
            public:
                float x;
                float y;

                inline Vector2( ) 
                {
                }

                inline Vector2(const float x, const float y ) : x( x ), y( y ) 
                {
                }

                inline explicit Vector2( const float scaler ) : x( scaler), y( scaler ) 
                {
                }

                inline Vector2( const Vector2& vector ) : x( vector.x ), y( vector.y ) 
                {
                }

                inline Vector2& operator = ( const Vector2& vector ) 
                {
                    x = vector.x;
                    y = vector.y;

                    return *this;
                }

                inline Vector2& operator = ( const float scalar) 
                {
                    x = scalar;
                    y = scalar;

                    return *this;
                }

                inline bool operator == ( const Vector2& vector ) const 
                {
                    return ( x == vector.x && y == vector.y );
                }

                inline bool operator != ( const Vector2& vector ) const 
                {
                    return ( x != vector.x || y != vector.y  );
                }

                // arithmetic operations
                inline Vector2 operator + ( const Vector2& vector ) const 
                {
                    return Vector2( x + vector.x, y + vector.y);
                }

                inline Vector2 operator - ( const Vector2& vector ) const 
                {
                    return Vector2( x - vector.x, y - vector.y );
                }

                inline Vector2 operator * ( const float scalar ) const 
                {
                    return Vector2( x * scalar, y * scalar );
                }

                inline Vector2 operator * ( const Vector2& vector) const 
                {
                    return Vector2( x * vector.x,  y * vector.y );
                }

                inline Vector2 operator / ( const float scalar ) const 
                {
                    assert( scalar != 0.0 );

                    float fInv = 1.0 / scalar;

                    return Vector2( x * fInv, y * fInv);
                }

                inline Vector2 operator / ( const Vector2& vector) const 
                {
                    return Vector2( x / vector.x, y / vector.y );
                }

                inline const Vector2& operator +( ) const 
                {
                    return *this;
                }

                inline Vector2 operator -( ) const 
                {
                    return Vector2(-x, -y);
                }

                // overloaded operators to help Vector2
                inline friend Vector2 operator * ( const float scalar, const Vector2& vector ) 
                {
                    return Vector2( scalar * vector.x, scalar * vector.y );
                }

                inline friend Vector2 operator / ( const float scalar, const Vector2& vector ) 
                {
                    return Vector2( scalar / vector.x, scalar / vector.y );
                }

                inline friend Vector2 operator + (const Vector2& lhs, const float vector) 
                {
                    return Vector2( lhs.x + vector, lhs.y + vector);
                }

                inline friend Vector2 operator + (const float leftVector, const Vector2& vector) 
                {
                    return Vector2( leftVector + vector.x, leftVector + vector.y );
                }

                inline friend Vector2 operator - (const Vector2& leftVector, const float vector) 
                {
                    return Vector2( leftVector.x - vector, leftVector.y - vector );
                }

                inline friend Vector2 operator - (const float leftVector, const Vector2& vector) 
                {
                    return Vector2( leftVector - vector.x, leftVector - vector.y );
                }
                // arithmetic updates
                inline Vector2& operator += ( const Vector2& vector ) 
                {
                    x += vector.x;
                    y += vector.y;

                    return *this;
                }

                inline Vector2& operator += ( const float scaler ) 
                {
                    x += scaler;
                    y += scaler;

                    return *this;
                }

                inline Vector2& operator -= ( const Vector2& vector ) 
                {
                    x -= vector.x;
                    y -= vector.y;

                    return *this;
                }

                inline Vector2& operator -= ( const float scaler ) 
                {
                    x -= scaler;
                    y -= scaler;

                    return *this;
                }

                inline Vector2& operator *= ( const float scalar ) 
                {
                    x *= scalar;
                    y *= scalar;

                    return *this;
                }

                inline Vector2& operator *= ( const Vector2& vector ) 
                {
                    x *= vector.x;
                    y *= vector.y;

                    return *this;
                }

                inline Vector2& operator /= ( const float scalar ) 
                {
                    assert( scalar != 0.0 );

                    float fInv = 1.0 / scalar;

                    x *= fInv;
                    y *= fInv;

                    return *this;
                }

                inline Vector2& operator /= ( const Vector2& vector ) 
                {
                    x /= vector.x;
                    y /= vector.y;

                    return *this;
                }

                /** Returns the length (magnitude) of the vector.
                    @warning
                    This operation requires a square root and is expensive in
                    terms of CPU operations. If you don't need to know the exact
                    length (e.g. for just comparing lengths) use squaredLength()
                    instead.
                */
                inline float length () const 
                {
                    return std::sqrt( x * x + y * y );
                }

                /** Returns the square of the length(magnitude) of the vector.
                    @remarks
                    This  method is for efficiency - calculating the actual
                    length of a vector requires a square root, which is expensive
                    in terms of the operations required. This method returns the
                    square of the length of the vector, i.e. the same as the
                    length but before the square root is taken. Use this if you
                    want to find the longest / shortest vector without incurring
                    the square root.
                */
                inline float squaredLength () const 
                {
                    return x * x + y * y;
                }

                /** Calculates the dot (scalar) product of this vector with another.
                    @remarks
                    The dot product can be used to calculate the angle between 2
                    vectors. If both are unit vectors, the dot product is the
                    cosine of the angle; otherwise the dot product must be
                    divided by the product of the lengths of both vectors to get
                    the cosine of the angle. This result can further be used to
                    calculate the distance of a point from a plane.
                    @param
                    vec Vector with which to calculate the dot product (together
                    with this one).
                    @returns
                    A float representing the dot product value.
                */
                inline float dotProduct(const Vector2& vector) const 
                {
                    return x * vector.x + y * vector.y;
                }

                /** Normalises the vector.
                    @remarks
                    This method normalises the vector such that it's
                    length / magnitude is 1. The result is called a unit vector.
                    @note
                    This function will not crash for zero-sized vectors, but there
                    will be no changes made to their components.
                    @returns The previous length of the vector.
                */
                inline float normalise( ) 
                {
                    float length = std::sqrt( x * x + y * y);

                    // Will also work for zero-sized vectors, but will change nothing
                    if ( length > 1e-08 ) {
                        float invLength = 1.0 / length;
                        x *= invLength;
                        y *= invLength;
                    } // if

                    return length;
                }



                /** Returns a vector at a point half way between this and the passed
                    in vector.
                */
                inline Vector2 midPoint( const Vector2& vector ) const 
                {
                    return Vector2( ( x + vector.x ) * 0.5, ( y + vector.y ) * 0.5 );
                }

                /** Returns true if the vector's scalar components are all greater
                    that the ones of the vector it is compared against.
                */
                inline bool operator < ( const Vector2& vector ) const 
                {
                    return ( x < vector.x && y < vector.y );
                }

                /** Returns true if the vector's scalar components are all smaller
                    that the ones of the vector it is compared against.
                */
                inline bool operator > ( const Vector2& vector ) const 
                {
                    return ( x > vector.x && y > vector.y );
                }

                /** Sets this vector's components to the minimum of its own and the
                    ones of the passed in vector.
                    @remarks
                    'Minimum' in this case means the combination of the lowest
                    value of x, y and z from both vectors. Lowest is taken just
                    numerically, not magnitude, so -1 < 0.
                */
                inline void makeFloor( const Vector2& vector ) 
                {
                    if ( vector.x < x ) x = vector.x;
                    if ( vector.y < y ) y = vector.y;
                }

                /** Sets this vector's components to the maximum of its own and the
                    ones of the passed in vector.
                    @remarks
                    'Maximum' in this case means the combination of the highest
                    value of x, y and z from both vectors. Highest is taken just
                    numerically, not magnitude, so 1 > -3.
                */
                inline void makeCeil( const Vector2& vector ) 
                {
                    if ( vector.x > x ) x = vector.x;
                    if ( vector.y > y ) y = vector.y;
                }

                /** Generates a vector perpendicular to this vector (eg an 'up' vector).
                    @remarks
                    This method will return a vector which is perpendicular to this
                    vector. There are an infinite number of possibilities but this
                    method will guarantee to generate one of them.
                */
                inline Vector2 perpendicular(void) const 
                {
                    return Vector2 (-y, x);
                }
                /** Calculates the 2 dimensional cross-product of 2 vectors, which results
                    in a single floating point value which is 2 times the area of the triangle.
                */
                inline double crossProduct( const Vector2& vector ) const 
                {
                    return x * vector.y - y * vector.x;
                }

                /** Returns true if this vector is zero length. */
                inline bool isZeroLength(void) const 
                {
                    float sqlen = (x * x) + (y * y);
                    return (sqlen < (1e-06 * 1e-06));

                }

                /** As normalise, except that this vector is unaffected and the
                    normalised vector is returned as a copy. */
                inline Vector2 normalisedCopy(void) const 
                {
                    Vector2 result = *this;
                    result.normalise();
                    return result;
                }

                /** Calculates a reflection vector to the plane with the given normal .
                    @remarks NB assumes 'this' is pointing AWAY FROM the plane, invert if it is not.
                */
                inline Vector2 reflect(const Vector2& normal) const 
                {
                    return Vector2( *this - ( 2 * this->dotProduct(normal) * normal ) );
                }

                inline std::string toString( void ) const 
                {
                    std::stringstream response;
                    response << x << " " << y;
                    return response.str( );
                }

            };
        } // math
    } // spatial
} // opencog

#endif // _SPATIAL_MATH_VECTOR2_H_
