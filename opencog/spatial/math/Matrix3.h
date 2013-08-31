/*
 * opencog/spatial/math/Matrix3.h
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

#ifndef _SPATIAL_MATH_MATRIX3_H_
#define _SPATIAL_MATH_MATRIX3_H_

#include <vector>
#include <string>

#include <opencog/spatial/math/Vector3.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        namespace math {

            class Matrix3
            {
            public:
                // All elements of the matrix are 0
                static const Matrix3 ZERO;
                // An identity 3x3 matrix
                static const Matrix3 IDENTITY;

                /**
                 * Simple constructor
                 */
                Matrix3( void );

                /**
                 * Build a matrix using the given values
                 * @param m matrix values
                 */
                Matrix3( const std::vector<std::vector<double> >& m );

                /**
                 * Build a matrix using the given values
                 * @param e00
                 * @param e01
                 * @param e02
                 * @param e10
                 * @param e11
                 * @param e12
                 * @param e20
                 * @param e21
                 * @param e22
                 */
                Matrix3( double e00, double e01, double e02,
                         double e10, double e11, double e12,
                         double e20, double e21, double e22 );

                /**
                 * A copy constructor
                 * @param matrix
                 */
                Matrix3( const Matrix3& matrix );

                inline virtual ~Matrix3( void ) { };

                /**
                 * Compute the matrix determinant
                 * @return determinant of this matrix
                 */
                double determinant( void ) const;

                /**
                 * Do the transpose of a copy of this matrix
                 * @return Transposed matrix
                 */
                Matrix3 transpose( void ) const;

                /**
                 * Inverse a copy of this matrix
                 * @return an inversed matrix
                 */
                Matrix3 inverse( void ) const;

                /**
                 * Invert this matrix using cofactors
                 * @param rkInverse
                 * @param fTolerance
                 * @return
                 */
                bool inverse( Matrix3& rkInverse, double fTolerance) const;

                /**
                 * Set a new value for a specific matrix element
                 * @param x
                 * @param y
                 * @param value
                 */
                void set( int x, int y, double value );

                /**
                 * Get the value of a specific matrix element
                 * @param x
                 * @param y
                 * @return
                 */
                double get( int x, int y );

                /**
                 * Multiply a given vector by this matrix and return the result
                 * @param point
                 * @return
                 */
                Vector3 operator*( const Vector3& point );

                /*
                 */
                std::string toString( void ) const;

                std::vector<std::vector<double> > m;

            }; // Matrix3

        } // math
    } // spatial
/** @}*/
} // opencog

#endif // _SPATIAL_MATH_MATRIX3_H_
