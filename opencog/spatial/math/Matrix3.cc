/*
 * opencog/spatial/math/Matrix3.cc
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

#include <opencog/spatial/math/Matrix3.h>
#include <sstream>

using namespace opencog;
using namespace opencog::spatial::math;


// All elements of the matrix are 0
const Matrix3 Matrix3::ZERO;
// An identity 3x3 matrix
const Matrix3 Matrix3::IDENTITY( 1, 0, 0, 0, 1, 0, 0, 0, 1 );

Matrix3::Matrix3( void )
{
    unsigned int i;
    unsigned int j;
    for ( i = 0; i < 3; ++i ) {
        std::vector<double> col;
        for ( j = 0; j < 3; ++j ) {
            col.push_back( 0 );
        } // for
        m.push_back( col );
    } // for
}

/**
 * Build a matrix using the given values
 * @param m matrix values
 */
Matrix3::Matrix3( const std::vector<std::vector<double> >& m )
{
    //OC_ASSERT((m.size( ) == 3 && m[0].size( ) == 3 && m[1].size( ) == 3 && m[2].size( ) == 3 );
    unsigned int i;
    unsigned int j;
    for ( i = 0; i < 4; ++i ) {
        std::vector<double> col;
        for ( j = 0; j < 4; ++j ) {
            col.push_back( m[i][j] );
        } // for
        this->m.push_back( col );
    } // for

}

Matrix3::Matrix3( double e00, double e01, double e02,
                  double e10, double e11, double e12,
                  double e20, double e21, double e22 )
{
    std::vector<double> row1;
    row1.push_back( e00 ); row1.push_back( e01 ); row1.push_back( e02 );
    std::vector<double> row2;
    row2.push_back( e10 ); row2.push_back( e11 ); row2.push_back( e12 );
    std::vector<double> row3;
    row3.push_back( e20 ); row3.push_back( e21 ); row3.push_back( e22 );

    this->m.push_back( row1 );
    this->m.push_back( row2 );
    this->m.push_back( row3 );
}

Matrix3::Matrix3( const Matrix3& matrix )
{
    this->m = matrix.m;
}

/**
 * Compute the matrix determinant
 * @return determinant of this matrix
 */
double Matrix3::determinant( void ) const
{
    double cofactor00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    double cofactor10 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    double cofactor20 = m[1][0] * m[2][1] - m[1][1] * m[2][0];

    double det = m[0][0] * cofactor00 + m[0][1] * cofactor10 + m[0][2] * cofactor20;

    return det;
}

Matrix3 Matrix3::transpose( void ) const
{
    return Matrix3(m[0][0], m[1][0], m[2][0],
                   m[0][1], m[1][1], m[2][1],
                   m[0][2], m[1][2], m[2][2]);
}

Matrix3 Matrix3::inverse( void ) const
{
    Matrix3 kInverse = Matrix3::ZERO;
    inverse( kInverse, 1e-06 );
    return kInverse;
}

bool Matrix3::inverse( Matrix3& rkInverse, double fTolerance) const
{
    // Invert a 3x3 using cofactors.  This is about 8 times faster than
    // the Numerical Recipes code which uses Gaussian elimination.

    rkInverse.m[0][0] = m[1][1] * m[2][2] -
                        m[1][2] * m[2][1];
    rkInverse.m[0][1] = m[0][2] * m[2][1] -
                        m[0][1] * m[2][2];
    rkInverse.m[0][2] = m[0][1] * m[1][2] -
                        m[0][2] * m[1][1];
    rkInverse.m[1][0] = m[1][2] * m[2][0] -
                        m[1][0] * m[2][2];
    rkInverse.m[1][1] = m[0][0] * m[2][2] -
                        m[0][2] * m[2][0];
    rkInverse.m[1][2] = m[0][2] * m[1][0] -
                        m[0][0] * m[1][2];
    rkInverse.m[2][0] = m[1][0] * m[2][1] -
                        m[1][1] * m[2][0];
    rkInverse.m[2][1] = m[0][1] * m[2][0] -
                        m[0][0] * m[2][1];
    rkInverse.m[2][2] = m[0][0] * m[1][1] -
                        m[0][1] * m[1][0];

    double fDet =
        m[0][0] * rkInverse.m[0][0] +
        m[0][1] * rkInverse.m[1][0] +
        m[0][2] * rkInverse.m[2][0];

    if ( std::fabs(fDet) <= fTolerance ) {
        return false;
    } // if

    double fInvDet = 1.0 / fDet;
    for (int iRow = 0; iRow < 3; iRow++) {
        for (int iCol = 0; iCol < 3; iCol++) {
            rkInverse.m[iRow][iCol] *= fInvDet;
        } // for
    } // for

    return true;
}

void Matrix3::set( int x, int y, double value )
{
//  OC_ASSERT( x >= 0 && x < 3 && y >= 0 && y < 3 );
    this->m[x][y] = value;
}

double Matrix3::get( int x, int y )
{
//  OC_ASSERT( x >= 0 && x < 3 && y >= 0 && y < 3 );
    return this->m[x][y];
}

Vector3 Matrix3::operator*( const Vector3& point )
{
    return Vector3(
               m[0][0]*point.x + m[0][1]*point.y + m[0][2]*point.z,
               m[1][0]*point.x + m[1][1]*point.y + m[1][2]*point.z,
               m[2][0]*point.x + m[2][1]*point.y + m[2][2]*point.z
           );
}


std::string Matrix3::toString( void ) const
{
    int i;
    int j;

    std::stringstream message;

    message << "{";
    for ( i = 0; i < 3; ++i ) {
        message << "{";
        for ( j = 0; j < 3; ++j ) {
            message << this->m[i][0];
            if ( j < 2 ) {
                message << ",";
            } // if
        } // for
        message << "}";
        if ( i < 2 ) {
            message << ",";
        } // if
    } // for
    message << "}";

    return message.str( );
}
