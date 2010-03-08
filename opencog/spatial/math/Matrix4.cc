/*
 * opencog/spatial/math/Matrix4.cc
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

#include <opencog/spatial/math/Matrix4.h>

using namespace opencog;
using namespace opencog::spatial::math;


const Matrix4 Matrix4::ZERO;
const Matrix4 Matrix4::IDENTITY( 1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1 );

Matrix4::Matrix4( void )
{
    unsigned int i;
    unsigned int j;
    for ( i = 0; i < 4; ++i ) {
        std::vector<double> col;
        for ( j = 0; j < 4; ++j ) {
            col.push_back( 0 );
        } // for
        m.push_back( col );
    } // for
}

Matrix4::Matrix4( const std::vector<std::vector<double> >& m )
{
    //opencog::cassert( m.size( ) == 4 && m[0].size( ) == 4 && m[1].size( ) == 4 && m[2].size( ) == 4 && m[3].size( ) == 4 );
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

Matrix4::Matrix4( double e00, double e01, double e02, double e03,
                  double e10, double e11, double e12, double e13,
                  double e20, double e21, double e22, double e23,
                  double e30, double e31, double e32, double e33 )
{
    std::vector<double> row1;
    row1.push_back( e00 ); row1.push_back( e01 ); row1.push_back( e02 ); row1.push_back( e03 );
    std::vector<double> row2;
    row2.push_back( e10 ); row2.push_back( e11 ); row2.push_back( e12 ); row2.push_back( e13 );
    std::vector<double> row3;
    row3.push_back( e20 ); row3.push_back( e21 ); row3.push_back( e22 ); row3.push_back( e23 );
    std::vector<double> row4;
    row4.push_back( e30 ); row4.push_back( e31 ); row4.push_back( e32 ); row4.push_back( e33 );

    this->m.push_back( row1 );
    this->m.push_back( row2 );
    this->m.push_back( row3 );
    this->m.push_back( row4 );
}

Matrix4::Matrix4( const Matrix4& matrix )
{
    *this = matrix;
}

double Matrix4::MINOR( int r0, int r1, int r2, int c0, int c1, int c2) const
{
    return m[r0][c0] * (m[r1][c1] * m[r2][c2] - m[r2][c1] * m[r1][c2]) -
           m[r0][c1] * (m[r1][c0] * m[r2][c2] - m[r2][c0] * m[r1][c2]) +
           m[r0][c2] * (m[r1][c0] * m[r2][c1] - m[r2][c0] * m[r1][c1]);
}

double Matrix4::determinant( void )
{
    return  m[0][0] * MINOR(1, 2, 3, 1, 2, 3) -
            m[0][1] * MINOR(1, 2, 3, 0, 2, 3) +
            m[0][2] * MINOR(1, 2, 3, 0, 1, 3) -
            m[0][3] * MINOR(1, 2, 3, 0, 1, 2);
}

Matrix4 Matrix4::transpose(void)
{
    return Matrix4(m[0][0], m[1][0], m[2][0], m[3][0],
                   m[0][1], m[1][1], m[2][1], m[3][1],
                   m[0][2], m[1][2], m[2][2], m[3][2],
                   m[0][3], m[1][3], m[2][3], m[3][3]);
}

Matrix4 Matrix4::inverse(void)
{
    double m00 = m[0][0], m01 = m[0][1], m02 = m[0][2], m03 = m[0][3];
    double m10 = m[1][0], m11 = m[1][1], m12 = m[1][2], m13 = m[1][3];
    double m20 = m[2][0], m21 = m[2][1], m22 = m[2][2], m23 = m[2][3];
    double m30 = m[3][0], m31 = m[3][1], m32 = m[3][2], m33 = m[3][3];

    double v0 = m20 * m31 - m21 * m30;
    double v1 = m20 * m32 - m22 * m30;
    double v2 = m20 * m33 - m23 * m30;
    double v3 = m21 * m32 - m22 * m31;
    double v4 = m21 * m33 - m23 * m31;
    double v5 = m22 * m33 - m23 * m32;

    double t00 = + (v5 * m11 - v4 * m12 + v3 * m13);
    double t10 = - (v5 * m10 - v2 * m12 + v1 * m13);
    double t20 = + (v4 * m10 - v2 * m11 + v0 * m13);
    double t30 = - (v3 * m10 - v1 * m11 + v0 * m12);

    double invDet = 1 / (t00 * m00 + t10 * m01 + t20 * m02 + t30 * m03);

    double d00 = t00 * invDet;
    double d10 = t10 * invDet;
    double d20 = t20 * invDet;
    double d30 = t30 * invDet;

    double d01 = - (v5 * m01 - v4 * m02 + v3 * m03) * invDet;
    double d11 = + (v5 * m00 - v2 * m02 + v1 * m03) * invDet;
    double d21 = - (v4 * m00 - v2 * m01 + v0 * m03) * invDet;
    double d31 = + (v3 * m00 - v1 * m01 + v0 * m02) * invDet;

    v0 = m10 * m31 - m11 * m30;
    v1 = m10 * m32 - m12 * m30;
    v2 = m10 * m33 - m13 * m30;
    v3 = m11 * m32 - m12 * m31;
    v4 = m11 * m33 - m13 * m31;
    v5 = m12 * m33 - m13 * m32;

    double d02 = + (v5 * m01 - v4 * m02 + v3 * m03) * invDet;
    double d12 = - (v5 * m00 - v2 * m02 + v1 * m03) * invDet;
    double d22 = + (v4 * m00 - v2 * m01 + v0 * m03) * invDet;
    double d32 = - (v3 * m00 - v1 * m01 + v0 * m02) * invDet;

    v0 = m21 * m10 - m20 * m11;
    v1 = m22 * m10 - m20 * m12;
    v2 = m23 * m10 - m20 * m13;
    v3 = m22 * m11 - m21 * m12;
    v4 = m23 * m11 - m21 * m13;
    v5 = m23 * m12 - m22 * m13;

    double d03 = - (v5 * m01 - v4 * m02 + v3 * m03) * invDet;
    double d13 = + (v5 * m00 - v2 * m02 + v1 * m03) * invDet;
    double d23 = - (v4 * m00 - v2 * m01 + v0 * m03) * invDet;
    double d33 = + (v3 * m00 - v1 * m01 + v0 * m02) * invDet;

    return Matrix4(
               d00, d01, d02, d03,
               d10, d11, d12, d13,
               d20, d21, d22, d23,
               d30, d31, d32, d33);
}

void Matrix4::set( int x, int y, double value )
{
    assert( x >= 0 && x < 4 && y >= 0 && y < 4 );
    this->m[x][y] = value;
}

double Matrix4::get( int x, int y ) const
{
    assert( x >= 0 && x < 4 && y >= 0 && y < 4 );
    return this->m[x][y];
}



Vector4 Matrix4::operator*( const Vector4& v ) const
{
    return Vector4(
               m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
               m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
               m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
               m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w
           );
}

Matrix4& Matrix4::operator=( const Matrix4 & m )
{
    this->m = m.m;
    return *this;
}

std::vector<double> Matrix4::getOpenGLArray( void ) const
{
    std::vector<double> array;
    int i;
    int j;

    for ( i = 0; i < 4; ++i ) {
        for ( j = 0; j < 4; ++j ) {
            array.push_back( m[j][i] );
        } // for
    } // for

    return array;
}

std::string Matrix4::toString( void ) const
{
    int i;
    int j;
    std::stringstream message;

    message << "{";
    for ( i = 0; i < 4; ++i ) {
        message << "{";
        for ( j = 0; j < 4; ++j ) {
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

