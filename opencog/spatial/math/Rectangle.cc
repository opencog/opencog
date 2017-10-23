/*
 * opencog/spatial/math/Rectangle.cc
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

#include <opencog/spatial/math/Rectangle.h>
#include <opencog/spatial/math/LineSegment.h>

#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

using namespace opencog;
using namespace opencog::spatial::math;

Rectangle::Rectangle( const Rectangle& other ) : leftTopCorner( other.leftTopCorner ), rightTopCorner( other.rightTopCorner ), rightBottomCorner( other.rightBottomCorner )
{
}

Rectangle::Rectangle( const Vector3& leftTopCorner, const Vector3& rightTopCorner, const Vector3& rightBottomCorner )
{
    Vector3 vec1 = rightTopCorner - leftTopCorner;
    vec1.normalise( );
    Vector3 vec2 = rightBottomCorner - rightTopCorner;
    double rightSideLength = vec2.length( );
    vec2.normalise( );

    double cosAngle = vec1.dotProduct( vec2 );
    if ( cosAngle < -0.01 || cosAngle > 0.01 ) {
        throw opencog::InvalidParamException( TRACE_INFO, "The given points must have 1.57 radians between its sides, but was %f radians. Cos: %f (p1: %s, p2: %s, p3: %s)", std::acos(cosAngle), cosAngle, leftTopCorner.toString().c_str(), rightTopCorner.toString().c_str(), rightBottomCorner.toString().c_str() );
    } // if

    this->leftTopCorner = leftTopCorner;
    this->rightTopCorner = rightTopCorner;
    this->rightBottomCorner = rightBottomCorner;
    this->leftBottomCorner = Vector3( ( vec2 * rightSideLength ) + leftTopCorner );
}

bool Rectangle::isInside( const Vector3& point )
{
    /**
     * Another answer:
     * P is the point.
     * C is a corner of the rectangle.
     * v1 and v2 are the two vectors that define the sides (with C as origin).
     * v = P-C
     * P is in the rectangle if and only if
     * 0<=dot_product(v,v1)<=dot_product(v1,v1) and 0<=dot_product(v,v2)<=dot_product(v2,v2)
     */
    Vector3 v = point - rightTopCorner;
    Vector3 v1 = leftTopCorner - rightTopCorner;
    Vector3 v2 = rightBottomCorner - rightTopCorner;
    double dot1 = v.dotProduct( v1);
    double dot2 = v1.dotProduct( v1);
    double dot3 = v.dotProduct( v2);
    double dot4 = v2.dotProduct( v2);
    return ( dot1 > 0.0 && dot2 > dot1 && dot3 > 0.0 && dot4 > dot3 );
}

Rectangle& Rectangle::operator=( const Rectangle & o )
{
    this->leftTopCorner = o.leftTopCorner;
    this->rightTopCorner = o.rightTopCorner;
    this->rightBottomCorner = o.rightBottomCorner;
    return *this;
}

bool Rectangle::operator==( const Rectangle& o ) const
{
    return( this->leftTopCorner == o.leftTopCorner &&
            this->rightTopCorner == o.rightTopCorner &&
            this->rightBottomCorner == o.rightBottomCorner );
}
