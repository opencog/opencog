/*
 * opencog/spatial/math/Face.cc
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

#include <opencog/spatial/math/Face.h>
#include <opencog/spatial/math/Matrix3.h>

using namespace opencog;
using namespace opencog::spatial::math;

Face::Face( const Vector3& pointA, const Vector3& pointB, const Vector3& pointC ) :
        pointA(pointA), pointB(pointB), pointC(pointC)
{

    double det = Matrix3(
                     pointA.x, pointA.y, pointA.z,
                     pointB.x, pointB.y, pointB.z,
                     pointC.x, pointC.y, pointC.z
                 ).determinant( );
    this->direction = ( det >= 0) ? Face::COUNTER_CLOCK_WISE : Face::CLOCK_WISE;
}

const Vector3 Face::getNormal( void ) const
{
    return getPlane( ).normal;
}

Face::POLYGON_DIRECTION Face::getPolygonDirection( void ) const
{
    return direction;
}

Plane Face::getPlane( ) const
{
    Plane plane( pointA, pointB, pointC);
    plane.distanceFromOrigo = -plane.distanceFromOrigo;
    return plane;
}

Face& Face::addSelf( const Vector3& vector )
{
    this->pointA += vector;
    this->pointB += vector;
    this->pointC += vector;
    return *this;
}

std::string Face::toString( void ) const
{
    return pointA.toString() + "|" + pointB.toString() + "|" + pointC.toString();
}

