/*
 * opencog/spatial/math/Plane.cc
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

#include <opencog/spatial/math/Plane.h>
#include <opencog/spatial/math/Vector4.h>
#include <opencog/spatial/math/Matrix3.h>

using namespace opencog;
using namespace opencog::spatial::math;

Plane::Plane( void ) : normal(), distanceFromOrigo(0)
{
}

Plane::Plane( const Plane& plane ) : normal(plane.normal), distanceFromOrigo(plane.distanceFromOrigo)
{
}

Plane::Plane( const Vector3& normal, double distance )
{
    set( normal, distance );
}

Plane::Plane( const Vector3& pointA, const Vector3& pointB, const Vector3& pointC )
{
    Vector3 segmentBA = pointA - pointB;
    Vector3 segmentBC = pointC - pointB;
    this->normal = segmentBA.crossProduct(segmentBC);
    this->normal.normalise( );
    this->distanceFromOrigo = -this->normal.dotProduct(pointA);
}

void Plane::set( const Vector3& normal, double distance )
{
    this->normal = normal;
    this->distanceFromOrigo = -distance;
}

Vector4 Plane::getVector4( void )
{
    return Vector4( this->normal, -this->distanceFromOrigo );
}

double Plane::getDistance( const Vector3& point )
{
    return this->normal.dotProduct( point ) + distanceFromOrigo;
}

/**
 * Identifies the side of the plane a given point is positioned
 * @param point
 * @return
 */
Plane::SIDE Plane::getSide( const Vector3& point )
{
    double distance = getDistance( point );

    if ( distance < 0.0 )
        return Plane::NEGATIVE;

    if ( distance > 0.0 )
        return Plane::POSITIVE;

    return Plane::INTERSECT;
}

void Plane::transformSelf( const Matrix4& transformation )
{
    Vector4 result = transformation * getVector4( ); 
    set( Vector3( result.x, result.y, result.z ), result.w );
}

Vector3 Plane::getIntersectionPoint( const Plane& plane2, const Plane& plane3 )
{
    Matrix3 linearSystem (
        this->normal.x, this->normal.y, this->normal.z,
        plane2.normal.x, plane2.normal.y, plane2.normal.z,
        plane3.normal.x, plane3.normal.y, plane3.normal.z
    );
    if ( linearSystem.determinant() == 0 ) {
        throw opencog::NotFoundException( TRACE_INFO, "There is no intersection point!" );
    } // if

    return linearSystem.inverse( ) * ( -Vector3( this->distanceFromOrigo, plane2.distanceFromOrigo, plane3.distanceFromOrigo ) );
}

bool Plane::operator==( const Plane& other ) const
{
    return ( this->normal == other.normal && this->distanceFromOrigo == other.distanceFromOrigo );
}

std::string Plane::toString( void ) const
{
    std::stringstream msg;
    msg << this->normal.toString();
    msg <<  ", " << -this->distanceFromOrigo;
    return msg.str( );
}

