/*
 * opencog/spatial/math/BoundingBox.cc
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

#include <opencog/spatial/math/BoundingBox.h>
#include <opencog/spatial/Entity.h>
#include <limits>

using namespace opencog;
using namespace opencog::spatial::math;

BoundingBox::BoundingBox( Entity* entity )
{
    this->entity = entity;
    update( );
}

const std::vector<Vector3>& BoundingBox::getAllCorners( ) const
{
    return corners;
}

/**
 * Update the bounding box rebuilding the corners positions
 */
void BoundingBox::update( void )
{
    // rebuild corners
    buildCorners( );
    buildEdges( );
    buildFaces( );
}

double BoundingBox::distanceTo( const Vector3& point ) const
{
    double distance = std::numeric_limits<double>::max( );
    unsigned int i;
    for ( i = 0; i < edges.size( ); ++i ) {
        double candidateDistance = edges[i].distanceTo( point );
        if ( candidateDistance < distance ) {
            distance = candidateDistance;
        } // if
    } // for
    return distance;
}

const Vector3& BoundingBox::getCorner(CORNER cornerToGet) const
{
    
    switch (cornerToGet) {
    default:
    case NEAR_LEFT_BOTTOM:
        return this->corners[0];
    case FAR_LEFT_BOTTOM:
        return this->corners[1];
    case FAR_RIGHT_BOTTOM:
        return this->corners[2];
    case NEAR_RIGHT_BOTTOM:
        return this->corners[3];
    case NEAR_LEFT_TOP:
        return this->corners[4];
    case FAR_LEFT_TOP:
        return this->corners[5];
    case FAR_RIGHT_TOP:
        return this->corners[6];
    case NEAR_RIGHT_TOP:
        return this->corners[7];
    }
}

const std::vector<LineSegment>& BoundingBox::getAllEdges( void ) const
{
    return edges;
}

bool BoundingBox::isInside( const Vector3& point ) const
{
    // ATTENTION: this is a limit implementation which ignores the height of the object
    // if you need a volume inside test, you must reimplement this using another algorithm.
    /*
     * P is the point.
     * C is a corner of the rectangle.
     * v1 and v2 are the two vectors that define the sides (with C as origin).
     * v = P-C
     * P is in the rectangle if and only if
     * 0 <= dot_product(v,v1) <= dot_product(v1,v1) and 0<=dot_product(v,v2)<=dot_product(v2,v2)
     */
    Vector3 v1 = this->corners[1] - this->corners[0];
    Vector3 v2 = this->corners[3] - this->corners[0];
    Vector3 v3 = this->corners[4] - this->corners[0];
    Vector3 v = point - this->corners[0] ;
    double dp1 = v.dotProduct( v1 );
    double dp2 = v1.dotProduct( v1 );

    double dp3 = v.dotProduct( v2 );
    double dp4 = v2.dotProduct( v2 );

    double dp5 = v.dotProduct(v3);
    double dp6 = v3.dotProduct(v3);

    return ( dp1 >= 0 && dp1 <= dp2 &&
             dp3 >= 0 && dp3 <= dp4 &&
             dp5 >= 0 && dp5 <= dp6 );
}

const std::vector<SquareFace>& BoundingBox::getAllFaces( )
{
    return squareFaces;
}


void BoundingBox::buildEdges( void )
{
    this->edges.clear( );
    this->cornerEdges.clear( );

    edges.push_back( LineSegment( this->corners[0], this->corners[1] ) );
    edges.push_back( LineSegment( this->corners[1], this->corners[2] ) );
    edges.push_back( LineSegment( this->corners[2], this->corners[3] ) );
    edges.push_back( LineSegment( this->corners[3], this->corners[0] ) );

    edges.push_back( LineSegment( this->corners[4], this->corners[5] ) );
    edges.push_back( LineSegment( this->corners[5], this->corners[6] ) );
    edges.push_back( LineSegment( this->corners[6], this->corners[7] ) );
    edges.push_back( LineSegment( this->corners[7], this->corners[4] ) );

    edges.push_back( LineSegment( this->corners[0], this->corners[4] ) );
    edges.push_back( LineSegment( this->corners[1], this->corners[5] ) );
    edges.push_back( LineSegment( this->corners[2], this->corners[6] ) );
    edges.push_back( LineSegment( this->corners[3], this->corners[7] ) );

    unsigned int i;
    for( i = 0; i < edges.size( ); ++i ) {
        cornerEdges[edges[i].pointA].push_back( &edges[i] );
        cornerEdges[edges[i].pointB].push_back( &edges[i] );
    } // for
}

void BoundingBox::buildFaces( void )
{
    this->squareFaces.clear( );

    // top
    squareFaces.push_back( SquareFace(corners[4], corners[5], corners[6], corners[7] ) );
    // right
    squareFaces.push_back( SquareFace(corners[1], corners[2], corners[6], corners[5] ) );
    // bottom
    squareFaces.push_back( SquareFace(corners[1], corners[0], corners[3], corners[2] ) );
    // left
    squareFaces.push_back( SquareFace(corners[3], corners[0], corners[4], corners[7] ) );
    // front
    squareFaces.push_back( SquareFace(corners[4], corners[0], corners[1], corners[5] ) );
    // back
    squareFaces.push_back( SquareFace(corners[2], corners[3], corners[7], corners[6] ) );

}

void BoundingBox::buildCorners( void )
{
    this->corners.clear( );

    const Vector3& position = this->entity->getPosition( );
    double radius = this->entity->getExpansionRadius( );
    //std::cout << "Radius: " << radius << std::endl;
    Vector3 front = this->entity->getDirection() * (this->entity->getLength( ) / 2 + radius);
    Vector3 up = Vector3::Z_UNIT * (this->entity->getHeight() / 2 + radius);

    Vector3 side = Quaternion( Vector3::Z_UNIT, M_PI / 2.0 ).rotate(this->entity->getDirection()) *
                   (this->entity->getWidth() / 2 + radius);


    // bottom left near
    corners.push_back( position + -front - up - side );
    // bottom left far
    corners.push_back( position + front - up - side );    
    // bottom right far
    corners.push_back( position + front - up + side );
    // bottom right near
    corners.push_back( position + -front - up + side );
    // top left near
    corners.push_back( position + -front + up - side );
    // top left far
    corners.push_back( position + front + up - side );
    // top right far
    corners.push_back( position + front + up + side );
    // top right near
    corners.push_back( position + -front + up + side );
}

bool BoundingBox::operator==( const BoundingBox& bb ) const
{
    return this->corners == bb.corners;
}

BoundingBox& BoundingBox::operator=( const BoundingBox & bb )
{
    this->entity = bb.entity;
    this->squareFaces = bb.squareFaces;
    this->edges = bb.edges;
    this->corners = bb.corners;

    return *this;
}

const std::list<LineSegment*>& BoundingBox::getEdges( const Vector3& corner ) const
{
    std::map<Vector3, std::list<LineSegment*> >::const_iterator it = 
        this->cornerEdges.find( corner );
    
    if ( it != this->cornerEdges.end( ) ) {
        return it->second;
    } // if
    throw opencog::NotFoundException( "BoundingBox there is no edges connected to the given corner[%s]", 
                                      TRACE_INFO, corner.toString( ).c_str( ) );
}
