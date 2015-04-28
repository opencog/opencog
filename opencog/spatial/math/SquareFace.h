/*
 * opencog/spatial/math/SquareFace.h
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

#ifndef _SPATIAL_MATH_SQUAREFACE_H_
#define _SPATIAL_MATH_SQUAREFACE_H_

#include <opencog/spatial/math/Face.h>
#include <vector>

namespace opencog {
/** \addtogroup grp_spatial
 *  @{
 */
namespace spatial {
namespace math {

class SquareFace : public Face
{
public:

    Vector3 pointD;

    /**
     * Build a face using four points
     *
     * @param pointA
     * @param pointB
     * @param pointC
     * @param pointD
     */
    SquareFace( Vector3 pointA, Vector3 pointB, Vector3 pointC, Vector3 pointD ) : 
        Face( pointA, pointB, pointC ), pointD( pointD )  
    {

    }

    Face& addSelf( const Vector3& vector ) 
    {
        Face::addSelf( vector );
        this->pointD + vector;
        return *this;
    }

    /**
     * Get the edges of the face
     *    A+-----+B
     *    /     /
     *   /     /
     * D+-----+C
     *
     * @return
     */
    std::vector<LineSegment> getAllEdges( void ) 
    {
        std::vector<LineSegment> edges;
        edges.push_back( LineSegment( pointA, pointB ) );
        edges.push_back( LineSegment( pointB, pointC ) );
        edges.push_back( LineSegment( pointC, pointD ) );
        edges.push_back( LineSegment( pointD, pointA ) );
        return edges;
    }

    std::string toString( void ) const 
    {
        return Face::toString( ) + "|" + pointD.toString();
    }


}; // SquareFace

} } } // namespace opencog::spatial::math

#endif // _SPATIAL_MATH_SQUAREFACE_H_
