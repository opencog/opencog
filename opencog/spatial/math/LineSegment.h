/*
 * opencog/spatial/math/LineSegment.h
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

#ifndef _SPATIAL_MATH_LINESEGMENT_H_
#define _SPATIAL_MATH_LINESEGMENT_H_

#include <cassert>
#include <cmath>
#include <sstream>
#include <opencog/spatial/math/Line.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        namespace math {

            /**
             * This class represents a bidimensional line linesegment. A linesegment has two points in
             * Cartesian plane.
             */
            class LineSegment : public Line
            {
            public:

                static const double TOLERANCE_DISTANCE;

                inline LineSegment( const Vector3& pointA, const Vector3& pointB ) :
                    Line( pointA, pointB ) 
                { 
                }

                inline LineSegment( const LineSegment& segment ) :
                    Line( segment.pointA, segment.pointB ) 
                { 
                }

                inline virtual ~LineSegment( void ) 
                { 
                }

                /**
                 * Return the length of this segment
                 * @return length of the segment
                 */
                inline double length( ) 
                {
                    return (this->pointB - this->pointA ).length( );
                }


                /**
                 * Compute the nearest point in segment of a given point
                 * @param point
                 * @return the Nearest point in segment
                 */
                inline math::Vector3 nearestPointInSegment( const math::Vector3& point ) const 
                {
                    // given point is considered as a point C
                    math::Vector3 segmentAC = point - this->pointA;
                    math::Vector3 segmentBC = point - this->pointB;
                    math::Vector3 segmentAB = this->pointB - this->pointA;

                    float distanceFromSegmentPointAInSegment =
                        segmentAC.dotProduct( segmentAB ) / segmentAB.dotProduct( segmentAB );
                    math::Vector3 pointInSegment = ( segmentAB * distanceFromSegmentPointAInSegment ) + this->pointA;

                    if ( distanceFromSegmentPointAInSegment < 0 || distanceFromSegmentPointAInSegment > 1 ) {
                        // the neares point is out of the segment, so which point (A or B) is nearest to C?
                        if ( segmentAC.dotProduct( segmentAC ) < segmentBC.dotProduct( segmentBC ) ) {
                            pointInSegment = this->pointA;
                        } else {
                            pointInSegment = this->pointB;
                        } // else
                    } // else

                    return pointInSegment;
                }


                /**
                 * Compute the distance between a line segments and a given point
                 * @param point
                 * @return the distance between segments
                 */
                inline double distanceTo( const math::Vector3& point ) const 
                {
                    return ( nearestPointInSegment( point ) - point ).length( );
                }

                /**
                 * Compute the distance between two line segments
                 * @param segment2
                 * @return the distance between segments
                 */
                inline double distanceTo( const LineSegment& segment2, Vector3* pointInA = 0, Vector3* pointInB = 0 ) const 
                {
                    // reference: http://geometryalgorithms.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment()

                    Vector3   u = this->pointB - this->pointA;
                    Vector3   v = segment2.pointB - segment2.pointA;
                    Vector3   w = this->pointA - segment2.pointA;
                    double    a = u.dotProduct(u);   // always >= 0
                    double    b = u.dotProduct(v);
                    double    c = v.dotProduct(v);   // always >= 0
                    double    d = u.dotProduct(w);
                    double    e = v.dotProduct(w);
                    double    D = a * c - b * b;     // always >= 0
                    double    sc, sN, sD = D;        // sc = sN / sD, default sD = D >= 0
                    double    tc, tN, tD = D;        // tc = tN / tD, default tD = D >= 0

                    // compute the line parameters of the two closest points
                    if (D < TOLERANCE_DISTANCE ) { // the lines are almost parallel
                        sN = 0.0;        // force using point P0 on segment S1
                        sD = 1.0;        // to prevent possible division by 0.0 later
                        tN = e;
                        tD = c;
                    } else {                // get the closest points on the infinite lines
                        sN = (b * e - c * d);
                        tN = (a * e - b * d);
                        if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
                            sN = 0.0;
                            tN = e;
                            tD = c;
                        } else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
                            sN = sD;
                            tN = e + b;
                            tD = c;
                        } // else if
                    } // else

                    if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
                        tN = 0.0;
                        // recompute sc for this edge
                        if (-d < 0.0) {
                            sN = 0.0;
                        } else if (-d > a) {
                            sN = sD;
                        } else {
                            sN = -d;
                            sD = a;
                        } // else
                    } else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
                        tN = tD;
                        // recompute sc for this edge
                        if ((-d + b) < 0.0) {
                            sN = 0;
                        } else if ((-d + b) > a) {
                            sN = sD;
                        } else {
                            sN = (-d + b);
                            sD = a;
                        } // else
                    } // else if

                    // finally do the division to get sc and tc
                    sc = ( std::fabs(sN) < TOLERANCE_DISTANCE ? 0.0 : sN / sD);
                    tc = ( std::fabs(tN) < TOLERANCE_DISTANCE ? 0.0 : tN / tD);

                    // get the difference of the two closest points
                    Vector3 dP = w + ( ( u * sc) - (v * tc) );  // = S1(sc) - S2(tc)
                    if ( pointInA != NULL ) {
                        *pointInA = ( u * sc) + this->pointA;
                    } // if
                    if ( pointInB != NULL ) {
                        *pointInB = ( v * tc) + segment2.pointA;
                    } // if
                    return dP.length( );   // return the closest distance
                }

                inline bool sharePoint( const LineSegment& o ) const {
                    return ( ( pointA - o.pointA ).length( ) < TOLERANCE_DISTANCE ||
                             ( pointA - o.pointB ).length( ) < TOLERANCE_DISTANCE ||
                             ( pointB - o.pointB ).length( ) < TOLERANCE_DISTANCE );
                }

                inline bool sharePoint( const Vector3& p ) const 
                {
                    return ( ( pointA - p ).length( ) < TOLERANCE_DISTANCE ||
                             ( pointB - p ).length( ) < TOLERANCE_DISTANCE );
                }

                inline Vector3 getMidPoint( ) const 
                {
                    Vector3 direction( pointB - pointA );
                    double len = direction.length() / 2;
                    direction.normalise( );
                    return pointA + ( direction * len );
                }

                inline LineSegment& operator=( const LineSegment& segment ) 
                    {
                    this->pointA = segment.pointA;
                    this->pointB = segment.pointB;
                    return *this;
                }

                inline bool operator<( const LineSegment& segment ) const 
                {
                    return ( pointA < segment.pointA || ( !(segment.pointA < pointA ) && pointB < segment.pointB ) );
                }

            };

        } // math
    } // spatial
} // opencog

#endif // _SPATIAL_MATH_LINESEGMENT_H_
