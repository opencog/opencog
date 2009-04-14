/*
 * opencog/spatial/LocalSpaceMap2DUtil.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
 * All Rights Reserved
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
#include "LocalSpaceMap2DUtil.h"
#include "LocalSpaceMap2D.h"
#include <opencog/util/functional.h>
#include <opencog/util/Logger.h>
#include <opencog/util/StringManipulator.h>

#include "Math/Vector3.h"
#include "Math/LineSegment.h"

#include <limits>

namespace Spatial
{


GridPointCollector::GridPointCollector( std::vector<Spatial::GridPoint>& gridPoints ) :
        gridPoints( gridPoints )
{
}

bool GridPointCollector::operator()( const Spatial::GridPoint& gridPoint )
{
    gridPoints.push_back( gridPoint );
    return true;
}

CollisionDetector::CollisionDetector( Spatial::LocalSpaceMap2D* map, Spatial::GridPoint& collisionPoint, bool& collided ) :
        map( map ), collisionPoint( collisionPoint ), collided( collided )
{
    collisionPoint = Spatial::GridPoint( 0, 0 );
};

bool CollisionDetector::operator()( const Spatial::GridPoint& gridPoint )
{
    collided = map->gridIllegal( gridPoint );

    if ( collided ) {
        // set the start point as a collision point when rayTrace starts from a invalid position
        if ( collisionPoint == Spatial::GridPoint( 0, 0 ) ) {
            collisionPoint = gridPoint;
        } // if
        return false;
    } // if

    collisionPoint = gridPoint;
    return true;
}

/**
 * ---------------------------------------------------------------------------
 * Struct ObjMetadata
 * -----------------------------------------------------------------------------
 */
ObjectMetaData::ObjectMetaData() {}

ObjectMetaData::ObjectMetaData(double cx, double cy, double l, double w, double h, double y) :
        centerX(cx), centerY(cy), length(l), width(w), height(h),  yaw(y)
{
}

bool ObjectMetaData::operator==(const ObjectMetaData& rhs) const
{
    return (centerX == rhs.centerX &&
            centerY == rhs.centerY &&
            length == rhs.length &&
            width == rhs.width &&
            height == rhs.height &&
            yaw == rhs.yaw);
}

bool ObjectMetaData::operator!=(const ObjectMetaData& rhs) const
{
    return (!(*this == rhs));
}

/**
 * DEPRECATED METHOD - it must be removed as like TangentBugTestExec and AStarTest
 */
void populateRandom(opencog::RandGen& rng, Spatial::LocalSpaceMap2D& lsm,
                    int obstacles, const Spatial::GridPoint& prob_center, int std_dev)
{

    for (int cnt = 0; cnt < obstacles; ++cnt) {
        unsigned int center_x = rng.pos_gaussian_rand(std_dev, prob_center.first);
        unsigned int center_y = rng.pos_gaussian_rand(std_dev, prob_center.second);
        unsigned int radius_y = 1 + static_cast<unsigned int>
                                (30 * rng.randDoubleOneExcluded());
        unsigned int radius_x = 1 + static_cast<unsigned int>
                                (20 * rng.randDoubleOneExcluded());

        Spatial::ObjectMetaData metaData;
        metaData.centerX = center_x;
        metaData.centerY = center_y;
        metaData.width = radius_x;
        metaData.height = 1;
        metaData.length = radius_y;
        metaData.yaw = 0;
        lsm.addObject( opencog::toString(cnt), metaData, true );

    } // for
}


}; // namespace Spatial
