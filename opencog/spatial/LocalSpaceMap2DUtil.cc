/*
 * opencog/spatial/LocalSpaceMap2DUtil.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Dan Zwell, Samir Araujo
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
#include <limits>

#include <opencog/util/functional.h>
#include <opencog/util/Logger.h>
#include <opencog/util/random.h>

#include <opencog/spatial/LocalSpaceMap2DUtil.h>
#include <opencog/spatial/LocalSpaceMap2D.h>
#include <opencog/spatial/math/Vector3.h>
#include <opencog/spatial/math/LineSegment.h>


using namespace opencog;
using namespace opencog::spatial;

GridPointCollector::GridPointCollector( std::vector<spatial::GridPoint>& gridPoints ) :
        gridPoints( gridPoints )
{
}

bool GridPointCollector::operator()( const spatial::GridPoint& gridPoint )
{
    gridPoints.push_back( gridPoint );
    return true;
}

CollisionDetector::CollisionDetector( spatial::LocalSpaceMap2D* map, spatial::GridPoint& collisionPoint, bool& collided ) :
        map( map ), collisionPoint( collisionPoint ), collided( collided )
{
    collisionPoint = spatial::GridPoint( 0, 0 );
};

bool CollisionDetector::operator()( const spatial::GridPoint& gridPoint )
{
    collided = map->gridIllegal( gridPoint );

    if ( collided ) {
        // set the start point as a collision point when rayTrace starts from a invalid position
        if ( collisionPoint == spatial::GridPoint( 0, 0 ) ) {
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

ObjectMetaData::ObjectMetaData(double cx, double cy, double cz, double l, double w, double h, double y) :
        centerX(cx), centerY(cy), centerZ(cz), length(l), width(w), height(h),  yaw(y)
{
    entityClass = "";
}

ObjectMetaData::ObjectMetaData(double cx, double cy, double cz, double l, double w, double h, double y, const std::string& ec) :
        centerX(cx), centerY(cy), centerZ(cz), length(l), width(w), height(h),  yaw(y), entityClass(ec)
{
}

bool ObjectMetaData::operator==(const ObjectMetaData& rhs) const
{
    return (centerX == rhs.centerX &&
            centerY == rhs.centerY &&
            centerZ == rhs.centerZ &&
            length == rhs.length &&
            width == rhs.width &&
            height == rhs.height &&
            yaw == rhs.yaw &&
            entityClass == rhs.entityClass);
}

bool ObjectMetaData::operator!=(const ObjectMetaData& rhs) const
{
    return (!(*this == rhs));
}

/**
 * DEPRECATED METHOD - it must be removed as like TangentBugTestExec and AStarTest
 */
void opencog::spatial::populateRandom(spatial::LocalSpaceMap2D& lsm,
                                      int obstacles,
                                      const spatial::GridPoint& prob_center,
                                      int std_dev)
{

    opencog::RandGen& rng = randGen();
    for (int cnt = 0; cnt < obstacles; ++cnt) {
        unsigned
            center_x = gaussian_rand<unsigned>(prob_center.first, std_dev, rng),
            center_y = gaussian_rand<unsigned>(prob_center.second, std_dev, rng),
            center_z = gaussian_rand<unsigned>(prob_center.second, std_dev, rng),
            radius_y = 1 + static_cast<unsigned>(30 * rng.randdouble_one_excluded()),
            radius_x = 1 + static_cast<unsigned>(20 * rng.randdouble_one_excluded());

        spatial::ObjectMetaData metaData;
        metaData.centerX = center_x;
        metaData.centerY = center_y;
        metaData.centerZ = center_z;
        metaData.width = radius_x;
        metaData.height = 1;
        metaData.length = radius_y;
        metaData.yaw = 0;
        metaData.entityClass = "";
        lsm.addObject( std::to_string(cnt), metaData, true );

    } // for
}
