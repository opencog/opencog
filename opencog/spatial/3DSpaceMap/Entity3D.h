/*
 * opencog/spatial/BlockEntity.h
 *
 * Copyright (C) 2002-2011 OpenCog Foundation
 * All Rights Reserved
 * Author(s): Shujing Ke
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


#ifndef _SPATIAL_ENTITY3D_H
#define _SPATIAL_ENTITY3D_H

#include "Block3DMapUtil.h"
#include "Block3D.h"
#include "Octree3DMapManager.h"
#include <opencog/spatial/math/Quaternion.h>
#include <vector>
#include <opencog/atomspace/Handle.h>

namespace opencog
{
    namespace spatial
    {

        class Entity3D
        {

        public:
            Entity3D(BlockVector _centerPosition, int _width, int _lenght, int _height,double yaw, std::string _entityName, std::string _entityClass, bool _is_obstacle);
            ~Entity3D();

            Entity3D* clone();

            inline int getEntityID() const {return mID;}
            inline std::string getEntityName() const { return mName;}
            inline std::string getEntityClass() const { return mEntityClass;}
            inline bool getIsObstacle() const { return is_obstacle;}
            inline BlockVector& getLeftBottomPosition() {return mBoundingBox.nearLeftBottomConer;}

            // the standing location
            BlockVector getPosition() const {return BlockVector(mCenterPosition.x, mCenterPosition.y, mBoundingBox.nearLeftBottomConer.z);}

            inline const BlockVector& getCenterPosition() const {return mCenterPosition;}

            inline int getWidth() const {return mBoundingBox.size_x;}
            inline int getLength() const {return mBoundingBox.size_y;}
            inline int getHeight() const {return mBoundingBox.size_z;}
            inline double getRadius() const {return  sqrt(mBoundingBox.size_x * mBoundingBox.size_x +  mBoundingBox.size_y * mBoundingBox.size_y)/2.0 ; }

            inline const AxisAlignedBox& getBoundingBox() const {return mBoundingBox;}

            void assignEntityName(std::string entityName) {mName = entityName;}// assign a name for this entity

            Handle mEntityNode; // the node represents this entity in the atomspace

            inline bool isBlockEntity()const {return false;}
            inline double getYaw() const {return mYaw;}

            // Get direction this entity face to.
            // For the sake of simplification, we only have 8 kinds of direction:
            // (x = 1, y = 0):    -pai/8 <= a < pai/8
            // (x = 1, y = 1):     pai/8 <= a < pai*3/8
            // (x = 0, y = 1):   pai*3/8 <= a < pai*5/8
            // (x = -1, y = 1):  pai*5/8 <= a < pai*7/8
            // (x = -1, y = 0):  pai*7/8 <= a < -pai*7/8
            // (x = -1, y = -1):-pai*7/8 <= a < -pai*5/8
            // (x = 0, y = -1): -pai*5/8 <= a < -pai*3/8
            // (x = 1, y = -1): -pai*5/8 <= a < -pai/8
            BlockVector getDirection() const;

            // when a nonblock entity change its position or direction or other space attributes, call this function
            // Note: only apply to non-block entities - pls do not apply in a blockEntity
            void updateNonBlockEntitySpaceInfo(BlockVector _centerPosition, int _width, int _lenght, int _height,double yaw, bool _is_obstacle);

            void updateNonBlockEntityLocation(BlockVector _centerPosition);

        protected:
            Entity3D(){};
            int mID;
            std::string mName;
            std::string mEntityClass;
            bool is_obstacle;
            AxisAlignedBox mBoundingBox;
            BlockVector mCenterPosition;
            double mYaw;
            math::Quaternion mOrientation;
        };
    }
}


#endif // _SPATIAL_ENTITY3D_H
