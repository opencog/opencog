/*
 * opencog/spatial/3DBlock.cc
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

#include "Block3D.h"
#include "BlockEntity.h"

using namespace opencog;
using namespace opencog::spatial;

extern const opencog::Handle opencog::Handle::UNDEFINED;

Block3D::Block3D(int _composedLevel, BlockVector& _position, string _materialType, string _color, bool _canDestroy )
{
    mLevel = _composedLevel;
    mPosition = _position;
    mBlockMaterial.materialType = _materialType;
    mBlockMaterial.color = _color;
    mCanDestroy = _canDestroy;

    mBoundingBox.nearLeftBottomConer = mPosition;

    int edgeSize = 1;
    for (int i = 0; i < _composedLevel-1; ++i)
        edgeSize *= 2;
    mBoundingBox.size = edgeSize;
    mBoundingBox.size_x = edgeSize;
    mBoundingBox.size_y = edgeSize;
    mBoundingBox.size_z = edgeSize;

    mBlockEntity = 0;

}

Block3D::~Block3D()
{
    // remove this big block from the blocklist of myEntity
    if (mBlockEntity)
        mBlockEntity->removeBlock(this);
}

Block3D* Block3D::clone()
{
    Block3D* cloneBLock = new Block3D(mLevel,mPosition,mBlockMaterial.materialType,mBlockMaterial.color,mCanDestroy);

    return cloneBLock;
}
