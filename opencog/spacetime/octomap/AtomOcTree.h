/*
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
 *
 * All rights reserved.
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

#ifndef TEMPLATE_OCTREE_H
#define TEMPLATE_OCTREE_H

#include <bitset>
#include <iostream>
#include "AtomOcTreeNode.h"
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap
{
// tree definition

template <typename T>
class AtomOcTree : public OccupancyOcTreeBase < AtomOcTreeNode<T> >
{

public:
    /// Default constructor, sets resolution of leafs
    AtomOcTree(double resolution = 0.1)
        : OccupancyOcTreeBase< AtomOcTreeNode<T> >(resolution)
    {}


    /// virtual constructor: creates a new object of same type
    AtomOcTree<T> *create() const
    {
        return new AtomOcTree<T>(this->resolution);
    }

    std::string getTreeType() const
    {
        return "AtomOcTree";
    }

    // set node dat at given key or coordinate. Replaces previous dat.
    AtomOcTreeNode<T>* setNodeData(const OcTreeKey& key, const T& r){
        AtomOcTreeNode<T>* n = this->search(key);
        if (n != 0) {
            n->setData(r);
        }
        return n;
    }

    AtomOcTreeNode<T>* setNodeData(const point3d& xyz, const T& r)
    {
        OcTreeKey key;
        if (!this->coordToKeyChecked(xyz, key)) return nullptr;
        return setNodeData(key, r);
    }

    inline std::string getMapName() const {return mMapName;}

    std::string mMapName;

protected:

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once.
     */

    struct StaticMemberInitializer
    {
        StaticMemberInitializer()
        {
            AtomOcTree<T>* tree = new AtomOcTree<T>(0.1);
            AbstractOcTree::registerTreeType(tree);
        }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer atomOcTreeMemberInit;

};

template <typename T>
typename AtomOcTree<T>::StaticMemberInitializer AtomOcTree<T>::atomOcTreeMemberInit;

} // end namespace

#endif
