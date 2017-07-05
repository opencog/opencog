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
    {
        atomOcTreeMemberInit.ensureLinking();
    }


    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    AtomOcTree<T> *create() const
    {
        return new AtomOcTree<T>(this->resolution);    //changed to this->resolution else templates cause problems
    }

    std::string getTreeType() const
    {
        return "AtomOcTree";
    }

    // set node dat at given key or coordinate. Replaces previous dat.
    AtomOcTreeNode<T>* setNodeData(const OcTreeKey& key, const T& r){
        AtomOcTreeNode<T>* n = this->search(key);
        //AtomOcTreeNode<T>* n = dynamic_cast<AtomOcTreeNode<T>*>(this->search(key));
        if (n != 0) {
            n->setData(r);//setColor
        }
        return n;
    }

    AtomOcTreeNode<T>* setNodeData(const point3d& xyz, const T& r)
    {
        OcTreeKey key;
        if (!this->coordToKeyChecked(xyz, key)) return nullptr;
        return setNodeData(key, r);
    }
    // update inner nodes, sets dat to average child dat
    void updateInnerOccupancy() {}


protected:

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */

    struct StaticMemberInitializer
    {
        StaticMemberInitializer()
        {
            AtomOcTree<T>* tree = new AtomOcTree<T>(0.1);
            AbstractOcTree::registerTreeType(tree);
        }

        /**
         * Dummy function to ensure that MSVC does not drop the
         * StaticMemberInitializer, causing this tree failing to register.
         * Needs to be called from the constructor of this octree.
         */
        void ensureLinking() {};
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer atomOcTreeMemberInit;

};

template <typename T>
typename AtomOcTree<T>::StaticMemberInitializer AtomOcTree<T>::atomOcTreeMemberInit;

} // end namespace

#endif
