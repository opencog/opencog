/*
 *
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
 * All rights reserved.
 * License: AGPL
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TEMPLATE_OCTREE_H
#define TEMPLATE_OCTREE_H

#include <iostream>
#include "AtomOcTreeNode.h"
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap
{
// tree definition
class AtomOcTree : public OccupancyOcTreeBase < AtomOcTreeNode >
{

public:

    /// Default constructor, sets resolution of leafs
    AtomOcTree(double resolution = 0.1); //does not work without optional resolution

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    AtomOcTree *create() const
    {
        return new AtomOcTree(resolution);    //changed to this->resolution else templates cause problems
    }

    std::string getTreeType() const
    {
        return "AtomOcTree";
    }

    // set node dat at given key or coordinate. Replaces previous dat.
    AtomOcTreeNode* setNodeData(const OcTreeKey& key, const opencog::Handle& r);

    AtomOcTreeNode* setNodeData(const point3d& xyz, const opencog::Handle& r)
    {
        OcTreeKey key;
        if (!this->coordToKeyChecked(xyz, key)) return nullptr;
        return setNodeData(key, r);
    }
    // update inner nodes, sets dat to average child dat
    void updateInnerOccupancy() {}


protected:
    ////void updateInnerOccupancyRecurs(AtomOcTreeNode<T>* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */

    class StaticMemberInitializer
    {
    public:
        StaticMemberInitializer()
        {
            AtomOcTree* tree = new AtomOcTree(0.1);
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

//! user friendly output in format (r g b)
//std::ostream& operator<<(std::ostream& out, AtomOcTreeNode::T const& c);
////std::ostream& operator<<(std::ostream& out, opencog::Handle const& c);

} // end namespace

#endif
