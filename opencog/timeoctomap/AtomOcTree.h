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
#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>

using namespace opencog::spatial;

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

    void addSolidUnitBlock(const T& block, BlockVector pos){
        this->setUnitBlock(block, pos, this->getOccupancyThresLog()); 
    }

    //Note that if you want to add/remove block with probability,
    //You should use setUnitBlock to control the occupancy probability.
    //the updateLogOddsOccupancy will be added on the log odds occupancy of block to in/decrease the occupancy
    //probabilistic set occupancy, e.g. new occupied log
    void setUnitBlock(const T& block, BlockVector pos, float updateLogOddsOccupancy){
        this->updateNode(pos.x, pos.y, pos.z, float(updateLogOddsOccupancy));
        this->setNodeBlock( point3d(pos.x, pos.y, pos.z), block);
    }

    AtomOcTreeNode<T>* setNodeBlock(const double& x,
            const double& y,
            const double& z,
            const T& block)
    {
        point3d pos(x, y, z);
        return setNodeBlock(pos, block);
    }


    AtomOcTreeNode<T>* setNodeBlock(const point3d& pos, const T& block)
    {
        AtomOcTreeNode<T>* n = static_cast<AtomOcTreeNode<T>*>(this->search(pos));
        if (n != NULL) {
            // add/remove record in atom->position map
            T oldBlock = n->getData();
            if (oldBlock == T::UNDEFINED && block != T::UNDEFINED) {
                mTotalUnitBlockNum++;
                mAllUnitAtomsToBlocksMap.insert(pair<T, BlockVector>(block, BlockVector(pos.x(), pos.y(), pos.z())));
            } else if (oldBlock != T::UNDEFINED && block == T::UNDEFINED) {
                mTotalUnitBlockNum--;
                mAllUnitAtomsToBlocksMap.erase(oldBlock);
            }

            n->setData(block);
        }

        return n;
    }


    // binary
    BlockVector getBlockLocation(const T& block) const{
        return getBlockLocation(block, this->getOccupancyThresLog());
    }
    // probabilistic
    BlockVector getBlockLocation(const T& block, float logOddsOccupancyThreshold) const{
        auto it = mAllUnitAtomsToBlocksMap.find(block);
        if(it == mAllUnitAtomsToBlocksMap.end()) {
            return BlockVector::ZERO;
        } else {
            BlockVector result=it->second;
            if( this->search(result.x, result.y, result.z)->getLogOdds() < logOddsOccupancyThreshold) {
                return BlockVector::ZERO;
            } else {
                return result;
            }
        }
    }

    //  check if the block is out of octree's max size
    bool checkIsOutOfRange(const BlockVector& pos) const{
        OcTreeKey key;
        return !this->coordToKeyChecked(point3d(pos.x, pos.y, pos.z), key);
    }
    //  use occ_prob_thres_log(see octomap doc) as threshold
    T getBlock(const BlockVector& pos) const{
        return getBlock(pos, this->occ_prob_thres_log);
    }
    //  get block in pos. If occupancy(log odds) larger than threshold
    //  It will return the block (including undefined handle) in pos;
    //  If smaller than threshold, it'll return Handle::UNDEFINED
    //  default threshold is the prob_hit_log which is the default
    //  octomap log odds threshold.
    T getBlock(const BlockVector& pos, const float logOddsOccupancyThreshold) const{
        AtomOcTreeNode<T>* blocknode = static_cast<AtomOcTreeNode<T>*>(this->search(pos.x, pos.y, pos.z));
        if (blocknode != nullptr){
            if(blocknode->getLogOdds() > logOddsOccupancyThreshold) {
                std::cout << "UNDEFINED HANDLE " << " " << blocknode->getLogOdds() << "  " << logOddsOccupancyThreshold << std::endl;
                return T::UNDEFINED;
            }
            else {
                return blocknode->getData();
            }
        }
        else{
            return T::UNDEFINED;
        }
    }
    //  use occ_prob_thres_log(see octomap doc) as threshold
    bool checkBlockInPos(const T& block, const BlockVector& pos) const{
        return checkBlockInPos(block, pos, this->occ_prob_thres_log);
    }
    //  check the block is in the position,
    //  Noth that even there's a block in that pos,
    //  if the handle is not equal it still return false.
    bool checkBlockInPos(const T& block, const BlockVector& pos, const float logOddsOccupancyThreshold) const{
        AtomOcTreeNode<T>* blocknode = this->search(pos.x, pos.y, pos.z);
        if (blocknode == NULL ||
                blocknode->getLogOdds() < logOddsOccupancyThreshold ||
                blocknode->getBlock() != block) {
            return false;
        } else {
            return true;
        }
    }

    inline float getAgentHeight() const {return mAgentHeight;}
    void setAgentHeight(float _height){ mAgentHeight = _height;}
    inline string getMapName() const {return mMapName;}

    std::string mMapName;
    float mAgentHeight;
    int mTotalUnitBlockNum;

    // We keep the map for quick search position.
    // Memory consuming: 50k blocks take about 10M RAM for one map
    // Time consuming: 2e-5 sec for 10k blocks; if using bindlink to get position cost 2e-3 sec
    map<T, BlockVector> mAllUnitAtomsToBlocksMap;

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
