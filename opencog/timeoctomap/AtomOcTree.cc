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

#include "AtomOcTree.h"
namespace octomap
{
// tree implementation  --------------------------------------
AtomOcTree::AtomOcTree(double resolution)
    : OccupancyOcTreeBase< AtomOcTreeNode >(resolution)
{
    atomOcTreeMemberInit.ensureLinking();
}

AtomOcTreeNode*
AtomOcTree::setNodeData(const OcTreeKey& key, const opencog::Handle& r)
{
    AtomOcTreeNode *n = search(key);
    if (n != 0) {
        n->setData(r);//setColor
    }
    return n;
}
/* This may not be required ..
  template <class T>
  void AtomOcTree<T>::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  template <class T>
  void AtomOcTree<T>::updateInnerOccupancyRecurs(AtomOcTreeNode<T>* node, unsigned int depth) {
    // only recurse and update for inner nodes:
    if (node->hasChildren()){
      // return early for last level:
      if (depth < this->tree_depth){
        for (unsigned int i=0; i<8; i++) {
          if (node->childExists(i)) {
            updateInnerOccupancyRecurs(node->getChild(i), depth+1);
          }
        }
      }
      node->updateOccupancyChildren();
      //node->updateTemplateChildren();//
    }
  }

  std::ostream& operator<<(std::ostream& out, opencog::Handle const& c) {
    return out << '(' << c << ')';
  }
*/

//typedef int opencog::Handle;
AtomOcTree::StaticMemberInitializer AtomOcTree::atomOcTreeMemberInit;

} // end namespace
