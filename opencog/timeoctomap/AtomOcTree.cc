/*
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
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
