/*
 * opencog/learning/PatternMiner/HTree.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com> Scott Jones <troy.scott.j@gmail.com>
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

#ifndef _OPENCOG_PATTERNMINER_HTREE_H
#define _OPENCOG_PATTERNMINER_HTREE_H
#include <map>
#include <vector>
#include "Pattern.h"
#include <opencog/atomspace/AtomSpace.h>

using namespace std;

namespace opencog
{
     namespace PatternMining
    {

     // HTree is an efficient data strusture to store all the patterns , for quick index / query

     class HTreeNode
     {
     public:
        Skeleton* skeleton;
        map<Pattern*, HandleSeqSeq> PatternsToInstances;
        vector<HTreeNode*> parentLinks;
        vector<HTreeNode*> childLinks;

        HTreeNode(Skeleton* _skeleton):skeleton(_skeleton)
        {
            parentLinks.clear();
            childLinks.clear();
        }

     };

     class HTree
     {

     public:
         unsigned int var_num;
         HTree()
         {
             rootNode = new HTreeNode(0); // the rootNode with an empty Skeleton
             var_num = 0;
         }

         // search in the HTree to find the corresponding pattern(HTreeNode) for an input instance
         HTreeNode* findPatternInHTreeForAnInstance(vector<Handle> inputLinks);


     private:
         HTreeNode* rootNode;


     };


    }
}

#endif //_OPENCOG_PATTERNMINER_HTREE_H
