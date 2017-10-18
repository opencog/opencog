/*
 * opencog/learning/PatternMiner/HTree.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com>
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

#include <opencog/atoms/base/Node.h>
#include <opencog/atomspace/AtomSpace.h>

using namespace std;

namespace opencog { namespace PatternMining {

class  HTreeNode;

struct ExtendRelation // to store a super pattern of a pattern
{
    HTreeNode* extendedHTreeNode; // super pattern HTreeNode
    Handle sharedLink; // link in the original pattern that connect to new extended Link
    Handle newExtendedLink; // in super pattern (contains variables, not the instance link), without unifying
    Handle extendedNode; // node that being extended in the original AtomSpace (the value node, not its variable name node)
    bool isExtendedFromVar;
};

struct SuperRelation_b // to store a super pattern of the same gram of a pattern
{
    HTreeNode* superHTreeNode; // the super pattern HTreeNode
    Handle constNode; // the const Node
};

struct SubRelation_b // to store a sub pattern of the same gram of a pattern
{
    HTreeNode* subHTreeNode; // the sub pattern HTreeNode
    Handle constNode; // the const Node
};

class HTreeNode
{
public:
    HandleSeq pattern;
    Handle quotedPatternLink; // only used when if_quote_output_pattern = true
    HandleSeqSeq instances; // the corresponding instances of this pattern in the original AtomSpace, only be used by breadth first mining
    set<HTreeNode*> parentLinks;
    set<HTreeNode*> childLinks;

    // set<string> instancesUidStrings;// all uid in each instance HandleSeq in all instances, in the form of 5152_815_201584. to prevent the same instance being count multiple times

    vector<ExtendRelation> superPatternRelations; // store all the connections to its super patterns

    vector<SuperRelation_b> superRelation_b_list; // store all the superRelation_b

    map<Handle, vector<SubRelation_b>> SubRelation_b_map;// map<VariableNode, vector<SubRelation_b>>

    unsigned int count; // instance number
    unsigned int var_num; // the number of all the variables in this pattern
    double interactionInformation;
    double nI_Surprisingness;
    double nII_Surprisingness;
    double nII_Surprisingness_b; // calculated from the other same gram patterns
    unsigned int max_b_subpattern_num;

    string surprisingnessInfo; // the middle info record the surpringness calculating process for this pattern

    HandleSeq sharedVarNodeList; // all the shared nodes in these links in the original AtomSpace, each handle is a shared node

    HTreeNode()
        : count(0),
          var_num(0),
          interactionInformation(0.0),
          nI_Surprisingness(0.0),
          nII_Surprisingness(0.0),
          nII_Surprisingness_b(1.0),
          max_b_subpattern_num(0) {}
};

class HTree
{
public:
    HTreeNode* rootNode;

    HTree()
    {
        rootNode = new HTreeNode(); // the rootNode with no parents
    }
};


}}

#endif //_OPENCOG_PATTERNMINER_HTREE_H
