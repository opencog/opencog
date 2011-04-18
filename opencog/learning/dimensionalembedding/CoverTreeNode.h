/*
 * opencog/learning/dimensionalembedding/CoverTreeNode.h
 *
 * Copyright (C) 2010 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by David Crane <dncrane@gmail.com>
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

#ifndef _OPENCOG_COVER_TREE_NODE_H
#define _OPENCOG_COVER_TREE_NODE_H

#include <vector>
#include <string>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/learning/dimensionalembedding/DimEmbedModule.h>
#include <opencog/util/cover_tree.h>

/**
 * The CoverTreeNode class and its methods are required by the cover tree
 * implementation
 */
namespace opencog {

class CoverTreeNode {
public:
    std::map<Handle, std::vector<double> >::const_iterator iter;
    CoverTreeNode() {}
    CoverTreeNode(std::map<Handle, std::vector<double> >::const_iterator it) : iter(it) {}
    const std::vector<double>& getVector() {
        return iter->second;
    }
    const Handle& getHandle() {
        return iter->first;
    }
};

void print(AtomSpace& atomspace, CoverTreeNode& p) {
    std::ostringstream oss;
    Handle h = p.getHandle();
    if(!atomspace.isValidHandle(h)) {
       oss << "[NODE'S BEEN DELETED]" << " : (";
    } else {
        oss << atomspace.atomAsString(h, true) << " : (";
    }
    const std::vector<double>& embedVector = p.getVector();
    for(std::vector<double>::const_iterator it=embedVector.begin();
        it!=embedVector.end(); it++)
        {
            oss << *it << " ";
        }       
    oss << ")" << std::endl;
    printf("%s", oss.str().c_str());
}

double distance(CoverTreeNode n1, CoverTreeNode n2, double upper_bound) {
    const std::vector<double>& v1=n1.getVector();
    const std::vector<double>& v2=n2.getVector();
    OC_ASSERT(v1.size()==v2.size());
    std::vector<double>::const_iterator it1=v1.begin();
    std::vector<double>::const_iterator it2=v2.begin();
    
    double distance=0;
    //Calculate euclidean distance between v1 and v2
    for(; it1!=v1.end(); it1++, it2++)
        distance+=sq(*it1 - *it2);
    distance=sqrt(distance);
    return distance;
}

}//namespace

#endif //_OPENCOG_DIM_EMBED_MODULE_H
