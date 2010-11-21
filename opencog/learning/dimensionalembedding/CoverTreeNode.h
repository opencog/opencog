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

#include <list>
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
        std::map<Handle, std::list<double> >::const_iterator iter;
        CoverTreeNode() {}
        CoverTreeNode(std::map<Handle, std::list<double> >::const_iterator it) : iter(it) {} 
        const std::list<double>& getList() {
            return iter->second;
        }
        const Handle& getHandle() {
            return iter->first;
        }
    };
    
    void print(CoverTreeNode& p) {
        std::ostringstream oss;
        Atom* atom = TLB::getAtom(p.getHandle());
        if(atom==NULL) {
           oss << "[NODE'S BEEN DELETED]" << " : (";
        } else {
            oss << atom->toShortString() << " : (";
        }
        const std::list<double>& embedList = p.getList();
        for(std::list<double>::const_iterator it=embedList.begin();
            it!=embedList.end(); it++)
            {
                oss << *it << " ";
            }       
        oss << ")" << std::endl;
        printf("%s", oss.str().c_str());
    }
    
    double distance(CoverTreeNode n1, CoverTreeNode n2, double upper_bound) {
        const std::list<double>& v1=n1.getList();
        const std::list<double>& v2=n2.getList();
        std::list<double>::const_iterator it1=v1.begin();
        std::list<double>::const_iterator it2=v2.begin();
        
        double distance=0;
        //Calculate euclidean distance between v1 and v2
        for(; it1!=v1.end(); it1++) {
            distance+=(*it1 - *it2)*(*it1 - *it2);
            if(it2!=v2.end()) it2++;
        }
        distance=sqrt(distance);
        return distance;
    }
}//namespace

#endif //_OPENCOG_DIM_EMBED_MODULE_H
