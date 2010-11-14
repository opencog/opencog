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
        Handle handle;
        //std::vector<double> embeddingVector;
        double embeddingVector[50];
        CoverTreeNode() {}
    CoverTreeNode(Handle h, std::vector<double> eV) :
        handle(h) {
            int i=0;
            std::vector<double>::iterator it;
            for(it=eV.begin(); it!=eV.end(); it++) {
                embeddingVector[i]=*it;
                i++;
            }
        }
        /*
        float distance(CoverTreeNode n1, CoverTreeNode n2, float upper_bound) {
            std::vector<double> v1=n1.getVector();
            std::vector<double> v2=n2.getVector();
            std::vector<double>::iterator it1=v1.begin();
            std::vector<double>::iterator it2=v2.begin();
            
            double distance=0;
            //Calculate euclidean distance between v1 and v2
            for(; it1 < v1.end(); it1++) {
                distance+=std::pow((*it1 - *it2), 2);
                if(it2!=v2.end()) it2++;
            }
            distance=sqrt(distance);
            return distance;
            //return DimEmbedModule::euclidDistance(n1.getVector(),
            //                                      n2.getVector());
        }
        v_array<CoverTreeNode> parse_points(char *filename) {
            //@TODO: implement
            return v_array<CoverTreeNode>();
        }
        void print(CoverTreeNode& p) {
            std::ostringstream oss;
            Atom* atom = TLB::getAtom(p.getHandle());
            if(atom==NULL) {
                oss << "[NODE'S BEEN DELETED]" << " : (";
            } else {
                oss << atom->toShortString() << " : (";
            }
            std::vector<double> embedVector = p.getVector();
            for(std::vector<double>::const_iterator it2=embedVector.begin();
                it2!=embedVector.end();
                ++it2){
                oss << *it2 << " ";
            }
            oss << ")" << std::endl;
            logger().info(oss.str());
            }*/
        /*        
        std::vector<double> getVector() {
            return embeddingVector;
        }
        Handle& getHandle() {
            return handle;
        }
    };
    
        float distance(CoverTreeNode n1, CoverTreeNode n2, float upper_bound) {
            std::vector<double> v1=n1.getVector();
            std::vector<double> v2=n2.getVector();
            std::vector<double>::iterator it1=v1.begin();
            std::vector<double>::iterator it2=v2.begin();
            
            double distance=0;
            //Calculate euclidean distance between v1 and v2
            for(; it1 < v1.end(); it1++) {
                distance+=std::pow((*it1 - *it2), 2);
                if(it2!=v2.end()) it2++;
                }
            distance=sqrt(distance);
            return distance;
            //return DimEmbedModule::euclidDistance(n1.getVector(),
            //                                      n2.getVector());
            }*/
    };
    float distance(CoverTreeNode n1, CoverTreeNode n2, float upper_bound) {
        double distance=0;
        for(int i=0; i<50; i++){
            distance+=std::pow(n1.embeddingVector[i]-n2.embeddingVector[i], 2);
        }
        distance=sqrt(distance);
        return distance;
    }
    v_array<CoverTreeNode> parse_points(char *filename) {
        //@TODO: implement
        return v_array<CoverTreeNode>();
    }
    void print(CoverTreeNode& p) {
        std::ostringstream oss;
        Atom* atom = TLB::getAtom(p.handle);
        if(atom==NULL) {
            oss << "[NODE'S BEEN DELETED]" << " : (";
        } else {
            oss << atom->toShortString() << " : (";
        }
        /*
          std::vector<double> embedVector = p.getVector();
          for(std::vector<double>::const_iterator it2=embedVector.begin();
          it2!=embedVector.end();
          ++it2){
          oss << *it2 << " ";
          }*/
        oss << ")" << std::endl;
        logger().info(oss.str());
    }
}//namespace

#endif //_OPENCOG_DIM_EMBED_MODULE_H
