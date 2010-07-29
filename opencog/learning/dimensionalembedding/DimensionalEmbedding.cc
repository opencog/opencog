/*
 * opencog/server/DimensionalEmbedding.cc
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
#include "DimensionalEmbedding.h"
#include <opencog/util/Logger.h>
#include <numeric>
#include <cmath>
#include <opencog/guile/SchemePrimitive.h>

using namespace opencog;

DimensionalEmbedding::DimensionalEmbedding() {
    numDimensions=50;
    CogServer& cogServer = static_cast<CogServer&>(server());
    this->as=cogServer.getAtomSpace();    
#ifdef HAVE_GUILE
    //Scheme functions for testing
    define_scheme_primitive("embedSpace",
                            &DimensionalEmbedding::embedAtomSpace,
                            this);
    define_scheme_primitive("logEmbedding",
                            &DimensionalEmbedding::logSimEmbedding,
                            this);
    /*
    define_scheme_primitive("euclidDist",
                            &DimensionalEmbedding::euclidDistSim,
                            this);
    */
#endif
}

void DimensionalEmbedding::run(CogServer* cogServer){
    logger().info("Running DimEmbed Agent");
}

void DimensionalEmbedding::embedSimLinks() {
    embedAtomSpace(SIMILARITY_LINK);
}
void DimensionalEmbedding::logSimEmbedding() {
    logAtomEmbedding(SIMILARITY_LINK);
}
/*
Handle DimensionalEmbedding::euclidDistSim(const Handle& h1, const Handle& h2) {
    euclidDist(h1, h2, SIMILARITY_LINK);
}
*/
    
// Uses a slightly modified version of Dijkstra's algorithm
double DimensionalEmbedding::findHighestWeightPath(const Handle& startHandle,
                                                   const Handle& targetHandle,
                                                   const Type& linkType)
{
    typedef std::map<Handle, double> NodeMap;
    NodeMap nodeMap;
    nodeMap[startHandle]=1;

    HandleSet visitedNodes;
    while(!nodeMap.empty()) {
        std::pair<Handle, double> bestNode = std::make_pair(startHandle, -1);

        //set bestNode to the node in nodeMap with the highest weight
        for(NodeMap::iterator it=nodeMap.begin(); it!=nodeMap.end(); ++it){
            if(it->second > bestNode.second) bestNode=(*it);
        }
        visitedNodes.add(bestNode.first);
        //Check whether we've reached the targetHandle
        if(bestNode.first==targetHandle) return bestNode.second;
        //Look at all of the links connected to the bestNode
        HandleSeq newLinks = as->getIncoming(bestNode.first);
        for(HandleSeq::iterator it=newLinks.begin(); it!=newLinks.end(); ++it){
            //ignore links that aren't of type linkType
            if(as->getType(*it)!=linkType) continue;
            
            HandleSeq newNodes = as->getOutgoing(*it);
            //update all of the nodes' weights.
            for(HandleSeq::iterator it2=newNodes.begin();
                it2!=newNodes.end(); ++it2){
                //if the node has been visited, don't do anything
                if(visitedNodes.contains(*it2)) continue;

                const TruthValue& linkTV = as->getTV(*it);
                //If this path is better than the currently known one, save it
                double pathWeight
                    = bestNode.second*(linkTV.getMean()*linkTV.getConfidence());
                if(nodeMap[*it2] < pathWeight) {
                    nodeMap[*it2] = pathWeight;
                }
            }
        }  
        nodeMap.erase(bestNode.first);
    }
    return 0; //no path found, return 0
}


std::vector<double> DimensionalEmbedding::getEmbedVector(const Handle& h,
                                                         const Type& l) {
    return ((atomMaps.find(l))->second).find(h)->second;
}

double DimensionalEmbedding::euclidDist(const Handle& h1,
                                        const Handle& h2,
                                        const Type& l) {
    double distance;
    std::vector<double> v1=getEmbedVector(h1,l);
    std::vector<double> v2=getEmbedVector(h2,l);
    std::vector<double>::iterator it1=v1.begin();
    std::vector<double>::iterator it2=v2.begin();
    //Calculate euclidean distance between v1 and v2
    for(; it1 < v1.end(); it1++) {
        distance+=std::pow((*it1 - *it2), 2);
    }
    distance=std::sqrt(distance);
    return distance;
}

void DimensionalEmbedding::addPivot(const Handle& h, const Type& linkType){
    pivotsMap[linkType].push_back(h);

    //update atomEmbedding for this linkType to include this pivot in its
    //embedding vector for each node.
    HandleSeq nodes;
    as->getHandleSet(std::back_inserter(nodes), NODE, true);
    for(HandleSeq::iterator it=nodes.begin(); it!=nodes.end(); ++it){
        std::vector<double>& embedVector=atomMaps[linkType][*it];
        embedVector.push_back
            (findHighestWeightPath(*it, h, linkType));
    }
}

void DimensionalEmbedding::embedAtomSpace(const Type& linkType){
    clearEmbedding(linkType);
    HandleSeq nodes;
    as->getHandleSet(std::back_inserter(nodes), NODE, true);

    PivotSeq& pivots = pivotsMap[linkType];
    if(nodes.empty()) return;
    Handle bestChoice = nodes.back();
    
    while((pivots.size() < numDimensions) && (!nodes.empty())){
        addPivot(bestChoice, linkType);
        //logger().info("Pivot %d picked", pivots.size());
        nodes.erase(std::find(nodes.begin(), nodes.end(), bestChoice));

        bestChoice = nodes[0];
        double bestChoiceWeight = 1;
        for(HandleSeq::iterator it=nodes.begin(); it!=nodes.end(); ++it){
            //calculate the nearness to pivots by multiplying the highest
            //weight paths from the node to each pivot.
            std::vector<double>& embedVector=atomMaps[linkType][*it];
            double testChoiceWeight =
                std::accumulate(embedVector.begin(),
                                embedVector.end(),
                                double(1),
                                std::multiplies<double>());
            //pick the node with the lowest testChoiceWeight
            //(farthest from the pivots)
            if(testChoiceWeight < bestChoiceWeight){
                bestChoice = *it;
                bestChoiceWeight = testChoiceWeight;
            }
        }
    }
    //logger().info("Embedding done");
}

void DimensionalEmbedding::addNode(const Handle& h, const Type& linkType){
    PivotSeq& pivots=pivotsMap[linkType];
    std::vector<double> embeddingVector;
    //The i'th entry of the handle's embeddingVector is the value of the
    //highest weight path between the handle and the i'th pivot.
    for(PivotSeq::iterator it=pivots.begin(); it!=pivots.end(); ++it){
        embeddingVector.push_back(findHighestWeightPath(h, *it, linkType));
    }
    atomMaps[linkType][h]=embeddingVector;
}

void DimensionalEmbedding::clearEmbedding(const Type& linkType){
    atomMaps.erase(linkType);
    pivotsMap.erase(linkType);
}

void DimensionalEmbedding::logAtomEmbedding(const Type& linkType){
    AtomEmbedding atomEmbedding=atomMaps[linkType];
    PivotSeq pivots = pivotsMap[linkType];

    std::ostringstream oss;
    
    oss << "PIVOTS:" << std::endl;
    for(PivotSeq::const_iterator it=pivots.begin(); it!=pivots.end(); ++it){
        Atom* atom = TLB::getAtom(*it);
        oss << atom->toShortString() << std::endl;
    }
    oss << "Node Embeddings:" << std::endl;
    AtomEmbedding::const_iterator it;
    for(it=atomEmbedding.begin(); it!=atomEmbedding.end(); ++it){
        Atom* atom = TLB::getAtom(it->first);
        oss << atom->toShortString() << " : (";
        std::vector<double> embedVector = it->second;
        for(std::vector<double>::const_iterator it2=embedVector.begin();
            it2!=embedVector.end();
            ++it2){
            oss << *it2 << " ";
        }
        oss << ")" << std::endl;
    }
    logger().info(oss.str());
    return;
}
