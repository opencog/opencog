
/*
 * opencog/learning/dimensionalembedding/DimEmbedModule.cc
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

#include "DimEmbedModule.h"
#include <algorithm>
#include <opencog/util/Logger.h>
#include <numeric>
#include <cmath>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/util/exceptions.h>
#include <limits>
#include <opencog/learning/dimensionalembedding/vector.h>

using namespace opencog;

DECLARE_MODULE(DimEmbedModule)

DimEmbedModule::DimEmbedModule() {
	logger().info("[DimEmbedModule] constructor");
}

DimEmbedModule::~DimEmbedModule() {
	logger().info("[DimEmbedModule] destructor");
}

void DimEmbedModule::init() {    
    logger().info("[DimEmbedModule] init");
    CogServer& cogServer = static_cast<CogServer&>(server());
    this->as=cogServer.getAtomSpace();    
#ifdef HAVE_GUILE
    //Functions available to scheme shell
    define_scheme_primitive("embedSpace",
                            &DimEmbedModule::embedAtomSpace,
                            this);
    define_scheme_primitive("logEmbedding",
                            &DimEmbedModule::logAtomEmbedding,
                            this);
    define_scheme_primitive("euclidDist",
                            &DimEmbedModule::euclidDist,
                            this);
    //not quite working yet. segfaults and stuff for some reason
    //define_scheme_primitive("kNN",
    //                        &DimEmbedModule::kNearestNeighbors,
    //                        this);
#endif
}

// Uses a slightly modified version of Dijkstra's algorithm
double DimEmbedModule::findHighestWeightPath(const Handle& startHandle,
                                                   const Handle& targetHandle,
                                                   const Type& linkType)
{
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
                                                                    
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

std::vector<double> DimEmbedModule::getEmbedVector(const Handle& h,
                                                         const Type& l) {
    if(!classserver().isLink(l))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(l).c_str());
 
    if(!isEmbedded(l)) {
        const char* tName = classserver().getTypeName(l).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    AtomEmbedding aE = (atomMaps.find(l))->second;
    AtomEmbedding::iterator aEit = aE.find(h);
    //an embedding exists, but h has not been added yet
    if(aEit==aE.end()) {
        return addNode(h,l);
    } else {
        return aEit->second;
    }
}

HandleSeq DimEmbedModule::kNearestNeighbors(const Handle& h, const Type& l, int k) {
    if(!classserver().isLink(l))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(l).c_str());
    logger().info("%d nearest neighbours start", k);
    if(!isEmbedded(l)) {
        const char* tName = classserver().getTypeName(l).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }

    AtomEmbedding aE = (atomMaps.find(l))->second;
    AtomEmbedding::iterator aEit = aE.find(h);
    //an embedding exists, but h has not been added yet
    if(aEit==aE.end()) {
        logger().error("Embedding exists, but %s is not in it",
                       TLB::getAtom(h)->toString().c_str());
        throw std::string("Embedding exists, but %s is not in it",
                          TLB::getAtom(h)->toString().c_str());
    } else {
        v_array<CoverTreeNode> v = v_array<CoverTreeNode>();
        //CoverTreeNode c = CoverTreeNode(aEit->first, &(aEit->second));
        CoverTreeNode c = CoverTreeNode(&(aEit->second));
        push(v,c);
        v_array<v_array<CoverTreeNode> > res;
        k_nearest_neighbor(embedTreeMap[l], batch_create(v), res, k);
        HandleSeq results = HandleSeq();
        for (int j = 1; j<res[0].index; j++) {
            print(res[0][j]);
            //results.push_back(*(distMap.find(res[0][j].getVector()));
            //results.push_back(res[0][j].getHandle());
        }
        //printf("\n");
        
        logger().info("kNN done");
        return results;
    }
}

void DimEmbedModule::addPivot(const Handle& h, const Type& linkType){   
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());

    std::list<Handle> nodes;
    as->getHandleSet(std::back_inserter(nodes), NODE, true);
    std::map<Handle,double> distMap;
    std::multimap<double,Handle> pQueue;
    for(std::list<Handle>::iterator it=nodes.begin(); it!=nodes.end(); ++it){
        if(*it==h) {
            pQueue.insert(std::pair<double, Handle>(1,*it));
            distMap[*it]=1;
        } else {
            pQueue.insert(std::pair<double, Handle>(0,*it));
            distMap[*it]=0;
        }
    }
    
    pivotsMap[linkType].push_back(h);
    while(!pQueue.empty()) {
        Handle& u = (*(pQueue.rbegin())).second;//extract min
        pQueue.erase(--pQueue.end());
        if (distMap[u]==0) { break;}
        HandleSeq newLinks = as->getIncoming(u);
        for(HandleSeq::iterator it=newLinks.begin(); it!=newLinks.end(); ++it){
            //ignore links that aren't of type linkType
            if(as->getType(*it)!=linkType) continue;
            const TruthValue& linkTV = as->getTV(*it);
            HandleSeq newNodes = as->getOutgoing(*it);
            for(HandleSeq::iterator it2=newNodes.begin();
                it2!=newNodes.end(); it2++) {
                double alt = distMap[u]*
                    linkTV.getMean()*linkTV.getConfidence();
                double oldDist=distMap[*it2];
                if(alt>oldDist) {
                    multimap<double,Handle>::iterator it3;
                    std::pair<std::multimap<double,Handle>::iterator,
                        std::multimap<double,Handle>::iterator> itPair
                        = pQueue.equal_range(oldDist);
                     for(it3=itPair.first;it3!=itPair.second;it3++) {
                         if(it3->second==*it2) {pQueue.erase(it3); break;}
                    }
                    pQueue.insert(std::pair<double, Handle>(alt,*it2));
                    distMap[*it2]=alt;
                }
            }
        }
    }
    for(std::map<Handle, double>::iterator it = distMap.begin();
        it!=distMap.end();it++) {
        atomMaps[linkType][(*it).first].push_back(distMap[(*it).first]);
    }
}

void DimEmbedModule::embedAtomSpace(const Type& linkType){    
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());

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
        //pick the next pivot to maximize its distance from its closest pivot
        //(maximizing distance = minimizing path weight)
        for(HandleSeq::iterator it=nodes.begin(); it!=nodes.end(); ++it){
            std::vector<double> eV = atomMaps[linkType][*it];
            double testChoiceWeight = *std::max_element(eV.begin(), eV.end());
            if(testChoiceWeight < bestChoiceWeight) {
                bestChoice = *it;
                bestChoiceWeight = testChoiceWeight;
            }
        }
    }

    //Now that all the points are calculated, we have to construct a
    //cover tree for them.
    v_array<CoverTreeNode> v = v_array<CoverTreeNode>();
    AtomEmbedding aE = atomMaps[linkType];
    AtomEmbedding::iterator it = aE.begin();
    for(;it!=aE.end();it++) {
        //CoverTreeNode c = CoverTreeNode(it->first, &(it->second));
        CoverTreeNode c = CoverTreeNode(&(it->second));
        push(v,c);
    }
    embedTreeMap[linkType]=batch_create(v);
    v_array<v_array<CoverTreeNode> > res;
    //kNearestNeighbors(aE.begin()->first,linkType,1);
    //kNearestNeighbors(aE.begin()->first,linkType,2);
    //kNearestNeighbors(aE.begin()->first,linkType,4);
    //kNearestNeighbors(aE.begin()->first,linkType,6);
    //kNearestNeighbors(aE.begin()->first,linkType,10);
    kNearestNeighbors(aE.begin()->first,linkType,5);
    kNearestNeighbors(aE.begin()->first,linkType,5);
    kNearestNeighbors(aE.begin()->first,linkType,5);
    kNearestNeighbors(aE.begin()->first,linkType,5);
    kNearestNeighbors(aE.begin()->first,linkType,5);
    kNearestNeighbors(aE.begin()->first,linkType,5);    
}

std::vector<double> DimEmbedModule::addNode(const Handle& h,
                                                  const Type& linkType){
    
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
    
    PivotSeq& pivots=pivotsMap[linkType];
    std::vector<double> embeddingVector;
    //The i'th entry of the handle's embeddingVector is the value of the
    //highest weight path between the handle and the i'th pivot.
    for(PivotSeq::iterator it=pivots.begin(); it!=pivots.end(); ++it){
        embeddingVector.push_back(findHighestWeightPath(h, *it, linkType));
    }
    atomMaps[linkType][h]=embeddingVector;
    return embeddingVector;
}

void DimEmbedModule::clearEmbedding(const Type& linkType){
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());

    atomMaps.erase(linkType);
    pivotsMap.erase(linkType);
}

void DimEmbedModule::logAtomEmbedding(const Type& linkType) {
    AtomEmbedding atomEmbedding=atomMaps[linkType];
    PivotSeq pivots = pivotsMap[linkType];

    std::ostringstream oss;
    
    oss << "PIVOTS:" << std::endl;
    for(PivotSeq::const_iterator it=pivots.begin(); it!=pivots.end(); ++it){
        if(as->isValidHandle(*it)) {
            oss << as->atomAsString(*it,true) << std::endl;
        } else {
            oss << "[PIVOT'S BEEN DELETED]" << std::endl;
        }
    }
    oss << "Node Embeddings:" << std::endl;
    AtomEmbedding::const_iterator it;
    for(it=atomEmbedding.begin(); it!=atomEmbedding.end(); ++it){
        if(as->isValidHandle(it->first)) {
            oss << as->atomAsString(it->first,true) << " : (";
        } else {
            //oss << "[NODE'S BEEN DELETED]" << " : (";
            //oss << atom->toShortString() << " : (";
        }
        std::vector<double> embedVector = it->second;
        for(std::vector<double>::const_iterator it2=embedVector.begin();
            it2!=embedVector.end();
            ++it2){
            oss << *it2 << " ";
        }
        //oss << ")" << std::endl;
        oss << std::endl;
    }
    logger().info(oss.str());
    return;
}

bool DimEmbedModule::isEmbedded(const Type& linkType) {
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());

    //See if atomMaps holds an embedding for linkType
    AtomEmbedMap::iterator aEMit = atomMaps.find(linkType);
    if(aEMit==atomMaps.end()) {
        return false;
    }
    return true;
}


double DimEmbedModule::euclidDist(const Handle& h1,
                                  const Handle& h2,
                                  const Type& l) {
    if(!classserver().isLink(l))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(l).c_str());
 
    std::vector<double> v1=getEmbedVector(h1,l);
    std::vector<double> v2=getEmbedVector(h2,l);
    return euclidDist(v1, v2);
}

double DimEmbedModule::euclidDist(std::vector<double> v1,
                                  std::vector<double> v2) {
    assert(v1.size()==v2.size());
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
}
