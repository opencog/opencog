
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
extern "C" {
#include <opencog/util/cluster.h>
}
#include <opencog/atomspace/SimpleTruthValue.h>

using namespace opencog;

DECLARE_MODULE(DimEmbedModule)

DimEmbedModule::DimEmbedModule(AtomSpace* atomSpace) {
    this->as=atomSpace;
}
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
    define_scheme_primitive("kNN",
                            &DimEmbedModule::kNearestNeighbors,
                            this);
    define_scheme_primitive("cluster",
                            &DimEmbedModule::cluster,
                            this);
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

                const TruthValue* linkTV = as->getTV(*it);
                //If this path is better than the currently known one, save it
                double pathWeight
                    = bestNode.second*(linkTV->getMean()*linkTV->getConfidence());
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
    AtomEmbedding::const_iterator aEit = aE.find(h);
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
    //logger().info("%d nearest neighbours start", k);
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
                       this->as->atomAsString(h).c_str());
        throw std::string("Embedding exists, but %s is not in it",
                          this->as->atomAsString(h).c_str());
    } else {
        v_array<CoverTreeNode> v = v_array<CoverTreeNode>();
        CoverTreeNode c = CoverTreeNode(aEit);
        push(v,c);
        v_array<v_array<CoverTreeNode> > res;
        k_nearest_neighbor(embedTreeMap[l], batch_create(v), res, k);
        HandleSeq results = HandleSeq();
        for (int j = 1; j<res[0].index; j++) {
            //print(*(this->as), res[0][j]);
            results.push_back(res[0][j].getHandle());
        }
        return results;
    }
}

void DimEmbedModule::addPivot(const Handle& h, const Type& linkType){   
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());

    HandleSeq nodes;
    as->getHandleSet(std::back_inserter(nodes), NODE, true);

    std::map<Handle,double> distMap;

    typedef std::multimap<double,Handle> pQueue_t;
    pQueue_t pQueue;
    for(HandleSeq::iterator it=nodes.begin(); it!=nodes.end(); ++it){
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
        pQueue_t::reverse_iterator p_it = pQueue.rbegin();
        Handle u = p_it->second;//extract min

        /*std::cout << "U=" << u << " distmap={";
        for(std::map<Handle, double>::iterator it = distMap.begin();
                it != distMap.end(); it++) {
            std::cout << it->second << ":h=" << it->first << ",";
        }
        std::cout << "}" << std::endl;*/

        pQueue.erase(p_it->first);
        if (distMap[u]==0) { break;}
        HandleSeq newLinks = as->getIncoming(u);
        for(HandleSeq::iterator it=newLinks.begin(); it!=newLinks.end(); ++it){
            //ignore links that aren't of type linkType
            if(as->getType(*it)!=linkType) continue;
            const TruthValue* linkTV = as->getTV(*it);
            HandleSeq newNodes = as->getOutgoing(*it);
            for(HandleSeq::iterator it2=newNodes.begin();
                it2!=newNodes.end(); it2++) {
                double alt =
                    distMap[u] * linkTV->getMean() * linkTV->getConfidence();
                double oldDist=distMap[*it2];
                //If we've found a better (higher weight) path, update distMap
                if(alt>oldDist) {
                    pQueue_t::iterator it3;

                    std::pair<pQueue_t::iterator, pQueue_t::iterator>
                        itPair = pQueue.equal_range(oldDist);

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
            it != distMap.end(); it++) {
        atomMaps[linkType][it->first].push_back(it->second);
    }
}

void DimEmbedModule::embedAtomSpace(const Type& linkType,
                                    const int numDimensions){    
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
    //logger().info("starting embedding");
    clearEmbedding(linkType);
    dimensionMap[linkType]=numDimensions;
    HandleSeq nodes;
    as->getHandleSet(std::back_inserter(nodes), NODE, true);

    //for(HandleSeq::iterator it=nodes.begin(); it!=nodes.end(); ++it){
    //    std::cout << as->atomAsString(*it,true) << std::endl;
    //}
    PivotSeq& pivots = pivotsMap[linkType];
    if(nodes.empty()) return;
    Handle bestChoice = nodes.back();

    while((pivots.size() < numDimensions) && (!nodes.empty())){
        addPivot(bestChoice, linkType);
        logger().info("Pivot %d picked", pivots.size());
        nodes.erase(std::find(nodes.begin(), nodes.end(), bestChoice));

        bestChoice = nodes[0];
        double bestChoiceWeight = 1;
        //pick the next pivot to maximize its distance from its closest pivot
        //(maximizing distance = minimizing path weight)
        for(HandleSeq::iterator it=nodes.begin(); it!=nodes.end(); ++it){
            std::vector<double>& eV = atomMaps[linkType][*it];
            double testChoiceWeight = *std::max_element(eV.begin(), eV.end());
            if(testChoiceWeight < bestChoiceWeight) {
                bestChoice = *it;
                bestChoiceWeight = testChoiceWeight;
            }
        }
    }
    //Now that all the points are calculated, we construct a
    //cover tree for them.
    v_array<CoverTreeNode> v = v_array<CoverTreeNode>();
    AtomEmbedding& aE = atomMaps[linkType];
    
    AtomEmbedding::const_iterator it = aE.begin();
    for(;it!=aE.end();it++) {
        CoverTreeNode c = CoverTreeNode(it);
        push(v,c);
    }
    embedTreeMap[linkType]=batch_create(v);
    //logger().info("done embedding");
}

std::vector<double> DimEmbedModule::addNode(const Handle& h,
                                                  const Type& linkType){    
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());

    HandleSeq links = as->getIncoming(h);
    std::vector<double> newEmbedding (dimensionMap[linkType], 0.0);
    //The embedding for each coordinate is the max of
    //tv.strength*tv.confidence*embedding of every directly connected
    //node.
    //eg if our new node is connected to node A with embedding (.1,.2)
    //with link weight .3, and to node B with embedding (.4,.5)
    //with link weight .6, then the new node's embedding is...
    //(max(.3*.1,.8*.4),max(.3*.2,.6*.5))    
    for(HandleSeq::iterator it=links.begin(); it<links.end(); it++) {
        HandleSeq nodes = as->getOutgoing(*it);
        const TruthValue& linkTV = *(as->getTV(*it));        
        double weight = linkTV.getConfidence()*linkTV.getMean();
        for(HandleSeq::iterator it2=nodes.begin();it2<nodes.end(); it2++) {
            if(*it2==h) continue;
            std::vector<double> embedding =
                getEmbedVector(*it2,linkType);
            //Alter our embedding whenever we find a higher weight path
            for(int i=0; i<embedding.size(); i++) {
                if(weight*embedding[i]>newEmbedding[i]) {
                    newEmbedding[i]=weight*embedding[i];
                }
            }
        }
    }
    return newEmbedding;
}

void DimEmbedModule::clearEmbedding(const Type& linkType){
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());

    atomMaps.erase(linkType);
    pivotsMap.erase(linkType);
    dimensionMap.erase(linkType);
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
            oss << "[NODE'S BEEN DELETED H=" << it->first << "] : (";
        }
        const std::vector<double>& embedvector = it->second;
        for(std::vector<double>::const_iterator it2=embedvector.begin();
            it2!=embedvector.end();
            ++it2){
            oss << *it2 << " ";
        }
        oss << ")" << std::endl;
    }
    logger().info(oss.str());
    return;
}

void DimEmbedModule::printEmbedding() {
    std::ostringstream oss;
    AtomEmbedMap::const_iterator mit = atomMaps.begin();
    oss << "Node Embeddings" << std::endl;
    for (; mit != atomMaps.end(); mit++) {
        oss << "=== for type" << classserver().getTypeName(mit->first).c_str() << std::endl;
        AtomEmbedding atomEmbedding=mit->second;
        AtomEmbedding::const_iterator it;
        for(it=atomEmbedding.begin(); it!=atomEmbedding.end(); ++it){
            if(as->isValidHandle(it->first)) {
                oss << as->atomAsString(it->first,true) << " : (";
            } else {
                oss << "[NODE'S BEEN DELETED. handle=";
                oss << it->first << "] : (";
            }
            const std::vector<double>& embedVector = it->second;
            for(std::vector<double>::const_iterator it2=embedVector.begin();
                it2!=embedVector.end();
                ++it2){
                oss << *it2 << " ";
            }
            oss << ")" << std::endl;
        }
    }
    std::cout << oss.str();
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

void DimEmbedModule::cluster(const Type& l, int numClusters) {
    if(!isEmbedded(l)) {
        const char* tName = classserver().getTypeName(l).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    int numDimensions=dimensionMap[l];
    int npass=1;
    AtomEmbedding& aE = atomMaps[l];
    int numVectors=aE.size();
    if(numVectors<numClusters) {
        logger().error("Cannot make more clusters than there are nodes");
        throw std::string("Cannot make more clusters than there are nodes");
    }
    //create the required matrices for the clustering function (which
    //takes double** as an argument...)
    double* embedding = new double[numDimensions*numVectors];
    double** embedMatrix = new double*[numVectors];
    int* maskArray = new int[numDimensions*numVectors];
    int** mask = new int*[numVectors];
    for(int i=0;i<numVectors;++i) {
        embedMatrix[i] = embedding + numDimensions*i;
        mask[i] = maskArray + numDimensions*i;
    }
    Handle* handleArray = new Handle[numVectors];
    AtomEmbedding::iterator aEit=aE.begin();
    int i=0;
    int j;
    //add the values to the embeddingmatrix...
    for(;aEit!=aE.end();aEit++) {
        handleArray[i]=aEit->first;
        std::vector<double> embedding = aEit->second;
        std::vector<double>::iterator vit=embedding.begin();
        j=0;
        for(;vit!=embedding.end();vit++) {
            embedMatrix[i][j]=*vit;
            mask[i][j]=1;
            j++;
        }
        i++;
    }
    double* weight = new double[numDimensions];
    for(int i=0;i<numDimensions;i++) {weight[i]=1;}

    int* clusterid = new int[numVectors]; //stores the result of clustering
    double error;
    int ifound;
    //clusterid[i]==j will indicate that handle i belongs in cluster j
    ::kcluster(numClusters, numVectors, numDimensions, embedMatrix,
               mask, weight, 0, npass, 'a', 'e', clusterid, &error, &ifound);

    //Now that we have the clusters (stored in clusterid), we find the centroid
    //of each cluster so we will know how to weight the inheritance links
    double* centroidArray = new double[numClusters*numDimensions];
    double** centroidMatrix = new double*[numClusters];
    
    int* cmaskArray = new int[numClusters*numDimensions];
    int** cmask = new int*[numClusters];
    for(int i=0;i<numClusters;++i) {
        centroidMatrix[i] = centroidArray + numDimensions*i;
        cmask[i] = cmaskArray + numDimensions*i;
    }
    for(int i=0;i<numClusters;i++){
        for(int j=0;j<numDimensions;j++){
            cmask[i][j]=1;
        }
    }
    ::getclustercentroids(numClusters, numVectors, numDimensions, embedMatrix,
                          mask, clusterid, centroidMatrix, mask, 0, 'a');
    HandleSeq clusterNodes (numClusters);
    for(int i=0;i<numClusters;i++) {
        clusterNodes[i] = as->addPrefixedNode(CONCEPT_NODE, "cluster_");
    }

    std::vector<HandleSeq> clusters(numClusters);
    for(int i=0;i<numVectors;i++) {
        //clusterid[i] indicates which cluster handleArray[i] is in.
        //so for each cluster, we make a new ConceptNode and make
        //inheritancelinks between it and all of its consituent nodes.
        int clustInd=clusterid[i];
        double dist = euclidDist(centroidMatrix[clustInd],embedMatrix[i],
                                 numDimensions);
        //TODO: How should we set the truth value for this link? (this is just
        //a placeholder)
        SimpleTruthValue tv(1-dist,
                            SimpleTruthValue::confidenceToCount(1-dist));
        //    as->addLink(INHERITANCE_LINK, handleArray[i],
        //            clusterNodes[clustInd], tv);
        clusters[clustInd].push_back(handleArray[i]);
    }
    
    for(std::vector<HandleSeq>::const_iterator it=clusters.begin();
        it!=clusters.end(); it++) {
        std::cout << "Homogeneity: " << homogeneity(*it,l) << std::endl;
        std::cout << "Separation: " << separation(*it,l) << std::endl;
    }

    delete[] embedding;
    delete[] embedMatrix;
    delete[] maskArray;
    delete[] mask;
    delete[] handleArray;
    delete[] weight;
    delete[] clusterid;
    delete[] centroidArray;
    delete[] centroidMatrix;
    delete[] cmaskArray;
    delete[] cmask;
}

double DimEmbedModule::homogeneity(const HandleSeq& cluster,
                                   const Type& linkType) {
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
    assert(cluster.size()>1);
    
    double average=0;
    for(HandleSeq::const_iterator it=cluster.begin();it!=cluster.end();it++) {
        double minDist=DBL_MAX;
        //find the distance to nearest clustermate
        for(HandleSeq::const_iterator it2=cluster.begin();
                                      it2!=cluster.end();it2++) {
            if(*it==*it2) continue;
            double dist=euclidDist(*it,*it2,linkType);
            if(dist<minDist) minDist=dist;
        }
        average+=minDist;
    }
    average = average/cluster.size();
    return 1.0/(1.0 + average);  //h=1/(1+A)
}

double DimEmbedModule::separation(const HandleSeq& cluster,
                                  const Type& linkType) {
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());

    AtomEmbedding aE = (atomMaps.find(linkType))->second;
    double minDist=DBL_MAX;
    for(AtomEmbedding::iterator it=aE.begin();it!=aE.end();it++) {
        bool inCluster=false; //whether *it is in cluster
        bool better=false; //whether *it is closer to some element of cluster
                           //than minDist
        double dist;
        for(HandleSeq::const_iterator it2=cluster.begin();
                                      it2!=cluster.end();it2++) {
            if(it->first==*it2) {
                inCluster=true;
                break;
            }
            dist = euclidDist(it->second,aE[*it2]);
            if(dist<minDist) better=true;
        }
        //If the node is closer and it is not in the cluster, update minDist
        if(better && !inCluster) {
            minDist=dist;
        }
        better=false;
        inCluster=false;
    }
    return minDist;
}

double DimEmbedModule::euclidDist(double v1[], double v2[], int size) {
    double dist=0;
    for(int i=0; i<size;i++) {
        dist+=std::pow((v1[i]-v2[i]), 2);
    }
    dist=sqrt(dist);
    return dist;
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
    for(; it1 != v1.end(); it1++) {
        distance+=std::pow((*it1 - *it2), 2);
        if(it2!=v2.end()) it2++;
    }
    distance=sqrt(distance);
    return distance;
}
