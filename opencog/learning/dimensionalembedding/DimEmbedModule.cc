
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

typedef std::vector<std::pair<HandleSeq,std::vector<double> > >
    ClusterSeq; //the vector of doubles is the centroid of the cluster

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
    define_scheme_primitive("kMeansCluster",
                            &DimEmbedModule::addKMeansClusters,
                            this);
#endif
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
                       as->atomAsString(h).c_str());
        throw std::string("Embedding exists, but %s is not in it",
                          as->atomAsString(h).c_str());
    } else {
        v_array<CoverTreeNode> v = v_array<CoverTreeNode>();
        CoverTreeNode c = CoverTreeNode(aEit);
        push(v,c);
        v_array<v_array<CoverTreeNode> > res;
        k_nearest_neighbor(embedTreeMap[l], batch_create(v), res, k);
        HandleSeq results = HandleSeq();
        for (int j = 1; j<res[0].index; ++j) {
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

    as->incVLTI(h);//We don't want pivot atoms to be forgotten...
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
                it != distMap.end(); ++it) {
            std::cout << it->second << ":h=" << it->first << ",";
        }
        std::cout << "}" << std::endl;*/
        
        //we can't simply erase p_it->first because there could be multiple
        //nodes that are the same distance from our pivot, and we can't
        //pass erase p_it because it's a reverseiterator, thus this awkwardness
        pQueue_t::iterator erase_it = pQueue.end();
        pQueue.erase(--erase_it);
        
        if (distMap[u]==0) { break;}
        HandleSeq newLinks = as->getIncoming(u);
        for(HandleSeq::iterator it=newLinks.begin(); it!=newLinks.end(); ++it){
            //ignore links that aren't of type linkType
            if(as->getType(*it)!=linkType) continue;
            TruthValuePtr linkTV = as->getTV(*it);
            HandleSeq newNodes = as->getOutgoing(*it);
            for(HandleSeq::iterator it2=newNodes.begin();
                it2!=newNodes.end(); ++it2) {
                double alt =
                    distMap[u] * linkTV->getMean() * linkTV->getConfidence();
                double oldDist=distMap[*it2];
                //If we've found a better (higher weight) path, update distMap
                if(alt>oldDist) {
                    pQueue_t::iterator it3;

                    std::pair<pQueue_t::iterator, pQueue_t::iterator>
                        itPair = pQueue.equal_range(oldDist);

                    for(it3=itPair.first;it3!=itPair.second;++it3) {
                         if(it3->second==*it2) {pQueue.erase(it3); break;}
                    }
                    pQueue.insert(std::pair<double, Handle>(alt,*it2));
                    distMap[*it2]=alt;
                }
            }
        }
    }
    for(std::map<Handle, double>::iterator it = distMap.begin();
            it != distMap.end(); ++it) {
        atomMaps[linkType][it->first].push_back(it->second);
    }
}

void DimEmbedModule::embedAtomSpace(const Type& linkType,
                                    const int _numDimensions)
{
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
    //logger().info("starting embedding");
    clearEmbedding(linkType);

    // Scheme wrapper doesn't deal with unsigned ints, so double check it's not
    // negative, or zero for that matter
    unsigned int numDimensions = 5;
    if (_numDimensions > 0) numDimensions = _numDimensions;

    dimensionMap[linkType]=numDimensions;
    HandleSeq nodes;
    as->getHandleSet(std::back_inserter(nodes), NODE, true);
    
    HandleSeq& pivots = pivotsMap[linkType];
    if(nodes.empty()) return;
    Handle bestChoice = nodes.back();

    while((pivots.size() < (unsigned int) numDimensions) && (!nodes.empty())){
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
    for(;it!=aE.end();++it) {
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
    //with link weight .6, then the new node's embedding is the vector...
    //(max(.3*.1,.8*.4),max(.3*.2,.6*.5))    
    for(HandleSeq::iterator it=links.begin(); it<links.end(); ++it) {
        HandleSeq nodes = as->getOutgoing(*it);
        TruthValuePtr linkTV = as->getTV(*it);        
        double weight = linkTV->getConfidence()*linkTV->getMean();
        for(HandleSeq::iterator it2=nodes.begin();it2<nodes.end(); ++it2) {
            if(*it2==h) continue;
            std::vector<double> embedding =
                getEmbedVector(*it2,linkType);
            //Alter our embedding whenever we find a higher weight path
            for(unsigned int i=0; i<embedding.size(); ++i) {
                if(weight*embedding[i]>newEmbedding[i]) {
                    newEmbedding[i]=weight*embedding[i];
                }
            }
        }
    }
    //TODO:add the node to the cover tree
    return newEmbedding;
}

void DimEmbedModule::addLink(const Handle& h, const Type& linkType) {
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());

    std::vector<std::vector<double> > embeddingVectors;
    AtomEmbedding aE = atomMaps[linkType];
    HandleSeq nodes = as->getOutgoing(h);
    for(HandleSeq::iterator it=nodes.begin(); it!=nodes.end(); ++it) {
        embeddingVectors.push_back(aE[*it]);
    }
    std::vector<std::vector<double> >::iterator it,it2;
    int i=0;
    bool changed = false;
    double weight = as->getMean(h)*as->getConfidence(h);
    for(it=embeddingVectors.begin(); it!=embeddingVectors.end(); ++it) {
        for(it2=embeddingVectors.begin(); it2!=embeddingVectors.end(); ++it2) {
            if((*it)[i]<(weight*(*it2)[i])) {
                (*it)[i]=(weight*(*it2)[i]);
                changed=true;
            }
        }
        i++;
    }
    if(changed) {
        //TODO: update the cover tree
    }
    return;
}

void DimEmbedModule::clearEmbedding(const Type& linkType){
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());

    HandleSeq pivots  = pivotsMap[linkType];
    for(HandleSeq::iterator it = pivots.begin(); it!=pivots.end(); ++it) {
        as->decVLTI(*it);
    }
    atomMaps.erase(linkType);
    pivotsMap.erase(linkType);
    dimensionMap.erase(linkType);
}

void DimEmbedModule::logAtomEmbedding(const Type& linkType) {
    AtomEmbedding atomEmbedding=atomMaps[linkType];
    HandleSeq pivots = pivotsMap[linkType];

    std::ostringstream oss;
    
    oss << "PIVOTS:" << std::endl;
    for(HandleSeq::const_iterator it=pivots.begin(); it!=pivots.end(); ++it){
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
    for (; mit != atomMaps.end(); ++mit) {
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

ClusterSeq DimEmbedModule::kMeansCluster(const Type& l, int numClusters) {
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
    for(;aEit!=aE.end();++aEit) {
        handleArray[i]=aEit->first;
        std::vector<double> embedding = aEit->second;
        std::vector<double>::iterator vit=embedding.begin();
        j=0;
        for(;vit!=embedding.end();++vit) {
            embedMatrix[i][j]=*vit;
            mask[i][j]=1;
            j++;
        }
        i++;
    }
    double* weight = new double[numDimensions];
    for(int i=0;i<numDimensions;++i) {weight[i]=1;}

    int* clusterid = new int[numVectors]; //stores the result of clustering
    double error;
    int ifound;
    for(int i=0;i<numVectors;++i) {
        for(int j=0;j<numDimensions;++j) {
            mask[i][j]=1;
        }
    }

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
    
    //Stores the centroid of each cluster in centroidMatrix
    ::getclustercentroids(numClusters, numVectors,
                          numDimensions, embedMatrix,
                          mask, clusterid, centroidMatrix,
                          cmask, 0, 'a');
    ClusterSeq clusters(numClusters);
    for(int i=0;i<numVectors;++i) {
        //clusterid[i] indicates which cluster handleArray[i] is in.
        int clustInd=clusterid[i];
        clusters[clustInd].first.push_back(handleArray[i]);
    }
    for(int i=0;i<numClusters;++i) {
        std::vector<double> centroid(centroidMatrix[i],
                                     centroidMatrix[i]+numDimensions);
        clusters[i].second=centroid;
    }
    
    //for(std::vector<HandleSeq>::const_iterator it=clusters.begin();
    //    it!=clusters.end(); ++it) {
    //    std::cout << "Homogeneity: " << homogeneity(*it,l) << std::endl;
    //    std::cout << "Separation: " << separation(*it,l) << std::endl;
    //}

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

    return clusters;
}

void DimEmbedModule::addKMeansClusters(const Type& l, int maxClusters,
                                       double threshold, int kPasses) {
    AtomEmbedding aE = atomMaps[l];

    typedef std::pair<double,std::pair<HandleSeq,vector<double> > > cPair;
    typedef std::multimap<double,std::pair<HandleSeq,vector<double> > >
        pQueue_t;
    pQueue_t clusters;
    //make a Priority Queue of (HandleSeq,vector<double>) pairs, where
    //we can easily extract those with the lowest value to discard if we
    //exceed maxClusters.
    for(int i=1;i<kPasses+1;++i) {
        int k = std::pow(2,std::log(aE.size()/std::log(2)));
        ClusterSeq newClusts = kMeansCluster(l,k);
        for(ClusterSeq::iterator it = newClusts.begin();
            it!=newClusts.end();++it) {
            //if we still have room for more clusters and the cluster quality
            //is high enough, insert the cluster into the pQueue
            double quality = separation(it->first,l)*homogeneity(it->first,l);
            if(quality>threshold) {
                if( (int) clusters.size() < maxClusters) {
                    clusters.insert(cPair(quality,*it));
                } else {
                    //if there is no room, but our new cluster is better
                    //than the worst current cluster, remove it and add new one
                    pQueue_t::iterator p_it = clusters.begin();
                    if(quality>p_it->first) {
                        clusters.erase(p_it);
                        clusters.insert(cPair(quality,*it));
                    }
                }
            }
        }
    }
    //Make a new node for each cluster and connect it with InheritanceLinks.
    for(pQueue_t::iterator it = clusters.begin();it!=clusters.end();++it) {
        HandleSeq cluster = it->second.first;
        std::vector<double> centroid = it->second.second;
        Handle newNode = as->addPrefixedNode(CONCEPT_NODE, "cluster_");
        for(HandleSeq::iterator it2=cluster.begin();it2!=cluster.end();++it2) {
            //Connect newNode to each handle in its cluster.
            //@TODO: Decide how to set this truth value.
            SimpleTruthValue tv(.99,
                                SimpleTruthValue::confidenceToCount(.99));
            as->addLink(INHERITANCE_LINK, *it2, newNode, tv);
        }
    }
}

double DimEmbedModule::homogeneity(const HandleSeq& cluster,
                                   const Type& linkType) {
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
    OC_ASSERT(cluster.size()>1);
    
    double average=0;
    for(HandleSeq::const_iterator it=cluster.begin();it!=cluster.end();++it) {
        double minDist=DBL_MAX;
        //find the distance to nearest clustermate
        for(HandleSeq::const_iterator it2=cluster.begin();
                                      it2!=cluster.end();++it2) {
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
    for(AtomEmbedding::iterator it=aE.begin();it!=aE.end();++it) {
        bool inCluster=false; //whether *it is in cluster
        bool better=false; //whether *it is closer to some element of cluster
                           //than minDist
        double dist;
        for(HandleSeq::const_iterator it2=cluster.begin();
                                      it2!=cluster.end();++it2) {
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
    for(int i=0; i<size;++i) {
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

double DimEmbedModule::euclidDist(const std::vector<double>& v1,
                                  const std::vector<double>& v2) {
    OC_ASSERT(v1.size()==v2.size());
    std::vector<double>::const_iterator it1=v1.begin();
    std::vector<double>::const_iterator it2=v2.begin();

    double distance=0;
    //Calculate euclidean distance between v1 and v2
    for(; it1 != v1.end(); ++it1, ++it2)
        distance+=sq(*it1 - *it2);
    distance=sqrt(distance);
    return distance;
}

void DimEmbedModule::handleAddSignal(AtomSpaceImpl* a, Handle h) {
    AtomEmbedMap::iterator it;
    if(as->isNode(h)) {
        //for each link type embedding that exists, add the node
        for(it=atomMaps.begin(); it!=atomMaps.end(); ++it) {
            addNode(h,it->first);
        }
    }
    else {//h is a link    
        for(it=atomMaps.begin(); it!=atomMaps.end(); ++it) {
            addLink(h,it->first);
        }
    }
}
