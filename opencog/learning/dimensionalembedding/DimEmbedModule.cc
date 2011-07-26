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
#include <string>
extern "C" {
#include <opencog/util/cluster.h>
}
#include <opencog/atomspace/SimpleTruthValue.h>

using namespace opencog;

typedef std::vector<std::pair<HandleSeq,std::vector<double> > >
    ClusterSeq; //the vector of doubles is the centroid of the cluster

DECLARE_MODULE(DimEmbedModule)

DimEmbedModule::DimEmbedModule(AtomSpace* atomSpace) {
    logger().info("[DimEmbedModule] constructor");
    this->as=atomSpace;
    as->atomSpaceAsync->
        addAtomSignal(boost::bind(&DimEmbedModule::handleAddSignal,
                                  this, _1, _2));
    as->atomSpaceAsync->
        removeAtomSignal(boost::bind(&DimEmbedModule::handleRemoveSignal,
                                     this, _1, _2));
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
    as->atomSpaceAsync->
        addAtomSignal(boost::bind(&DimEmbedModule::handleAddSignal,
                                  this, _1, _2));
    as->atomSpaceAsync->
        removeAtomSignal(boost::bind(&DimEmbedModule::handleRemoveSignal,
                                     this, _1, _2));
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

const std::vector<double>& DimEmbedModule::getEmbedVector(const Handle& h,
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
    const AtomEmbedding& aE = (atomMaps.find(l))->second;
    AtomEmbedding::const_iterator aEit = aE.find(h);
    return aEit->second;
}

const HandleSeq& DimEmbedModule::getPivots(const Type& l) {
    if(!classserver().isLink(l))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(l).c_str());
    if(!isEmbedded(l)) {
        const char* tName = classserver().getTypeName(l).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    return pivotsMap[l];
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
    if(aEit==aE.end()) { //an embedding exists, but h has not been added yet
        logger().error("Embedding exists, but %s is not in it",
                       as->atomAsString(h).c_str());
        throw std::string("Embedding exists, but %s is not in it",
                          as->atomAsString(h).c_str());
    } else {
        EmbedTreeMap::iterator treeMapIt = embedTreeMap.find(l);
        //if there is an AtomEmbedding, there should also be a tree.
        OC_ASSERT(treeMapIt!=embedTreeMap.end());
        CoverTree<CoverTreePoint>& cTree = treeMapIt->second;
        std::vector<CoverTreePoint>
            points = cTree.kNearestNeighbors(CoverTreePoint(h,aEit->second),k); 
        HandleSeq results;
        std::vector<CoverTreePoint>::const_iterator it;
        for (it=points.begin(); it!=points.end(); it++) {
            results.push_back(it->getHandle());
        }
        return results;
    }
}

void DimEmbedModule::addPivot(const Handle& h, const Type& linkType, bool backward){   
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
    bool symmetric = classserver().isA(linkType,UNORDERED_LINK);
    if(!backward) as->incVLTI(h);//We don't want pivot atoms to be forgotten...
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
            //ignore links that aren't a subtype of type linkType
            if(!classserver().isA(as->getType(*it),linkType));
            TruthValuePtr linkTV = as->getTV(*it);
            HandleSeq newNodes = as->getOutgoing(*it);
            HandleSeq::iterator it2=newNodes.begin();
            if(!symmetric && !backward) {
                if(*it2==u) ++it2;
                else continue;
            }
            if(!symmetric && backward) {
                if(*it2==u) {
                    continue;
                } else {
                    newNodes.clear();
                    newNodes.push_back(*it2);
                    it2=newNodes.begin();
                }
            }
            for(;it2!=newNodes.end(); ++it2) {
                if(!as->isNode(*it2)) continue;
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
        if(symmetric) {
            atomMaps[linkType][it->first].push_back(it->second);
        } else {
            if(backward) asymAtomMaps[linkType].second[it->first].push_back(it->second);
            else asymAtomMaps[linkType].first[it->first].push_back(it->second);
        }
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
    bool symmetric = classserver().isA(linkType,UNORDERED_LINK);
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
        if(symmetric) {
            addPivot(bestChoice, linkType);
        } else {
            addPivot(bestChoice, linkType, false);
            addPivot(bestChoice, linkType, true);
        }
        logger().info("Pivot %d picked", pivots.size());
        nodes.erase(std::find(nodes.begin(), nodes.end(), bestChoice));

        bestChoice = nodes[0];
        double bestChoiceWeight = 1;
        //pick the next pivot to maximize its distance from its closest pivot
        //(maximizing distance = minimizing path weight)
        for(HandleSeq::iterator it=nodes.begin(); it!=nodes.end(); ++it){
            std::vector<double> eV;
            if(symmetric) eV = atomMaps[linkType][*it];
            else eV = asymAtomMaps[linkType].first[*it];
            double testChoiceWeight = *std::max_element(eV.begin(), eV.end());
            if(testChoiceWeight < bestChoiceWeight) {
                bestChoice = *it;
                bestChoiceWeight = testChoiceWeight;
            }
        }
    }
    //Now that all the points are calculated, we construct a
    //cover tree for them.
    //since every element of each embedding vector ranges from 0 to 1, no
    //two elements will have distance greater than numDimensions.
    if(symmetric) {
        CoverTree<CoverTreePoint>& cTree =
            embedTreeMap.insert(make_pair(linkType,CoverTree<CoverTreePoint>(numDimensions+.1))).first->second;
        AtomEmbedding& aE = atomMaps[linkType];
        AtomEmbedding::const_iterator it = aE.begin();
        for(;it!=aE.end();++it) {
            cTree.insert(CoverTreePoint(it->first,it->second));
        }
    } else {
        std::pair<CoverTree<CoverTreePoint>, CoverTree<CoverTreePoint> >& cTrees
            = asymEmbedTreeMap.insert(make_pair(linkType,
                                            make_pair(CoverTree<CoverTreePoint>(numDimensions+.1), CoverTree<CoverTreePoint>(numDimensions+.1)))).first->second;
        std::pair<AtomEmbedding, AtomEmbedding>& aE = asymAtomMaps[linkType];
        AtomEmbedding& aE1 = aE.first;
        AtomEmbedding::const_iterator it = aE1.begin();
        for(;it!=aE1.end();++it) {
            cTrees.first.insert(CoverTreePoint(it->first,it->second));
        }
        AtomEmbedding& aE2 = aE.second;
        it = aE2.begin();
        for(;it!=aE2.end();++it) {
            cTrees.second.insert(CoverTreePoint(it->first,it->second));
        }
    }
    //logger().info("done embedding");
}

std::vector<double> DimEmbedModule::addNode(const Handle& h,
                                            const Type& linkType,
                                            AtomSpaceImpl* a) {    
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
    if(!isEmbedded(linkType)) {
        const char* tName = classserver().getTypeName(linkType).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    HandleSeq links = a->getIncoming(h);
    std::vector<double> newEmbedding (dimensionMap[linkType], 0.0);
    //The embedding for each coordinate is the max of
    //tv.strength*tv.confidence*embedding of every directly connected
    //node.
    //eg if our new node is connected to node A with embedding (.1,.2)
    //with link weight .3, and to node B with embedding (.4,.5)
    //with link weight .6, then the new node's embedding is the vector...
    //(max(.3*.1,.8*.4),max(.3*.2,.6*.5))    
    for(HandleSeq::iterator it=links.begin(); it<links.end(); ++it) {
        HandleSeq nodes = a->getOutgoing(*it);
        const TruthValue& linkTV = a->getTV(*it);        
        double weight = linkTV.getConfidence()*linkTV.getMean();
        for(HandleSeq::iterator it2=nodes.begin();it2<nodes.end(); ++it2) {
            if(*it2==h) continue;
            const std::vector<double>& embedding =
                getEmbedVector(*it2,linkType);
            //Alter our embedding whenever we find a higher weight path
            for(unsigned int i=0; i<embedding.size(); ++i) {
                if(weight*embedding[i]>newEmbedding[i]) {
                    newEmbedding[i]=weight*embedding[i];
                }
            }
        }
    }
    atomMaps[linkType][h] = newEmbedding;
    EmbedTreeMap::iterator treeMapIt = embedTreeMap.find(linkType);
    OC_ASSERT(treeMapIt!=embedTreeMap.end());
    CoverTree<CoverTreePoint>& cTree = treeMapIt->second;
    cTree.insert(CoverTreePoint(h,newEmbedding));
    return newEmbedding;
}

void DimEmbedModule::removeNode(const Handle& h,
                                const Type& linkType) {
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
    if(!isEmbedded(linkType)) {
        const char* tName = classserver().getTypeName(linkType).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }

    EmbedTreeMap::iterator treeMapIt = embedTreeMap.find(linkType);
    OC_ASSERT(treeMapIt!=embedTreeMap.end());
    CoverTree<CoverTreePoint>& cTree = treeMapIt->second;
    AtomEmbedding::iterator aEit = atomMaps[linkType].find(h);
    cTree.remove(CoverTreePoint(h,aEit->second));
    atomMaps[linkType].erase(aEit);
}

void DimEmbedModule::addLink(const Handle& h,
                             const Type& linkType,
                             AtomSpaceImpl* a) {
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
    if(!isEmbedded(linkType)) {
        const char* tName = classserver().getTypeName(linkType).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    bool symmetric = classserver().isA(linkType,UNORDERED_LINK);
    if(symmetric) symAddLink(h,linkType,a);
    else asymAddLink(h,linkType,a);
}
void DimEmbedModule::symAddLink(const Handle& h,
                                const Type& linkType,
                                AtomSpaceImpl* a) {
    EmbedTreeMap::iterator treeMapIt = embedTreeMap.find(linkType);    
    OC_ASSERT(treeMapIt!=embedTreeMap.end());
    CoverTree<CoverTreePoint>& cTree = treeMapIt->second;
    int dim = dimensionMap[linkType];
    AtomEmbedding& aE = atomMaps[linkType];
    const TruthValue& linkTV = a->getTV(h);        
    double weight = linkTV.getConfidence()*linkTV.getMean();
    HandleSeq nodes = a->getOutgoing(h);
    for(HandleSeq::iterator it=nodes.begin();it!=nodes.end();++it) {
        AtomEmbedding::iterator aEit = aE.find(*it);
        bool changed=false;
        for(HandleSeq::iterator it2=nodes.begin();it2!=nodes.end();++it2) {
            std::vector<double> vec = aE[*it2];
            for(int i=0; i<dim; ++i) {
                if((aEit->second)[i]<weight*vec[i]) {
                    if(!changed) {
                        changed=true;
                        cTree.remove(CoverTreePoint(aEit->first,aEit->second));
                    }
                    (aEit->second)[i]=weight*vec[i];
                }
            }
        }
        if(changed) cTree.insert(CoverTreePoint(aEit->first,aEit->second));
    }
}

void DimEmbedModule::asymAddLink(const Handle& h,
                                 const Type& linkType,
                                 AtomSpaceImpl* a) {
    AsymEmbedTreeMap::iterator treeMapIt = asymEmbedTreeMap.find(linkType);    
    OC_ASSERT(treeMapIt!=asymEmbedTreeMap.end());
    CoverTree<CoverTreePoint>& cTreeForw = treeMapIt->second.first;
    CoverTree<CoverTreePoint>& cTreeBackw = treeMapIt->second.second;
    int dim = dimensionMap[linkType];
    AtomEmbedding& aEForw = asymAtomMaps[linkType].first;
    AtomEmbedding& aEBackw = asymAtomMaps[linkType].second;
    const TruthValue& linkTV = a->getTV(h);        
    double weight = linkTV.getConfidence()*linkTV.getMean();
    HandleSeq nodes = a->getOutgoing(h);
    Handle source = nodes.front();
    HandleSeq::iterator it=nodes.begin();
    ++it;
    std::vector<double>& sourceVecForw = aEForw[source];
    const std::vector<double>& sourceVecBackw = aEBackw[source];
    bool sourceChanged=false;
    for(;it!=nodes.end();++it) {
        bool changed=false;
        std::vector<double>& vecBackw = aEBackw[*it];
        for(int i=0; i<dim; ++i) {
            if(vecBackw[i]<weight*sourceVecBackw[i]) {
                if(!changed) {
                    changed=true;
                    cTreeBackw.remove(CoverTreePoint(*it,vecBackw));
                }
                vecBackw[i]=weight*sourceVecBackw[i];
            }
        }
        if(changed) cTreeBackw.insert(CoverTreePoint(*it,vecBackw));
        const std::vector<double>& vecForw = aEForw[*it];
        for(int i=0; i<dim; ++i) {
            if(sourceVecForw[i]<weight*vecForw[i]) {
                if(!sourceChanged) {
                    sourceChanged=true;
                    cTreeForw.remove(CoverTreePoint(source,sourceVecForw));
                }
                sourceVecForw[i]=weight*vecForw[i];
            }
        }
    }
    if(sourceChanged) cTreeForw.insert(CoverTreePoint(source, sourceVecForw));
}

void DimEmbedModule::clearEmbedding(const Type& linkType){
    if(!classserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(linkType).c_str());
    
    HandleSeq pivots  = pivotsMap[linkType];
    for(HandleSeq::iterator it = pivots.begin(); it!=pivots.end(); ++it) {
        if(as->isValidHandle(*it)) as->decVLTI(*it);
    }
    atomMaps.erase(linkType);
    pivotsMap.erase(linkType);
    embedTreeMap.erase(linkType);
    dimensionMap.erase(linkType);
}

void DimEmbedModule::logAtomEmbedding(const Type& linkType) {
    AtomEmbedding atomEmbedding=atomMaps[linkType];
    const HandleSeq& pivots = getPivots(linkType);

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
    if(aEMit==atomMaps.end()) return false;
    else return true;
}

ClusterSeq DimEmbedModule::kMeansCluster(const Type& l, int numClusters, int npass, bool pivotWise) {
    if(!isEmbedded(l)) {
        const char* tName = classserver().getTypeName(l).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    int numDimensions=dimensionMap[l];
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
    int transpose;
    if(pivotWise) transpose=1;
    else transpose=0;
    //clusterid[i]==j will indicate that handle i belongs in cluster j
    ::kcluster(numClusters, numVectors, numDimensions, embedMatrix,
               mask, weight, transpose, npass, 'a', 'e', clusterid, &error, &ifound);

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
    const HandleSeq& pivots = getPivots(l);
    if(!pivotWise) {
        for(int i=0;i<numVectors;++i) {
            //clusterid[i] indicates which cluster handleArray[i] is in.
            int clustInd=clusterid[i];
            clusters[clustInd].first.push_back(handleArray[i]);
        }
    } else {
        for(int i=0;i<numDimensions;++i) {
            int clustInd=clusterid[i];
            clusters[clustInd].first.push_back(pivots[i]);
        }
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
    if(kPasses==-1) kPasses = (std::log(aE.size())/std::log(2))-1;

    typedef std::pair<double,std::pair<HandleSeq,vector<double> > > cPair;
    typedef std::multimap<double,std::pair<HandleSeq,vector<double> > >
        pQueue_t;
    pQueue_t clusters;
    //make a Priority Queue of (HandleSeq,vector<double>) pairs, where
    //we can easily extract those with the lowest value to discard if we
    //exceed maxClusters.
    int k = aE.size()/2;
    double c = std::pow(2,(std::log(k)/std::log(2))/kPasses);
    while(k>2) {
        ClusterSeq newClusts = kMeansCluster(l,k);
        for(ClusterSeq::iterator it = newClusts.begin();
            it!=newClusts.end();++it) {
            if(it->first.size()==1) continue;//ignore singleton clusters
            //if we still have room for more clusters and the cluster quality
            //is high enough, insert the cluster into the pQueue
            double quality = separation(it->first,l)*homogeneity(it->first,l);
            if(quality>threshold) {
                if( (int) clusters.size() < maxClusters) {
                    clusters.insert(cPair(quality,*it));
                } else {
                    //if there is no room, but our new cluster is better
                    //than the worst current cluster, replace the worst one
                    pQueue_t::iterator p_it = clusters.begin();
                    if(quality>p_it->first) {
                        clusters.erase(p_it);
                        clusters.insert(cPair(quality,*it));
                    }
                }
            }
        }
        k=k/c;
    }
    const HandleSeq& pivots = getPivots(l);
    const int& numDims = dimensionMap[l];
    //Make a new node for each cluster and connect it with InheritanceLinks.
    for(pQueue_t::iterator it = clusters.begin();it!=clusters.end();++it) {
        const HandleSeq& cluster = it->second.first;
        const std::vector<double>& centroid = it->second.second;
        Handle newNode = as->addPrefixedNode(CONCEPT_NODE, "cluster_");
        std::vector<double> strNumer(0,numDims);
        std::vector<double> strDenom(0,numDims);
        //Connect newNode to each handle in its cluster and each pivot
        for(HandleSeq::const_iterator it2=cluster.begin();
            it2!=cluster.end();++it2) {
            const std::vector<double>& embedVec = getEmbedVector(*it2,l);
            double dist = euclidDist(centroid,embedVec);
            //TODO: we should do some normalizing of this probably...
            double strength = sqrt(std::pow(2.0, -dist));
            SimpleTruthValue tv(strength,
                                SimpleTruthValue::confidenceToCount(strength));
            as->addLink(INHERITANCE_LINK, *it2, newNode, tv);
            for(int i=0; i<numDims; ++i) {
                strNumer[i]+=strength*embedVec[i];
                strDenom[i]+=strength;
            }
        }
        for(int i=0; i<numDims; ++i) {
            //the link between a clusterNode and an attribute (pivot) is
            //a weighted average of the cluster's members' links to the pivot
            double attrStrength = sqrt(strNumer[i]/strDenom[i]);
            SimpleTruthValue tv(attrStrength,
                                SimpleTruthValue::confidenceToCount(attrStrength));
            as->addLink(l, newNode, pivots[i], tv);
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

Handle DimEmbedModule::blendNodes(const Handle& n1,
                                  const Handle& n2, const Type& l) {
    if(!classserver().isLink(l))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            classserver().getTypeName(l).c_str());
    if(!as->isNode(n1) || !as->isNode(n2))
        throw InvalidParamException(TRACE_INFO,
                                    "blendNodes requires two nodes.");
    if(!isEmbedded(l)) {
        const char* tName = classserver().getTypeName(l).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    const HandleSeq& pivots = getPivots(l);
    const unsigned int numDims = (unsigned int) dimensionMap[l];
    const std::vector<double>& embedVec1 = getEmbedVector(n1,l);
    const std::vector<double>& embedVec2 = getEmbedVector(n2,l);
    OC_ASSERT(numDims==embedVec1.size() &&
              numDims==embedVec2.size() && numDims==pivots.size());
    std::vector<double> newVec(embedVec1.begin(), embedVec1.end());

    EmbedTreeMap::iterator treeMapIt = embedTreeMap.find(l);
    OC_ASSERT(treeMapIt!=embedTreeMap.end());
    CoverTree<CoverTreePoint>& cTree = treeMapIt->second;
    //For each pivot, see whether replacing embedVec1's embedding with
    //embedVec2's will make newVec farther from any existing point. Replace
    //it if so.
    for(unsigned int i=0; i<numDims; i++) {
        CoverTreePoint p1(Handle::UNDEFINED,newVec);
        newVec[i]=embedVec2[i];
        CoverTreePoint p2(Handle::UNDEFINED,newVec);
        double dist1 = p1.distance(cTree.kNearestNeighbors(p1,1)[0]);
        double dist2 = p2.distance(cTree.kNearestNeighbors(p2,1)[0]);
        if(dist1>dist2) newVec[i]=embedVec2[i];
    }
    std::string prefix("blend_"+as->getName(n1)+"_"+as->getName(n2)+"_");
    Handle newNode = as->addPrefixedNode(as->getType(n1), prefix);
    
    for(unsigned int i=0; i<numDims; i++) {
        double strength = sqrt(newVec[i]);
        SimpleTruthValue tv(strength,
                            SimpleTruthValue::confidenceToCount(strength));
        as->addLink(l, newNode, pivots[i], tv);
    }
    return newNode;
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
    return euclidDist(getEmbedVector(h1,l), getEmbedVector(h2,l));
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
    if(a->isNode(h)) {
        //for each link type embedding that exists, add the node
        for(it=atomMaps.begin();it!=atomMaps.end();it++) {
            addNode(h,it->first,a);
        }
    }
    else {//h is a link
        for(it=atomMaps.begin();it!=atomMaps.end();it++) {
            //if the new link is a subtype of an existing embedding, add it
            if(classserver().isA(a->getType(h),it->first))
                addLink(h,it->first,a);
        }
    }
}

void DimEmbedModule::handleRemoveSignal(AtomSpaceImpl* a, Handle h) {
    if(a->isNode(h)) {
        //for each link type embedding that exists, remove the node
        AtomEmbedMap::iterator it;
        for(it=atomMaps.begin();it!=atomMaps.end();it++) {
            removeNode(h,it->first);
        }
    }
}
