/*
 * opencog/learning/dimensionalembedding/DimEmbedModule.cc
 *
 * Copyright (C) 2010 by OpenCog Foundation
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
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <string>
#include <utility>

#include <opencog/atomspaceutils/AtomSpaceUtils.h>
#include <opencog/atoms/proto/NameServer.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/truthvalue/SimpleTruthValue.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

extern "C" {
#include <opencog/util/cluster.h>
}

#include "DimEmbedModule.h"

using namespace opencog;
using namespace std::placeholders;

typedef std::vector<std::pair<HandleSeq,std::vector<double> > >
    ClusterSeq; //the vector of doubles is the centroid of the cluster

DECLARE_MODULE(DimEmbedModule)

DimEmbedModule::DimEmbedModule(CogServer& cs) : Module(cs)
{
    logger().info("[DimEmbedModule] constructor");
    as = &_cogserver.getAtomSpace();
    _bank = &attentionbank(as);

    addedAtomConnection = as->
        atomAddedSignal().connect(std::bind(&DimEmbedModule::handleAddSignal, this, _1));
    removedAtomConnection = as->
        atomRemovedSignal().connect(std::bind(&DimEmbedModule::atomRemoveSignal, this, _1));
    tvChangedConnection = as->
        TVChangedSignal().connect(std::bind(&DimEmbedModule::TVChangedSignal, this, _1, _2, _3));
}

DimEmbedModule::~DimEmbedModule()
{
    logger().info("[DimEmbedModule] destructor");
    as->atomAddedSignal().disconnect(addedAtomConnection);
    as->atomRemovedSignal().disconnect(removedAtomConnection);
    as->TVChangedSignal().disconnect(tvChangedConnection);
}

void DimEmbedModule::init()
{
    logger().info("[DimEmbedModule] init");
    this->as = &_cogserver.getAtomSpace();
    addedAtomConnection = as->
        atomAddedSignal().connect(std::bind(&DimEmbedModule::handleAddSignal, this, _1));
    removedAtomConnection = as->
        atomRemovedSignal().connect(std::bind(&DimEmbedModule::atomRemoveSignal, this, _1));
    tvChangedConnection = as->
        TVChangedSignal().connect(std::bind(&DimEmbedModule::TVChangedSignal, this, _1, _2, _3));
#ifdef HAVE_GUILE
    //Functions available to scheme shell
    define_scheme_primitive("embedSpace",
                            &DimEmbedModule::embedAtomSpace,
                            this);
    define_scheme_primitive("logEmbedding",
                            &DimEmbedModule::logAtomEmbedding,
                            this);
//    define_scheme_primitive("euclidDist",
//                            &DimEmbedModule::euclidDist,
//                            this);
    define_scheme_primitive("kNN",
                            &DimEmbedModule::kNearestNeighbors,
                            this);
    define_scheme_primitive("kMeansCluster",
                            &DimEmbedModule::addKMeansClusters,
                            this);
#endif
}

const std::vector<double>& DimEmbedModule::getEmbedVector(Handle h,
                                                          Type l,
                                                          bool fanin) const
{
    if (!nameserver().isLink(l))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(l).c_str());

    if (!isEmbedded(l)) {
        const char* tName = nameserver().getTypeName(l).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }

    bool symmetric = nameserver().isA(l,UNORDERED_LINK);
    if (symmetric) {
        const AtomEmbedding& aE = (atomMaps.find(l))->second;
        //AtomEmbedding::const_iterator aEit = aE.find(h);
        //return aEit->second;
        return aE.find(h)->second;
    } else {
        const std::pair<AtomEmbedding, AtomEmbedding>& aEPair =
            (asymAtomMaps.find(l))->second;
        if (fanin) {
            //AtomEmbedding::const_iterator aEit = aEPair.second.find(h);
            //return aEit->second;
            return aEPair.second.find(h)->second;
        } else {
            //AtomEmbedding::const_iterator aEit = aE.find(h);
            //return aEit->second;
            return aEPair.first.find(h)->second;
        }
    }
}

HandleSeq& DimEmbedModule::getPivots(Type l, bool fanin)
{
    if (!nameserver().isLink(l))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(l).c_str());
    if (!isEmbedded(l)) {
        const char* tName = nameserver().getTypeName(l).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    bool symmetric = nameserver().isA(l,UNORDERED_LINK);
    if (symmetric) {
        return pivotsMap.find(l)->second;
    } else {
        if (fanin) return asymPivotsMap.find(l)->second.second;
        else return asymPivotsMap.find(l)->second.first;
    }
}

HandleSeq DimEmbedModule::kNearestNeighbors(Handle h, Type l, int k, bool fanin)
{
    if (!nameserver().isLink(l))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(l).c_str());
    //logger().info("%d nearest neighbours start", k);
    if (!isEmbedded(l)) {
        const char* tName = nameserver().getTypeName(l).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    bool symmetric = nameserver().isA(l,UNORDERED_LINK);

    std::vector<CoverTreePoint> points;
    if (symmetric) {
        EmbedTreeMap::const_iterator it = embedTreeMap.find(l);
        points =
            it->second.k_nearest_neighbors(CoverTreePoint(h,atomMaps[l][h]),k);
    } else {
        AsymEmbedTreeMap::const_iterator it = asymEmbedTreeMap.find(l);
        if (fanin) {
            points =
                it->second.second.
                k_nearest_neighbors(CoverTreePoint(h,asymAtomMaps[l].second[h]),k);
        }
        else {
            points =
                it->second.first.
                k_nearest_neighbors(CoverTreePoint(h,asymAtomMaps[l].first[h]),k);
        }
    }
    HandleSeq results;
    std::vector<CoverTreePoint>::const_iterator it;
    for (it=points.begin(); it!=points.end(); ++it) {
        results.push_back(it->getHandle());
    }
    return results;
}

static bool is_source(const Handle& source, const Handle& link)
{
    LinkPtr lptr(LinkCast(link));
    // On ordered links, only the first position in the outgoing set
    // is a source of this link. So, if the handle given is equal to
    // the first position, true is returned.
    Arity arity = lptr->get_arity();
    if (nameserver().isA(lptr->get_type(), ORDERED_LINK)) {
        return arity > 0 and lptr->getOutgoingAtom(0) == source;
    } else if (nameserver().isA(lptr->get_type(), UNORDERED_LINK)) {
        // If the link is unordered, the outgoing set is scanned;
        // return true if any position is equal to the source.
        for (const Handle& h : lptr->getOutgoingSet())
            if (h == source) return true;
        return false;
    }
    return false;
}

void DimEmbedModule::addPivot(Handle h, Type linkType, bool fanin)
{
    if (!nameserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(linkType).c_str());
    bool symmetric = nameserver().isA(linkType,UNORDERED_LINK);
    if (!fanin) _bank->inc_vlti(h); //We don't want pivot atoms to be forgotten...
    HandleSeq nodes;
    as->get_handles_by_type(std::back_inserter(nodes), NODE, true);

    std::map<Handle, double> distMap;

    typedef std::multimap<double,Handle> pQueue_t;
    pQueue_t pQueue;
    for (HandleSeq::iterator it=nodes.begin(); it!=nodes.end(); ++it){
        if (*it==h) {
            pQueue.insert(std::pair<double, Handle>(1,*it));
            distMap[*it]=1;
        } else {
            pQueue.insert(std::pair<double, Handle>(0,*it));
            distMap[*it]=0;
        }
    }
    if (symmetric) {
        pivotsMap[linkType].push_back(h);
    } else {
        if (fanin) asymPivotsMap[linkType].second.push_back(h);
        else asymPivotsMap[linkType].first.push_back(h);
    }
    while(!pQueue.empty()) {
        pQueue_t::reverse_iterator p_it = pQueue.rbegin();
        Handle u = p_it->second;//extract max (highest weight)

        /*std::cout << "U=" << u << " distmap={";
        for (std::map<Handle, double>::iterator it = distMap.begin();
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
        HandleSeq newLinks;
        u->getIncomingSet(back_inserter(newLinks));
        for (HandleSeq::iterator it=newLinks.begin(); it!=newLinks.end(); ++it){
            //ignore links that aren't a subtype of linkType
            if (!nameserver().isA((*it)->get_type(), linkType)) continue;
            TruthValuePtr linkTV = (*it)->getTruthValue();
            HandleSeq newNodes = (*it)->getOutgoingSet();
            HandleSeq::iterator it2=newNodes.begin();
            //if !fanin, we're following the "outward" links, so it's only a
            //valid link if u is the source
            if (!symmetric && !fanin && !is_source(u,*it)) continue;
            if (!symmetric && fanin) {
                if (is_source(u,*it)) {
                    continue;
                }
                else {
                    newNodes.clear();
                    newNodes.push_back(*it2);
                    it2=newNodes.begin();
                }
            }
            for (;it2!=newNodes.end(); ++it2) {
                if (!(*it2)->is_node()) continue;
                double alt =
                    distMap[u] * linkTV->get_mean() * linkTV->get_confidence();
                double oldDist=distMap[*it2];
                //If we've found a better (higher weight) path, update distMap
                if (alt>oldDist) {
                    pQueue_t::iterator it3;
                    std::pair<pQueue_t::iterator, pQueue_t::iterator>
                        itPair = pQueue.equal_range(oldDist);
                    for (it3=itPair.first;it3!=itPair.second;++it3) {
                        if (it3->second==*it2) {pQueue.erase(it3); break;}
                    }
                    pQueue.insert(std::pair<double, Handle>(alt,*it2));
                    distMap[*it2]=alt;
                }
            }
        }
    }
    for (std::map<Handle, double>::iterator it = distMap.begin();
            it != distMap.end(); ++it) {
        if (symmetric) {
            atomMaps[linkType][it->first].push_back(it->second);
        } else {
            if (fanin) asymAtomMaps[linkType].second[it->first].push_back(it->second);
            else asymAtomMaps[linkType].first[it->first].push_back(it->second);
        }
    }
}

Handle DimEmbedModule::pickPivot(Type linkType, HandleSeq& nodes, bool fanin)
{
    bool symmetric = nameserver().isA(linkType,UNORDERED_LINK);

    HandleSeq& pivots = getPivots(linkType,fanin);
    Handle bestChoice = nodes.back();
    if (pivots.empty()) return bestChoice;
    logger().info("Pivot %d picked", pivots.size());
    double bestChoiceWeight = 1;
    //pick the next pivot to maximize its distance from its closest pivot
    //(maximizing distance = minimizing path weight)
    for (HandleSeq::iterator it=nodes.begin(); it!=nodes.end(); ++it) {
        std::vector<double> eV;
        if (symmetric) {
            eV = atomMaps[linkType][*it];
        } else {
            if (fanin) eV = asymAtomMaps[linkType].second[*it];
            else eV = asymAtomMaps[linkType].first[*it];
        }
        double testChoiceWeight = *std::max_element(eV.begin(), eV.end());
        if (testChoiceWeight < bestChoiceWeight) {
            bestChoice = *it;
            bestChoiceWeight = testChoiceWeight;
        }
    }
    return bestChoice;
}

void DimEmbedModule::embedAtomSpace(Type linkType,
                                    int _numDimensions)
{
    if (!nameserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(linkType).c_str());
    //logger().info("starting embedding");
    clearEmbedding(linkType);
    bool symmetric = nameserver().isA(linkType,UNORDERED_LINK);
    // Scheme wrapper doesn't deal with unsigned ints, so double check it's not
    // negative, or zero for that matter
    int numDimensions = 5;
    if (_numDimensions > 0) numDimensions = _numDimensions;

    bool fanin=false;
    dimensionMap[linkType]=numDimensions;
    //const HandleSeq& pivots = getPivots(linkType);
    HandleSeq nodes;//candidates for new pivots
    as->get_handles_by_type(std::back_inserter(nodes), NODE, true);
    if (nodes.empty()) return;
    if (nodes.size() < (size_t) numDimensions) numDimensions = nodes.size();

    for (int i=0; i<numDimensions; ++i) {
        Handle newPivot;
        if (i!=0) newPivot = pickPivot(linkType,nodes,fanin);
        else newPivot = nodes.back();
        nodes.erase(std::find(nodes.begin(), nodes.end(), newPivot));
        addPivot(newPivot, linkType, fanin);
        if (!symmetric && !fanin && (nodes.empty() || i==numDimensions-1)) {
            fanin=true;
            nodes.clear();
            as->get_handles_by_type(std::back_inserter(nodes), NODE, true);
            i=-1;
        }

    }
    //Now that all the points are calculated, we construct a
    //cover tree for them.
    //since every element of each embedding vector ranges from 0 to 1, no
    //two elements will have distance greater than numDimensions.
    if (symmetric) {
        CoverTree<CoverTreePoint>& cTree =
            embedTreeMap.insert(std::make_pair(linkType,CoverTree<CoverTreePoint>(numDimensions+.1))).first->second;
        AtomEmbedding& aE = atomMaps[linkType];
        AtomEmbedding::const_iterator it = aE.begin();
        for (;it!=aE.end();++it) {
            cTree.insert(CoverTreePoint(it->first,it->second));
        }
    } else {
        std::pair<CoverTree<CoverTreePoint>, CoverTree<CoverTreePoint> >& cTrees
            = asymEmbedTreeMap.insert(std::make_pair(linkType,
                                            std::make_pair(CoverTree<CoverTreePoint>(numDimensions+.1), CoverTree<CoverTreePoint>(numDimensions+.1)))).first->second;
        std::pair<AtomEmbedding, AtomEmbedding>& aE = asymAtomMaps[linkType];
        const AtomEmbedding& aE1 = aE.first;
        AtomEmbedding::const_iterator it = aE1.begin();
        for (;it!=aE1.end();++it) {
            cTrees.first.insert(CoverTreePoint(it->first,it->second));
        }
        const AtomEmbedding& aE2 = aE.second;
        it = aE2.begin();
        for (;it!=aE2.end();++it) {
            cTrees.second.insert(CoverTreePoint(it->first,it->second));
        }
    }
    //logger().info("done embedding");
}

std::vector<double> DimEmbedModule::addNode(Handle h,
                                            Type linkType)
{
    if (!nameserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %d \"%s\"",
            linkType,
            nameserver().getTypeName(linkType).c_str());
    if (!isEmbedded(linkType)) {
        const char* tName = nameserver().getTypeName(linkType).c_str();
        logger().error("No embedding exists for type \"%s\"", tName);
        throw std::string("No embedding exists for type \"%s\"", tName);
    }
    bool symmetric = nameserver().isA(linkType,UNORDERED_LINK);
    std::vector<double> newEmbedding (dimensionMap[linkType], 0.0);

    /* Since addNode is only called just as a new node is added, the new node
       must not have any links yet, so this code is probably not needed...
    HandleSeq links = a->getIncoming(h);
    //The embedding for each coordinate is the max of
    //tv.strength*tv.confidence*embedding of every directly connected
    //node.
    //eg if our new node is connected to node A with embedding (.1,.2)
    //with link weight .3, and to node B with embedding (.4,.5)
    //with link weight .6, then the new node's embedding is the vector...
    //(max(.3*.1,.8*.4),max(.3*.2,.6*.5))
    for (HandleSeq::iterator it=links.begin(); it<links.end(); ++it) {
        HandleSeq nodes = a->getOutgoing(*it);
        const TruthValue& linkTV = a->getTV(*it);
        double weight = linkTV.get_confidence()*linkTV.get_mean();
        for (HandleSeq::iterator it2=nodes.begin();it2<nodes.end(); ++it2) {
            if (*it2==h) continue;
            const std::vector<double>& embedding =
                getEmbedVector(*it2,linkType);
            //Alter our embedding whenever we find a higher weight path
            for (unsigned int i=0; i<embedding.size(); ++i) {
                if (weight*embedding[i]>newEmbedding[i]) {
                    newEmbedding[i]=weight*embedding[i];
                }
            }
        }
    }
    */
    if (symmetric) {
        atomMaps[linkType][h] = newEmbedding;
        EmbedTreeMap::iterator treeMapIt = embedTreeMap.find(linkType);
        OC_ASSERT(treeMapIt!=embedTreeMap.end());
        CoverTree<CoverTreePoint>& cTree = treeMapIt->second;
        cTree.insert(CoverTreePoint(h,newEmbedding));
    } else {
        asymAtomMaps[linkType].first[h] = newEmbedding;
        asymAtomMaps[linkType].second[h] = newEmbedding;
        AsymEmbedTreeMap::iterator treeMapIt = asymEmbedTreeMap.find(linkType);
        OC_ASSERT(treeMapIt!=asymEmbedTreeMap.end());
        CoverTree<CoverTreePoint>& cTree1 = treeMapIt->second.first;
        cTree1.insert(CoverTreePoint(h,newEmbedding));
        CoverTree<CoverTreePoint>& cTree2 = treeMapIt->second.second;
        cTree2.insert(CoverTreePoint(h,newEmbedding));
    }
    return newEmbedding;
}

void DimEmbedModule::removeNode(Handle h,
                                Type linkType)
{
    if (!nameserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(linkType).c_str());
    if (!isEmbedded(linkType)) {
        const char* tName = nameserver().getTypeName(linkType).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    bool symmetric = nameserver().isA(linkType,UNORDERED_LINK);
    if (symmetric) {
        EmbedTreeMap::iterator treeMapIt = embedTreeMap.find(linkType);
        OC_ASSERT(treeMapIt!=embedTreeMap.end());
        CoverTree<CoverTreePoint>& cTree = treeMapIt->second;
        AtomEmbedding::iterator aEit = atomMaps[linkType].find(h);
        cTree.remove(CoverTreePoint(h,aEit->second));
        atomMaps[linkType].erase(aEit);
    } else {
        AsymEmbedTreeMap::iterator treeMapIt = asymEmbedTreeMap.find(linkType);
        OC_ASSERT(treeMapIt!=asymEmbedTreeMap.end());
        CoverTree<CoverTreePoint>& cTree1 = treeMapIt->second.first;
        CoverTree<CoverTreePoint>& cTree2 = treeMapIt->second.second;
        AtomEmbedding::iterator aEit = asymAtomMaps[linkType].first.find(h);
        cTree1.remove(CoverTreePoint(h,aEit->second));
        asymAtomMaps[linkType].first.erase(aEit);
        aEit = asymAtomMaps[linkType].second.find(h);
        cTree2.remove(CoverTreePoint(h,aEit->second));
        asymAtomMaps[linkType].second.erase(aEit);
    }
}

void DimEmbedModule::addLink(Handle h,
                             Type linkType)
{
    if (!nameserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(linkType).c_str());
    if (!isEmbedded(linkType)) {
        const char* tName = nameserver().getTypeName(linkType).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    bool symmetric = nameserver().isA(linkType, UNORDERED_LINK);
    if (symmetric) symAddLink(h, linkType);
    else asymAddLink(h, linkType);
}

void DimEmbedModule::symAddLink(Handle h, Type linkType)
{
    EmbedTreeMap::iterator treeMapIt = embedTreeMap.find(linkType);
    OC_ASSERT(treeMapIt!=embedTreeMap.end());
    CoverTree<CoverTreePoint>& cTree = treeMapIt->second;
    int dim = dimensionMap[linkType];
    AtomEmbedding& aE = atomMaps[linkType];
    TruthValuePtr linkTV = h->getTruthValue();
    double weight = linkTV->get_confidence() * linkTV->get_mean();
    HandleSeq nodes;
    if (LinkCast(h)) nodes = LinkCast(h)->getOutgoingSet();
    for (HandleSeq::iterator it=nodes.begin();it!=nodes.end();++it) {
        AtomEmbedding::iterator aEit = aE.find(*it);
        bool changed=false;
        for (HandleSeq::iterator it2=nodes.begin();it2!=nodes.end();++it2) {
            std::vector<double> vec = aE[*it2];
            for (int i=0; i<dim; ++i) {
                if ((aEit->second)[i]<weight*vec[i]) {
                    if (!changed) {
                        changed=true;
                        cTree.remove(CoverTreePoint(aEit->first,aEit->second));
                    }
                    (aEit->second)[i]=weight*vec[i];
                }
            }
        }
        if (changed) cTree.insert(CoverTreePoint(aEit->first,aEit->second));
    }
}

void DimEmbedModule::asymAddLink(Handle h, Type linkType)
{
    AsymEmbedTreeMap::iterator treeMapIt = asymEmbedTreeMap.find(linkType);
    OC_ASSERT(treeMapIt!=asymEmbedTreeMap.end());
    CoverTree<CoverTreePoint>& cTreeForw = treeMapIt->second.first;
    CoverTree<CoverTreePoint>& cTreeBackw = treeMapIt->second.second;
    int dim = dimensionMap[linkType];
    AtomEmbedding& aEForw = asymAtomMaps[linkType].first;
    AtomEmbedding& aEBackw = asymAtomMaps[linkType].second;
    TruthValuePtr linkTV = h->getTruthValue();
    double weight = linkTV->get_confidence() * linkTV->get_mean();
    HandleSeq nodes;
    if (LinkCast(h)) nodes = LinkCast(h)->getOutgoingSet();
    Handle source = nodes.front();
    HandleSeq::iterator it = nodes.begin();
    ++it;
    std::vector<double>& sourceVecForw = aEForw[source];
    const std::vector<double>& sourceVecBackw = aEBackw[source];
    bool sourceChanged=false;
    for (;it!=nodes.end();++it) {
        bool changed=false;
        std::vector<double>& vecBackw = aEBackw[*it];
        for (int i=0; i<dim; ++i) {
            if (vecBackw[i]<weight*sourceVecBackw[i]) {
                if (!changed) {
                    changed=true;
                    cTreeBackw.remove(CoverTreePoint(*it,vecBackw));
                }
                vecBackw[i]=weight*sourceVecBackw[i];
            }
        }
        if (changed) cTreeBackw.insert(CoverTreePoint(*it,vecBackw));
        const std::vector<double>& vecForw = aEForw[*it];
        for (int i=0; i<dim; ++i) {
            if (sourceVecForw[i]<weight*vecForw[i]) {
                if (!sourceChanged) {
                    sourceChanged=true;
                    cTreeForw.remove(CoverTreePoint(source,sourceVecForw));
                }
                sourceVecForw[i]=weight*vecForw[i];
            }
        }
    }
    if (sourceChanged) cTreeForw.insert(CoverTreePoint(source, sourceVecForw));
}

void DimEmbedModule::clearEmbedding(Type linkType)
{
    if (!nameserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(linkType).c_str());
    bool symmetric = nameserver().isA(linkType,UNORDERED_LINK);

    HandleSeq pivots  = pivotsMap[linkType];
    for (HandleSeq::iterator it = pivots.begin(); it!=pivots.end(); ++it) {
        if (as->is_valid_handle(*it)) _bank->dec_vlti(*it);
    }
    if (symmetric) {
        atomMaps.erase(linkType);
        embedTreeMap.erase(linkType);
    } else {
        asymAtomMaps.erase(linkType);
        asymEmbedTreeMap.erase(linkType);
    }
    pivotsMap.erase(linkType);
    dimensionMap.erase(linkType);
}

void DimEmbedModule::logAtomEmbedding(Type linkType)
{
    bool symmetric = nameserver().isA(linkType,UNORDERED_LINK);
    AtomEmbedding atomEmbedding;
    if (symmetric) atomEmbedding=atomMaps[linkType];
    else atomEmbedding=asymAtomMaps[linkType].first;
    const HandleSeq& pivots = getPivots(linkType);

    std::ostringstream oss;

    oss << "PIVOTS:" << std::endl;
    for (HandleSeq::const_iterator it=pivots.begin(); it!=pivots.end(); ++it){
        if (as->is_valid_handle(*it)) {
            oss << (*it)->to_short_string() << std::endl;
        } else {
            oss << "[PIVOT'S BEEN DELETED]" << std::endl;
        }
    }
    oss << "Node Embeddings:" << std::endl;
    AtomEmbedding::const_iterator it;
    for (it=atomEmbedding.begin(); it!=atomEmbedding.end(); ++it){
        if (as->is_valid_handle(it->first)) {
            oss << (it->first)->to_short_string() << " : (";
        } else {
            oss << "[NODE'S BEEN DELETED H=" << it->first << "] : (";
        }
        const std::vector<double>& embedvector = it->second;
        for (std::vector<double>::const_iterator it2=embedvector.begin();
            it2!=embedvector.end();
            ++it2){
            oss << *it2 << " ";
        }
        oss << ")" << std::endl;
    }
    logger().info(oss.str());
    return;
}

void DimEmbedModule::printEmbedding()
{
    std::ostringstream oss;
    AtomEmbedMap::const_iterator mit = atomMaps.begin();
    oss << "Node Embeddings" << std::endl;
    for (; mit != atomMaps.end(); ++mit) {
        oss << "=== for type" << nameserver().getTypeName(mit->first).c_str() << std::endl;
        AtomEmbedding atomEmbedding=mit->second;
        AtomEmbedding::const_iterator it;
        for (it=atomEmbedding.begin(); it!=atomEmbedding.end(); ++it){
            if (as->is_valid_handle(it->first)) {
                oss << (it->first)->to_short_string() << " : (";
            } else {
                oss << "[NODE'S BEEN DELETED. handle=";
                oss << it->first.value() << "] : (";
            }
            const std::vector<double>& embedVector = it->second;
            for (std::vector<double>::const_iterator it2=embedVector.begin();
                it2!=embedVector.end();
                ++it2){
                oss << *it2 << " ";
            }
            oss << ")" << std::endl;
        }
    }
    std::cout << oss.str();
}

bool DimEmbedModule::isEmbedded(Type linkType) const
{
    if (!nameserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(linkType).c_str());
    //See if atomMaps holds an embedding for linkType
    AtomEmbedMap::const_iterator aEMit = atomMaps.find(linkType);
    if (aEMit!=atomMaps.end()) return true;
    AsymAtomEmbedMap::const_iterator aaEMit = asymAtomMaps.find(linkType);
    if (aaEMit!=asymAtomMaps.end()) return true;
    return false;
}

ClusterSeq DimEmbedModule::kMeansCluster(Type l, int numClusters, int npass, bool pivotWise)
{
    if (!isEmbedded(l)) {
        const char* tName = nameserver().getTypeName(l).c_str();
        logger().error("No embedding exists for type %s", tName);
        throw std::string("No embedding exists for type %s", tName);
    }
    int numDimensions=dimensionMap[l];
    const AtomEmbedding& aE = atomMaps[l];
    int numVectors=aE.size();
    if (numVectors<numClusters) {
        logger().error("Cannot make more clusters than there are nodes");
        throw std::string("Cannot make more clusters than there are nodes");
    }
    //create the required matrices for the clustering function (which
    //takes double** as an argument...)
    double* embedding = new double[numDimensions*numVectors];
    double** embedMatrix = new double*[numVectors];
    int* maskArray = new int[numDimensions*numVectors];
    int** mask = new int*[numVectors];
    for (int i=0;i<numVectors;++i) {
        embedMatrix[i] = embedding + numDimensions*i;
        mask[i] = maskArray + numDimensions*i;
    }
    Handle* handleArray = new Handle[numVectors];
    AtomEmbedding::const_iterator aEit=aE.begin();
    int i=0;
    int j;
    //add the values to the embeddingmatrix...
    for (;aEit!=aE.end();++aEit) {
        handleArray[i]=aEit->first;
        const std::vector<double>& embedding = aEit->second;
        std::vector<double>::const_iterator vit=embedding.begin();
        j=0;
        for (;vit!=embedding.end();++vit) {
            embedMatrix[i][j]=*vit;
            mask[i][j]=1;
            j++;
        }
        i++;
    }
    double* weight = new double[numDimensions];
    for (int i=0;i<numDimensions;++i) {weight[i]=1;}

    int* clusterid = new int[numVectors]; //stores the result of clustering
    double error;
    int ifound;
    for (int i=0;i<numVectors;++i) {
        for (int j=0;j<numDimensions;++j) {
            mask[i][j]=1;
        }
    }
    int transpose;
    if (pivotWise) transpose=1;
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
    for (int i=0;i<numClusters;++i) {
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
    if (!pivotWise) {
        for (int i=0;i<numVectors;++i) {
            //clusterid[i] indicates which cluster handleArray[i] is in.
            int clustInd=clusterid[i];
            clusters[clustInd].first.push_back(handleArray[i]);
        }
    } else {
        for (int i=0;i<numDimensions;++i) {
            int clustInd=clusterid[i];
            clusters[clustInd].first.push_back(pivots[i]);
        }
    }
    for (int i=0;i<numClusters;++i) {
        std::vector<double> centroid(centroidMatrix[i],
                                     centroidMatrix[i]+numDimensions);
        clusters[i].second=centroid;
    }

    //for (HandleSeqSeq::const_iterator it=clusters.begin();
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

void DimEmbedModule::addKMeansClusters(Type l, int maxClusters,
                                       double threshold, int kPasses)
{
    const AtomEmbedding& aE = atomMaps[l];
    if (kPasses==-1) kPasses = (std::log(aE.size())/std::log(2))-1;

    typedef std::pair<double,std::pair<HandleSeq,std::vector<double> > > cPair;
    typedef std::multimap<double,std::pair<HandleSeq,std::vector<double> > >
        pQueue_t;
    pQueue_t clusters;
    //make a Priority Queue of (HandleSeq,std::vector<double>) pairs, where
    //we can easily extract those with the lowest value to discard if we
    //exceed maxClusters.
    int k = aE.size()/2;
    double c = std::pow(2,(std::log(k)/std::log(2))/kPasses);
    while(k>2) {
        ClusterSeq newClusts = kMeansCluster(l,k);
        for (ClusterSeq::iterator it = newClusts.begin();
            it!=newClusts.end();++it) {
            if (it->first.size()==1) continue;//ignore singleton clusters
            //if we still have room for more clusters and the cluster quality
            //is high enough, insert the cluster into the pQueue
            double quality = separation(it->first,l)*homogeneity(it->first,l);
            if (quality>threshold) {
                if ( (int) clusters.size() < maxClusters) {
                    clusters.insert(cPair(quality,*it));
                } else {
                    //if there is no room, but our new cluster is better
                    //than the worst current cluster, replace the worst one
                    pQueue_t::iterator p_it = clusters.begin();
                    if (quality>p_it->first) {
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
    for (pQueue_t::iterator it = clusters.begin();it!=clusters.end();++it) {
        const HandleSeq& cluster = it->second.first;
        const std::vector<double>& centroid = it->second.second;
        Handle newNode = add_prefixed_node(*as, CONCEPT_NODE, "cluster_");
        std::vector<double> strNumer(0,numDims);
        std::vector<double> strDenom(0,numDims);
        //Connect newNode to each handle in its cluster and each pivot
        for (HandleSeq::const_iterator it2=cluster.begin();
            it2!=cluster.end();++it2) {
            const std::vector<double>& embedVec = getEmbedVector(*it2,l);
            double dist = euclidDist(centroid,embedVec);
            //TODO: we should do some normalizing of this probably...
            double strength = sqrt(std::pow(2.0, -dist));
            TruthValuePtr tv(SimpleTruthValue::createTV(strength, strength));
            Handle hi = as->add_link(INHERITANCE_LINK, *it2, newNode);
            hi->setTruthValue(hi->getTruthValue()->merge(tv));

            for (int i=0; i<numDims; ++i) {
                strNumer[i] += strength * embedVec[i];
                strDenom[i] += strength;
            }
        }
        for (int i=0; i<numDims; ++i) {
            //the link between a clusterNode and an attribute (pivot) is
            //a weighted average of the cluster's members' links to the pivot
            double attrStrength = sqrt(strNumer[i]/strDenom[i]);
            TruthValuePtr tv(SimpleTruthValue::createTV(attrStrength, attrStrength));
            Handle hi = as->add_link(l, newNode, pivots[i]);
            hi->setTruthValue(hi->getTruthValue()->merge(tv));
        }
    }
}

double DimEmbedModule::homogeneity(const HandleSeq& cluster,
                                   Type linkType) const
{
    if (!nameserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(linkType).c_str());
    OC_ASSERT(cluster.size()>1);

    double average=0;
    for (HandleSeq::const_iterator it=cluster.begin();it!=cluster.end();++it) {
        double minDist=DBL_MAX;
        //find the distance to nearest clustermate
        const std::vector<double>& embedding = getEmbedVector(*it,linkType);
        for (HandleSeq::const_iterator it2=cluster.begin();
                                      it2!=cluster.end();++it2) {
            if (*it==*it2) continue;
            double dist=euclidDist(embedding,getEmbedVector(*it2,linkType));
            if (dist<minDist) minDist=dist;
        }
        average+=minDist;
    }
    average = average/cluster.size();
    return 1.0/(1.0 + average);  //h=1/(1+A)
}

double DimEmbedModule::separation(const HandleSeq& cluster,
                                  Type linkType) const
{
    if (!nameserver().isLink(linkType))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(linkType).c_str());

    const AtomEmbedding& aE = (atomMaps.find(linkType))->second;
    double minDist=DBL_MAX;
    for (AtomEmbedding::const_iterator it=aE.begin();it!=aE.end();++it) {
        bool inCluster=false; //whether *it is in cluster
        bool better=false; //whether *it is closer to some element of cluster
                           //than minDist
        double dist;
        for (HandleSeq::const_iterator it2=cluster.begin();
                                      it2!=cluster.end();++it2) {
            if (it->first==*it2) {
                inCluster=true;
                break;
            }
            dist = euclidDist(it->second,getEmbedVector(*it2,linkType));
            if (dist<minDist) better=true;
        }
        //If the node is closer and it is not in the cluster, update minDist
        if (better && !inCluster) {
            minDist=dist;
        }
        better=false;
        inCluster=false;
    }
    return minDist;
}

Handle DimEmbedModule::blendNodes(Handle n1,
                                  Handle n2, Type l)
{
    if (!nameserver().isLink(l))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(l).c_str());
    if (!n1->is_node() || !n2->is_node())
        throw InvalidParamException(TRACE_INFO,
                                    "blendNodes requires two nodes.");
    if (!isEmbedded(l)) {
        const char* tName = nameserver().getTypeName(l).c_str();
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
    for (unsigned int i=0; i<numDims; i++) {
        CoverTreePoint p1(Handle::UNDEFINED,newVec);
        newVec[i]=embedVec2[i];
        CoverTreePoint p2(Handle::UNDEFINED,newVec);
        double dist1 = p1.distance(cTree.k_nearest_neighbors(p1,1)[0]);
        double dist2 = p2.distance(cTree.k_nearest_neighbors(p2,1)[0]);
        if (dist1>dist2) newVec[i]=embedVec2[i];
    }
    std::string prefix("blend_"+n1->to_string()+"_"+n2->to_string()+"_");
    Handle newNode = add_prefixed_node(*as, n1->get_type(), prefix);

    for (unsigned int i=0; i<numDims; i++) {
        double strength = sqrt(newVec[i]);
        TruthValuePtr tv(SimpleTruthValue::createTV(strength, strength));
        Handle hi = as->add_link(l, newNode, pivots[i]);
        hi->setTruthValue(hi->getTruthValue()->merge(tv));
    }
    return newNode;
}

double DimEmbedModule::euclidDist(double v1[], double v2[], int size)
{
    double dist=0;
    for (int i=0; i<size;++i) {
        dist += std::pow((v1[i]-v2[i]), 2);
    }
    dist = sqrt(dist);
    return dist;
}

double DimEmbedModule::euclidDist(Handle h1,
                                  Handle h2,
                                  Type l,
                                  bool fanin)
{
    if (!nameserver().isLink(l))
        throw InvalidParamException(TRACE_INFO,
            "DimensionalEmbedding requires link type, not %s",
            nameserver().getTypeName(l).c_str());
    return euclidDist(getEmbedVector(h1, l, fanin),
                      getEmbedVector(h2, l, fanin));
}

double DimEmbedModule::euclidDist(const std::vector<double>& v1,
                                  const std::vector<double>& v2)
{
    OC_ASSERT(v1.size()==v2.size());
    std::vector<double>::const_iterator it1=v1.begin();
    std::vector<double>::const_iterator it2=v2.begin();

    double distance=0;
    //Calculate euclidean distance between v1 and v2
    for (; it1 != v1.end(); ++it1, ++it2)
        distance+=sq(*it1 - *it2);
    distance=sqrt(distance);
    return distance;
}

void DimEmbedModule::handleAddSignal(Handle h)
{
    AtomEmbedMap::iterator it;
    AsymAtomEmbedMap::iterator it2;
    if (NodeCast(h)) {
        //for each link type embedding that exists, add the node
        for (it = atomMaps.begin(); it != atomMaps.end(); ++it) {
            addNode(h, it->first);
        }
        for (it2 = asymAtomMaps.begin(); it2 != asymAtomMaps.end(); ++it2) {
            addNode(h, it2->first);
        }
    }
    else {//h is a link
        for (it = atomMaps.begin(); it != atomMaps.end(); ++it) {
            //if the new link is a subtype of an existing embedding, add it
            if (nameserver().isA(h->get_type(), it->first))
                addLink(h, it->first);
        }
        for (it2 = asymAtomMaps.begin(); it2 !=asymAtomMaps.end(); ++it2) {
            if (nameserver().isA(h->get_type(), it2->first))
                addLink(h, it2->first);
        }
    }
}

void DimEmbedModule::atomRemoveSignal(AtomPtr atom)
{
    Handle h = atom->get_handle();
    if (NodeCast(atom)) {
        //for each link type embedding that exists, remove the node
        AtomEmbedMap::iterator it;
        for (it = atomMaps.begin(); it != atomMaps.end(); ++it) {
            removeNode(h, it->first);
        }
        AsymAtomEmbedMap::iterator it2;
        for (it2 = asymAtomMaps.begin(); it2 != asymAtomMaps.end(); ++it2) {
            removeNode(h, it2->first);
        }
    }
}

void DimEmbedModule::TVChangedSignal(Handle h, TruthValuePtr a, TruthValuePtr b)
{
	atomRemoveSignal(h);
	handleAddSignal(h);
}
