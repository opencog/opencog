/*
 * opencog/learning/dimensionalembedding/DimEmbedModule.h
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

#ifndef _OPENCOG_DIM_EMBED_MODULE_H
#define _OPENCOG_DIM_EMBED_MODULE_H

#include <map>
#include <string>
#include <vector>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/AttentionBank.h>
#include <opencog/cogserver/server/Module.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/util/Cover_Tree.h>
#include "CoverTreePoint.h"

namespace opencog
{
    /**
     * The DimEmbedModule class implements the dimensional embedding
     * technique as described on
     * http://www.opencog.org/wiki/OpenCogPrime:DimensionalEmbedding
     *
     * @todo Add support for non-symmetric links.
     */
    class DimEmbedModule : public Module
    {
    private:
        AttentionBank* _bank;
        typedef std::map<Handle, std::vector<double> > AtomEmbedding;
        typedef std::map<Type, HandleSeq> PivotMap;
        typedef std::map<Type, std::pair<HandleSeq, HandleSeq> > AsymPivotMap;
        typedef std::map<Type, AtomEmbedding> AtomEmbedMap;
        typedef std::map<Type, std::pair<AtomEmbedding, AtomEmbedding> >
            AsymAtomEmbedMap; //For asymmetric embeddings: The first of the pair
        //is for links like (inheritance pivot atom) (ie pivot is source)
        //the second is for (inheritance atom pivot) (ie pivot is target)
        //the "fanin" argument in several functions represents whether the links
        //go "inward", with pivots as targets (ie the second embedding)
        typedef std::map<Type, CoverTree<CoverTreePoint> > EmbedTreeMap;
        typedef std::map<Type, std::pair<CoverTree<CoverTreePoint>,
                                         CoverTree<CoverTreePoint> > >
            AsymEmbedTreeMap;
        typedef std::vector<std::pair<HandleSeq,std::vector<double> > >
            ClusterSeq; //the vector of doubles is the centroid of the cluster
        
        AtomSpace* as;
        boost::signals2::connection removedAtomConnection;
        boost::signals2::connection addedAtomConnection;
        boost::signals2::connection tvChangedConnection;

        AtomEmbedMap atomMaps;
        AsymAtomEmbedMap asymAtomMaps;
        PivotMap pivotsMap;//Pivot atoms which act as the basis
        AsymPivotMap asymPivotsMap;
        EmbedTreeMap embedTreeMap;
        AsymEmbedTreeMap asymEmbedTreeMap;
        std::map<Type,int> dimensionMap;//Stores the number of dimensions that
                                        //each link type is embedded under

        /**
         * Adds h as a pivot and adds the distances from each node to
         * the pivot to the appropriate atomEmbedding. Also increase
         * the VLTI of the new pivot (we don't want pivot atoms to be
         * forgotten).
         *
         * @param h Handle to be added as a pivot
         * @param linkType Type of link for which h should be added as a pivot
         * @param fanin For asymmetric link types, we need to embed twice,
         * once with fanin=true and once with fanin=false. The fanin=true
         * embedding of a given node represents the weight of the path
         * starting from the node and going to the pivot.
         */
        void addPivot(Handle h, Type linkType, bool fanin=false);
        Handle pickPivot(Type linkType, HandleSeq& nodes, bool fanin=false);
        /**
         * Adds node to the appropriate AtomEmbedding in the AtomEmbedMap.
         *
         * We assume that the atomspace has already been embedded, so we can
         * quickly calculate the new embedding for this node. If every other
         * node is already embedded exactly, then this new embedding will be
         * exact, except that the embeddings of other nodes may now be off
         * (eg if we add node A, which is connect to a pivot and node B,
         * node B will have a new, shorter path to the pivot, but its
         * embedding will not be updated to reflect this).
         *
         * @param h Handle of node to be embedded.
         * @param linkType Type of link to use to embed h
         * @return The embedding vector (a vector of doubles between 0 and 1)
         */
        std::vector<double> addNode(Handle h,
                                    Type linkType);

        /**
         * Removes the node from the AtomEmbedding and Cover Tree for linkType.
         *
         * @param h Handle of node to be removed.
         * @param linkType Type for which h is removed from the embedding.
         */
        void removeNode(Handle h, Type linkType);
        
        /**
         * Updates nodes directly connected to link in the appropriate
         * AtomEmbedding for type linkType.
         *
         * We assume the atomspace has already been embedded, so we can
         * update any atoms directly connected by the new link to take
         * new paths allowed by the link into account. Note that this
         * will only change directly-connected atoms. Even though nodes
         * more hops away may now have shorter paths to the pivots, their
         * embedding vectors will not be updated to reflect this.
         *
         * @param h Handle of new link
         * @param linkType Type of link (which embedding to alter)
         */
        void addLink(Handle h, Type linkType);

        /**
         * For adding symmetric links after the atomspace has been embedded. See
         * addLink.
         */
        void symAddLink(Handle h, Type linkType);

        /**
         * For adding asymmetric links after the atomspace has been embedded.
         * See addLink.
         */
        void asymAddLink(Handle h, Type linkType);
    public:
        const char* id();

        DimEmbedModule(CogServer&);
        virtual ~DimEmbedModule();
        virtual void init();

        /**
         * Returns a vector of doubles corresponding to the handle h's
         * embedding of link type l. Throws an exception if no embedding
         * exists yet for type l. Calculates the vector if an embedding exists
         * (ie pivots are picked) but handle h has not been calculated yet.
         *
         * @param h The handle whose embedding vector is returned
         * @param l The link type for which h's embedding vector is wanted
         * @param fanin For asymmetric link types.
         * @return A vector of doubles corresponding to handle h's distance
         * from each of the pivots.
         */
        const std::vector<double>& getEmbedVector(Handle h, Type l, bool fanin=false) const;

        /**
         * Returns the list of pivots for the embedding of type l.
         */
        HandleSeq& getPivots(Type l, bool fanin=false);

        /**
         * Creates an AtomEmbedding of the atomspace using linkType
         * and registers it with the AtomEmbedMap. If an AtomEmbedding
         * already exists for the supplied link type it will replace
         * it.
         *
         * @param linkType The type of link for which a dimensional embedding
         * is wanted.
         * @param numDimensions The number of dimensions to embed the atomspace
         * in.
         *
         * @todo improve pivot-picking technique: currently pivots are just
         * picked as the farthest from the current pivots, but connectedness
         * should be incorporated to avoid picking pivots with no links
         */
        void embedAtomSpace(Type linkType, int numDimensions=5);

        /**
         * Clears the AtomEmbedMap and PivotMap for linkType, also
         * decreasing the VLTI of any pivots by 1.
         *
         * @param linkType Type of link for which the embedding should be
         * cleared.
         */
        void clearEmbedding(Type linkType);
        
        /**
         * Logs a string representation of of the (Handle,vector<Double>)
         * pairs for linkType. This will have as many entries as there are nodes
         * in the atomspace (unless nodes have been added since the embedding).
         * Just used for testing/debugging.
         */
        void logAtomEmbedding(Type linkType);
        
        /**
         * Returns true if a dimensional embedding exists for linkType l
         */
        bool isEmbedded(Type linkType) const;
        
        /**
         * Returns the euclidean distance between handles h1 and h2 for the
         * embedding of link type l.
         *
         * ie if the embedding vectors for h1 and h2 for type l is given by
         * h1: (a1, b1, ..., n1) and
         * h2: (a2, b2, ..., n2)
         * Then their distance for link type l is
         * sqrt((a1-a2)^2 + (b1-b2)^2 + ... + (n1-n2)^2)
         */
        double euclidDist(Handle h1, Handle h2, Type l, bool fanin=false);
        static double euclidDist
            (const std::vector<double>& v1, const std::vector<double>& v2);
        static double euclidDist(double v1[], double v2[], int size);
        
        /**
         * Returns a vector of Handles of the k nearest nodes for the given 
         * link type.
         *
         * @param h The handle to find the neighbors of
         * @param l The Type of link for which the neighbors are found
         * @param k The number of neighbors to find
         * @param fanin If l is asymmetric, indicates the embedding direction
         * @return A vector of at least k handles, sorted from nearest to
         * farthest (the 0ths element of the vector is closest to h). If there
         * is a tie for the kth closest, this may return more than k handles.
         */
        HandleSeq kNearestNeighbors(Handle h, Type l, int k, bool fanin=false);

        /**
         * Use k-means clustering to find clusters using the
         * dimensional embedding. This function won't actually add
         * anything to the atomspace, it just returns k vectors
         * of handles which might make good clusters.
         *
         * @param l type of link for which to find clusters.
         * @param numClusters The number of clusters to return.
         * @param nPasses The number of times to try clustering (there is
         * a stochastic component to k-means clustering, so the clustering
         * may be slightly different each time).
         * @return A vector of (HandleSeq,vector<double>) pairs, where
         * each HandleSeq represents a cluster and the vector of doubles its
         * centroid.
         */
        ClusterSeq kMeansCluster(Type l, int numClusters, int nPasses=1, bool pivotWise=false);

        /**
         * Use k-means clustering to add new nodes to the atomspace (one
         * new node for each good cluster found, plus inheritance links
         * between each new node and its corresponding cluster members).
         *
         * This will add up to maxClusters new clusters, choosing those
         * clusters which maximize homogeneity*separation.
         *
         * @param l Type of link for which to find clusters.
         * @param maxClusters The maximum number of nodes to add to the
         * atomspace.
         * @param threshold The threshold for accepting a cluster; clusters
         * not meeting this threshold will not be added to the atomspace.
         * Cluster quality is measured as homogeneity*separation (see the
         * homogeneity and separation functions for further explanation).
         * @param kPasses The number of different k values to try for
         * k-means clustering. K values are chosen exponentially across half
         * of the number of nodes. eg let n:=log(numNodes/2).
         * Then for kPasses=1, the k value is 2^(n/2). For
         * kPasses=2, the k values are 2^(n/3) and 2^(2n/3). For
         * kPasses=3, the k values are 2^(n/4), 2^(2n/4), and 2^(3n/4).
         */
        void addKMeansClusters(Type l, int maxClusters,
                               double threshold=0., int kPasses=-1);

        /**
         * Calculate the homogeneity of a cluster of handles for given linkType.
         *
         * Homogeneity is calculated as 1/(1+A) where A is the mean of
         * the distances of all members of the cluster to their nearest
         * clustermates.
         */
        double homogeneity(const HandleSeq& cluster, Type linkType) const;

        /**
         * Calculate the separation of a cluster of handles for given linkType.
         *
         * Separation is the minimum distance from any given member of the
         * cluster to elements outside the cluster.
         */
        double separation(const HandleSeq& cluster, Type linkType) const;

        /**
         * Create a new node by blending the two existing nodes, n1 and n2,
         * based on their embeddings for link type l.
         */
        Handle blendNodes(Handle n1, Handle n2, Type l);

        void printEmbedding();

        /**
         * Updates the embeddings for a new atom (the atomspace will
         * still need to be periodically reembedded). Should be registered
         * with the atomspace via addAtomSignal
         */
        void handleAddSignal(Handle h);

        /**
         * Removes the node from embedding & cover tree. Does not alter
         * the embedding vector of any nodes. Does nothing if removed
         * atom is a link.
         */
        void atomRemoveSignal(AtomPtr h);

        void TVChangedSignal(Handle, TruthValuePtr, TruthValuePtr);
    }; // class
} //namespace

#endif // _OPENCOG_DIM_EMBED_MODULE_H
