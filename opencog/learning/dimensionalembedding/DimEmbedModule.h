/*
 * opencog/learning/dimensionalembedding/DimEmbedModule.h
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

#ifndef _OPENCOG_DIM_EMBED_MODULE_H
#define _OPENCOG_DIM_EMBED_MODULE_H

#include <list>
#include <map>
#include <string>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/Module.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/cover_tree.h>
#include <opencog/learning/dimensionalembedding/CoverTreeNode.h>

namespace opencog
{
    /**
     * The DimEmbedModule class implements the dimensional embedding
     * technique as described on
     * http://www.opencog.org/wiki/OpenCogPrime:DimensionalEmbedding
     *
     * @todo Add support for non-symmetric links.
     * @bug might fail if a node is deleted after embedding (esp. a pivot node)
     * or added during embedding
     */
    class DimEmbedModule : public Module
    {
    private:
        typedef std::map<Handle, std::list<double> > AtomEmbedding;
        typedef std::list<Handle> PivotSeq;
        typedef std::map<Type, PivotSeq> PivotMap;
        typedef std::map<Type, AtomEmbedding> AtomEmbedMap;
        typedef std::map<Type, node<CoverTreeNode> > EmbedTreeMap;

        AtomSpace* as;
        AtomEmbedMap atomMaps;
        PivotMap pivotsMap;//Pivot atoms which act as the basis
        EmbedTreeMap embedTreeMap;
        int clusters;//to keep track of the number of clusters
        /**
         * Adds h as a pivot and adds the distances from each node to
         * the pivot to the appropriate atomEmbedding.
         *
         * @param h Handle to be added as a pivot
         * @param linkType Type of link for which h should be added as a pivot
         */
        void addPivot(const Handle& h, const Type& linkType);

        /**
         * Clears the AtomEmbedMap and PivotMap for linkType
         *
         * @param linkType Type of link for which the embedding should be
         * cleared.
         */
        void clearEmbedding(const Type& linkType);

        /**
         * Returns the highest weight path between the handles following
         * links of type linkType, where path weight is the product of

         * (tv.strength*tv.confidence) of the links in the path. ie if
         * (l1, l2,...ln) are the links in the path, then the path weight is
         * defined as
         * (l1.tv.strength*l1.tv.confidence)*(l2.tv.strength*l2.tv.confidence)*
         * ...(ln.tv.strength*ln.tv.confidence).
         * The greater the path weight, the closer the two nodes are.
         *
         * Path weight will always be between 0 and 1. Returns 0 if no path
         * exists and 1 if startHandle == targetHandle.
         *
         * Uses a modified version of Dijkstra's algorithm.
         *
         * @param startHandle The starting handle for pathfinding
         * @param targetHandle The target handle for pathfinding
         * @param linkType The type of link to follow in pathfinding
         *
         * @return The highest weight path from startHandle to targetHandle
         * following only links of type linkType, where path weight is defined
         * above.
         *
         * @todo implement some kind of threshold or limit (as an optional 4th 
         * parameter) after which two nodes are considered unconnected.
         */
        double findHighestWeightPath(const Handle& startHandle,
                                     const Handle& targetHandle,
                                     const Type& linkType);

        /**
         * Returns a list of doubles corresponding to the handle h's
         * embedding of link type l. Throws an exception if no embedding
         * exists yet for type l. Calculates the list if an embedding exists
         * (ie pivots are picked) but handle h has not been calculated yet.
         *
         * @param h The handle whose embedding list is returned
         * @param l The link type for which h's embedding list is wanted
         *
         * @return A list of doubles corresponding to handle h's distance
         * from each of the pivots.
         */
        std::list<double> getEmbedlist(const Handle& h, const Type& l);

        double euclidDist(double v1[], double v2[], int size);
    public:
        static const unsigned int numDimensions=5;//number of pivot atoms
        const char* id();

        DimEmbedModule();
        virtual ~DimEmbedModule();
        virtual void init();
        
        /**
         * Creates an AtomEmbedding of the atomspace using linkType
         * and registers it with the AtomEmbedMap. If an AtomEmbedding
         * already exists for the supplied link type it will replace
         * it.
         *
         * @param linkType The type of link for which a dimensional embedding
         * is wanted.
         *
         * @todo improve pivot-picking technique: currently pivots are just
         * picked as the farthest from the current pivots, but connectedness
         * should be incorporated to avoid picking pivots with no links
         */
        void embedAtomSpace(const Type& linkType);

        /**
         * Logs a string representation of of the (Handle,list<Double>)
         * pairs for linkType. This will have as many entries as there are nodes
         * in the atomspace (unless nodes have been added since the embedding.
         * Just used for testing/debugging.
         */
        void logAtomEmbedding(const Type& linkType);

        /**
         * Adds node to the appropriate AtomEmbedding in the AtomEmbedMap.
         *
         * @param h Handle of node to be embedded.
         * @param linkType Type of link to use to embed h
         * @return The embedding list (a list of doubles between 0 and 1)
         */
        std::list<double> addNode(const Handle& h, const Type& linkType);

        /**
         * Returns true if a dimensional embedding exists for linkType l
         */
        bool isEmbedded(const Type& linkType);
        
        /**
         * Returns the distance between handles h1 and h2 for the embedding of
         * link type l.
         *
         * ie if the embedding lists for h1 and h2 for type l is given by
         * h1: (a1, b1, ..., n1) and
         * h2: (a2, b2, ..., n2)
         * Then their distance for link type l is
         * sqrt((a1-a2)^2 + (b1-b2)^2 + ... + (n1-n2)^2)
         */
        double euclidDist(const Handle& h1, const Handle& h2, const Type& l);
        static double euclidDist
            (const std::list<double> v1, const std::list<double> v2);

        /**
         * Returns a list of Handles of the k nearest nodes for the given 
         * link type.
         */
        HandleSeq kNearestNeighbors(const Handle& h, const Type& l, int k);

        /**
         * Use hierarchical clustering to make new nodes using the
         * dimensional embedding.
         */
        void cluster(const Type& l);

        /** updates the given atom (recalculates its distance from pivots) */
        //void updateAtom(const Handle& h, const Type& linkType);

        void printEmbedding();
    }; // class
} //namespace

#endif // _OPENCOG_DIM_EMBED_MODULE_H
