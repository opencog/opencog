/*
 * opencog/server/DimensionalEmbedding.h
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
#ifndef _OPENCOG_DIMENSIONAL_EMBEDDING_H
#define _OPENCOG_DIMENSIONAL_EMBEDDING_H

#include <vector>
#include <map>
#include <string>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>

namespace opencog
{
    class CogServer;
    /**
     * This class implements the dimensional embedding technique as described
     * on http://www.opencog.org/wiki/OpenCogPrime:DimensionalEmbedding
     */
    class DimensionalEmbedding : public opencog::Agent {
    private:
        typedef std::map<Handle, std::vector<double> > AtomEmbedding;
        typedef std::vector<Handle> PivotSeq;
        typedef std::map<Type, PivotSeq> PivotMap;
        typedef std::map<Type, AtomEmbedding> AtomEmbedMap;

        AtomSpace* as;
        AtomEmbedMap atomMaps;
        PivotMap pivotsMap;//Pivot atoms which act as the basis
        size_t numDimensions;//Number of pivot atoms
        
        /**
         * Adds h as a pivot and adds the distances from each node to
         * the pivot to the appropriate atomEmbedding.
         */
        void addPivot(const Handle& h, const Type& linkType);

        /** clears the AtomEmbedMap and PivotMap for linkType */
        void clearEmbedding(const Type& linkType);

        /**
         * Returns the highest weight path between the handles following
         * links of type linkType, where path weight is the product of
         * (tv.strength*tv.confidence) of the links. Path weight will
         * always be between 0 and 1. Returns 0 if no path exists.
         * Uses a modified version of Dijkstra's algorithm.
         */   
        double findHighestWeightPath(const Handle& startHandle,
                                     const Handle& targetHandle,
                                     const Type& linkType);
    public:
        virtual const ClassInfo& classinfo() const { return info(); }
        static const ClassInfo& info() {
            static const ClassInfo _ci("DimensionalEmbedding");
            return _ci;
        }

        void run (CogServer* server);

        DimensionalEmbedding();
        ~DimensionalEmbedding() { };

        /**
         * Embeds and prints similarity links.
         * Just used for scheme for testing
         */
        void embedSimLinks();
        void logSimEmbedding();

        /**
         * Creates an AtomEmbedding of the atomspace using linkType
         * and registers it with the AtomEmbedMap. If an AtomEmbedding
         * already exists for the supplied link type it will replace
         * it.
         */
        void embedAtomSpace(const Type& linkType);

        /**
         * Logs a string representation of of the (Handle,vector<Double>)
         * pairs for linkType. This will have as many entries as there are nodes
         * in the atomspace. Just used for testing/debugging.
         */
        void logAtomEmbedding(const Type& linkType);

        /** adds node to the appropriate AtomEmbedding in the AtomEmbedMap */
        void addNode(const Handle& h, const Type& linkType);

        /** updates the given atom (recalculates its distance from pivots) */
        //void updateAtom(const Handle& h, const Type& linkType);
    };
}
#endif // _OPENCOG_DIMENSIONAL_EMBEDDING_H
