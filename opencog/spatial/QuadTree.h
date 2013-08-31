/*
 * opencog/spatial/QuadTree.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
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

#ifndef _SPATIAL_QUADTREE_H_
#define _SPATIAL_QUADTREE_H_

#include <opencog/spatial/HPASearch.h>
#include <boost/shared_ptr.hpp>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        /**
         * class QuadTree
         * Subdivides the grid on a quad tree and builds
         * a graph that represents the free nodes of the tree
         */
        class QuadTree
        {
        public:
            enum POSITION 
            {
                TOP_LEFT,
                TOP_RIGHT,
                BOTTOM_LEFT,
                BOTTOM_RIGHT,
                RIGHT,
                LEFT,
                TOP,
                BOTTOM
            };

            QuadTree( HPASearch::Level* level, const GridPoint& cellPosition, 
                unsigned int clusterSideSize, HPASearch::Graph* graph, 
                    QuadTree* parentQuad = 0, GridPoint* currentPosition = 0 );

            virtual ~QuadTree( void ) { }

            void connectEdges( void );

        private:
            // connect two quads vertices
            void connectQuads( QuadTree* quad1, QuadTree* quad2, bool vertical );

            // connect the two vertices lists with edges
            void processVertices( const std::vector<unsigned int>& vertices1, const std::vector<unsigned int>& vertices2 );

            // return the vertices at POSITION
            std::vector<unsigned int> getVerticesFrom( POSITION position );

            // if this quad has a free center position, vertexId will be defined
            unsigned int vertexId;
            unsigned int clusterSideSize;
            // cell positioned at the center of quad
            GridPoint centerCellPosition;

            bool hasFreeCenter;

            std::map<POSITION, boost::shared_ptr<QuadTree> > quads;

            HPASearch::Graph* graph;
            HPASearch::Level* level;
        };

    } // spatial
/** @}*/
} // opencog

#endif // _SPATIAL_QUADTREE_H_
