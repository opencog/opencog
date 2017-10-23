/*
 * opencog/spatial/HPASearch.h
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

#ifndef _SPATIAL_HPASEARCH_H_
#define _SPATIAL_HPASEARCH_H_

#include <opencog/spatial/LocalSpaceMap2D.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <string>
#include <opencog/spatial/Prerequisites.h>
#include <opencog/spatial/math/Dimension2.h>
#include <opencog/spatial/math/Vector2.h>
#include <vector>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {


        /**
         * This is an implementation of the HPA* algorithm. HPA* builds an abstract graph by doing
         * a grid partitioning that results on a set of clusters of original grid cells.
         * Each two sibbling clusters may or may not has entrances. An entrance is the way
         * to cross from on cluster to another. The pathfinding is the execution of the A* on
         * the abstract graph.
         */
        class HPASearch
        {
        public:

            // typedefs of the boost graph datatype
            struct VertexPosition {
                typedef boost::vertex_property_tag kind;
            };

            typedef boost::property< VertexPosition, math::Vector2> VertexProperty;

            typedef boost::property< boost::edge_weight_t, float > EdgeProperty;

            //typedef std::pair< LocalSpaceMap::Cell*, LocalSpaceMap::Cell* > Edge;
            typedef std::pair< GridPoint, GridPoint > Edge;

            typedef boost::adjacency_list < boost::vecS, boost::vecS,
                boost::undirectedS, VertexProperty, EdgeProperty > Graph;

            typedef boost::graph_traits < Graph >::vertex_descriptor VertexDescriptor;
            typedef boost::graph_traits < Graph >::edge_descriptor EdgeDescriptor;
            typedef boost::graph_traits < Graph >::vertex_iterator VertexIterator;
            typedef std::pair < boost::graph_traits < Graph >::edge_iterator,
                boost::graph_traits < Graph >::edge_iterator > EdgeIterator;

            /**
             * class FoundGoal
             * Exception throwed when a path was found
             */
            class FoundGoal { };

            /**
             * class AStarGoalVisitor
             * Visitor that terminates the algorithm's excetution when we find the goal
             */
            class AStarGoalVisitor : public boost::default_astar_visitor
                {
                public:
                AStarGoalVisitor( unsigned int vertexGoalId ) :
                    vertexGoalId( vertexGoalId ) { }

                    // visitor method
                    template <class Graph> void examine_vertex( VertexDescriptor u, Graph& graph ) {
                        //   math::Vector2 vertexPosition = boost::get( HPASearch::VertexPosition( ), graph, u );
                        //   std::cout << vertexPosition.toString( ) << " >>> " << u << vertexGoalId << std::endl;
                        if ( u == vertexGoalId ) {
                            throw FoundGoal( );
                        } // if
                    }
                private:
                    unsigned int vertexGoalId;
                };


            /**
             * class AStarDistanceHeuristic
             * Heuristic used by AStar. (Euclidean distance)
             */
            class AStarDistanceHeuristic : public boost::astar_heuristic< Graph, float >
            {
            public:

            AStarDistanceHeuristic( const math::Vector2& goalPosition, const Graph& graph )
                : goalPosition( goalPosition ), graph( graph ) { }

                float operator( )( VertexDescriptor u ) {
                    // math::Vector2 uPos = boost::get( HPASearch::VertexPosition( ), graph, u );
                    // std::cout << "H: " << uPos.toString( ) << std::endl;
                    return ( goalPosition - boost::get( HPASearch::VertexPosition( ), graph, u ) ).length( );
                }

            private:
                math::Vector2 goalPosition;
                Graph graph;
            };

            /**
             * class Level
             * HPA* can handle multiple levels in a hierarchical way,
             * Each level has n clusters. More clusters means that such level is more detailed
             */
            class Level
            {
            public:

                Level( LocalSpaceMap2D* map, unsigned int level, unsigned int maximumClusters );

                virtual ~Level( );

                const Graph& getAbstractGraph( void );

                bool processPath( const math::Vector2& startPoint, const math::Vector2& endPoint );

                const std::vector<math::Vector2>& getProcessedPath( void ) const;

                float getClusterWidth( void );

                float getClusterHeight( void );

                unsigned int getLevel( void );

                unsigned int getClusterId( const GridPoint& gridPoint ) const;

            protected:

                void buildClusters( void );

                void buildEntrance( unsigned int row, unsigned int col, bool horizontal );
                GridPoint getNearestEntrance( unsigned int clusterId, const math::Vector2& position );

                GridPoint getNearestVertex( unsigned int clusterId, const math::Vector2& position );

                void setupVertex( const GridPoint& gridPoint );

                void smoothPath( void );

                // try to reduce path nodes doing triangulations using raytrace
                void smoothTriangulate( void );

                std::vector<math::Vector2> processedPath;

                LocalSpaceMap2D* map;
                std::vector< std::pair<GridPoint, GridPoint> > entrances;
                std::map< unsigned int, std::vector< GridPoint > > clusterEntrances;
                std::map< GridPoint, unsigned int > graphVertices;
                std::map< unsigned int, std::pair< unsigned int, unsigned int> > clustersVertexRange;
                math::Dimension2 clusterDimension;
                Graph* abstractGraph;
                unsigned int level;

                unsigned int vertexCounter;

                unsigned int numberOfCols;
                unsigned int numberOfRows;

                bool needsUpdate;

                friend class Cluster;
                friend class QuadTree;
            };

            HPASearch( LocalSpaceMap2D* map, unsigned int numberOfLevels = 1, unsigned int maximumClusters = 16 );

            // process a path from informed start and end positions
            bool processPath( const math::Vector2& startPoint, const math::Vector2& endPoint, unsigned int levelId = 1 );

            const std::vector<math::Vector2>& getProcessedPath( unsigned int levelId ) const;

            virtual ~HPASearch(void);

            Level* getLevel( unsigned int levelId ) const;

            unsigned int getNumberOfLevels( void );

            LocalSpaceMap2D* map;
            unsigned int numberOfLevels;

            std::vector<Level*> levels;

        };

    } // spatial
/** @}*/
} // opencog

#endif
