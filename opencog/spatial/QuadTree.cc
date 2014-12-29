/*
 * opencog/spatial/QuadTree.cc
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

#include <opencog/spatial/QuadTree.h>

using namespace opencog;
using namespace opencog::spatial;

QuadTree::QuadTree( HPASearch::Level* level, const GridPoint& cellPosition, unsigned int clusterSideSize, HPASearch::Graph* graph, QuadTree* parentQuad,  GridPoint* currentPosition )
{

    this->hasFreeCenter = false;
    this->graph = graph;
    this->level = level;
    this->clusterSideSize = clusterSideSize;
    bool splitted = false;
    if ( clusterSideSize > 1 ) {
        unsigned int row;
        unsigned int col;

        int currentRow = 0;
        int currentCol = 0;
        // do not start from beginning if currentPosition was set
        if ( currentPosition ) {

            currentRow = static_cast<int>( currentPosition->second ) -
                         static_cast<int>( cellPosition.second );

            currentCol = static_cast<int>( currentPosition->first ) -
                         static_cast<int>( cellPosition.first );

            if ( currentRow < 0 ) {
                currentRow = 0;
            } // if

            // if currentRow is greater than clusterSideSize, ignore it

            if ( currentCol < 0 ) {
                currentCol = 0;
            } else if ( currentCol > static_cast<int>( clusterSideSize ) ) { // goto next line
                currentCol = 0;
                ++currentRow;
            } // if

        } // if

        for ( row = currentRow; !splitted && row < clusterSideSize; ++row ) {
            for ( col = currentCol; !splitted && col < clusterSideSize; ++col ) {
                GridPoint currentPosition( cellPosition.first + col,
                                           cellPosition.second + row );

                // split cluster if it has obstacles
                if ( level->map->gridIllegal( currentPosition ) ) {
                    // split on four quads
                    unsigned int nextNumberOfColumns = clusterSideSize / 2;
                    // quad 1
                    QuadTree* quad1 = new QuadTree( level, cellPosition, nextNumberOfColumns, graph, this, &currentPosition );

                    // quad 2
                    QuadTree* quad2 = new QuadTree( level, GridPoint( cellPosition.first + nextNumberOfColumns, cellPosition.second ), nextNumberOfColumns, graph, this, &currentPosition );

                    // quad 3
                    QuadTree* quad3 = new QuadTree( level, GridPoint( cellPosition.first, cellPosition.second + nextNumberOfColumns ), nextNumberOfColumns, graph, this, &currentPosition );

                    // quad 4
                    QuadTree* quad4 = new QuadTree( level, GridPoint( cellPosition.first + nextNumberOfColumns, cellPosition.second + nextNumberOfColumns ), nextNumberOfColumns, graph, this, &currentPosition );

                    quads[ TOP_LEFT ].reset( quad1 );
                    quads[ TOP_RIGHT ].reset( quad2 );
                    quads[ BOTTOM_LEFT ].reset( quad3 );
                    quads[ BOTTOM_RIGHT ].reset( quad4 );

                    splitted = true;
                } // if
            } // for

            currentCol = 0;
        } // for

        // if cluster was not splitted, put a vertex on it's center
        if ( !splitted ) {
            this->centerCellPosition.second = cellPosition.second + clusterSideSize / 2;
            this->centerCellPosition.first = cellPosition.first + clusterSideSize / 2;

            this->hasFreeCenter = true;

            if ( parentQuad ) {

                boost::property_map<HPASearch::Graph, HPASearch::VertexPosition>::type position =
                    boost::get( HPASearch::VertexPosition( ), *graph );

                Point realPosition = level->map->unsnap( this->centerCellPosition );
                boost::put( position, level->vertexCounter, math::Vector2( realPosition.first, realPosition.second ) );
                this->vertexId = level->vertexCounter;
                level->graphVertices[ this->centerCellPosition ] = this->vertexId;
                ++level->vertexCounter;

            } // if

        } // if

    } else if ( !level->map->gridIllegal( cellPosition ) ) {
        // partitioning reaches the deeper level
        this->hasFreeCenter = true;

        if ( parentQuad ) {
            boost::property_map<HPASearch::Graph, HPASearch::VertexPosition>::type position =
                boost::get( HPASearch::VertexPosition( ), *graph );

            Point realPosition = level->map->unsnap( cellPosition );
            boost::put( position, level->vertexCounter, math::Vector2( realPosition.first, realPosition.second ) );

            this->vertexId = level->vertexCounter;
            level->graphVertices[ cellPosition ] = this->vertexId;
            ++level->vertexCounter;
        } // if
    } // else

}

void QuadTree::connectEdges( )
{
    if ( !quads.empty()) {

        std::map<POSITION, boost::shared_ptr<QuadTree> >::iterator it;

        for ( it = quads.begin( ); it != quads.end( ); ++it ) {
            it->second->connectEdges( );
        } // for
        // quad1 and quad2
        connectQuads( quads[ TOP_LEFT ].get( ), quads[ TOP_RIGHT ].get( ), false );
        // quad3 and quad4
        connectQuads( quads[ BOTTOM_LEFT ].get( ), quads[ BOTTOM_RIGHT ].get( ), false );
        // quad1 and quad3
        connectQuads( quads[ TOP_LEFT ].get( ), quads[ BOTTOM_LEFT ].get( ), true );
        // quad2 and quad4
        connectQuads( quads[ TOP_RIGHT ].get( ), quads[ BOTTOM_RIGHT ].get( ), true );

    } // if

}

void QuadTree::connectQuads( QuadTree* quad1, QuadTree* quad2, bool vertical )
{
    POSITION side1;
    POSITION side2;
    POSITION position1;
    POSITION position2;
    POSITION position3;
    POSITION position4;

    if ( vertical ) {
        side1 = BOTTOM;
        side2 = TOP;
        position1 = BOTTOM_LEFT;
        position2 = TOP_LEFT;
        position3 = BOTTOM_RIGHT;
        position4 = TOP_RIGHT;
    } else {
        side1 = RIGHT;
        side2 = LEFT;
        position1 = TOP_RIGHT;
        position2 = TOP_LEFT;
        position3 = BOTTOM_RIGHT;
        position4 = BOTTOM_LEFT;
    } // else

    if ( ( quad1->hasFreeCenter && quad2->hasFreeCenter ) ||
            ( !quad1->hasFreeCenter && !quad1->quads.empty() && quad2->hasFreeCenter ) ||
            ( !quad2->hasFreeCenter && !quad2->quads.empty() && quad1->hasFreeCenter ) ) {
        // simple quads
        processVertices( quad1->getVerticesFrom( side1 ),
                         quad2->getVerticesFrom( side2 ) );
    } else if (!quad1->quads.empty()) {
        // complexs quads (many subdivisions)
        connectQuads( quad1->quads[ position1 ].get( ), quad2->quads[ position2 ].get( ), vertical );
        connectQuads( quad1->quads[ position3 ].get( ), quad2->quads[ position4 ].get( ), vertical );
    } // else
}

void QuadTree::processVertices( const std::vector<unsigned int>& vertices1, const std::vector<unsigned int>& vertices2 )
{
    unsigned int i;
    unsigned int j;

    for ( i = 0; i < vertices1.size( ); ++i )
    {
        for ( j = 0; j < vertices2.size( ); ++j )
        {
            // math::Vector2 position1 =
            boost::get(HPASearch::VertexPosition(), *this->graph, vertices1[i]);

            // math::Vector2 position2 =
            boost::get(HPASearch::VertexPosition(), *this->graph, vertices2[j]);

            boost::add_edge(vertices1[i], vertices2[j], 1, *graph);
        }
    }
}

std::vector<unsigned int> QuadTree::getVerticesFrom( POSITION position )
{
    std::vector<unsigned int> response;
    if ( this->hasFreeCenter ) {
        response.push_back( vertexId );
    } else {
        std::back_insert_iterator< std::vector<unsigned int> > ii( response );

        std::map<POSITION, boost::shared_ptr<QuadTree> >::iterator it;
        for ( it = quads.begin( ); it != quads.end( ); ++it ) {
            if ( it->first == position ||
                    ( position == RIGHT && ( it->first == TOP_RIGHT || it->first == BOTTOM_RIGHT ) ) ||
                    ( position == LEFT && ( it->first == TOP_LEFT || it->first == BOTTOM_LEFT ) ) ||
                    ( position == TOP && ( it->first == TOP_LEFT || it->first == TOP_RIGHT ) ) ||
                    ( position == BOTTOM && ( it->first == BOTTOM_LEFT || it->first == BOTTOM_RIGHT ) ) ) {
                std::vector<unsigned int> partialResponse =
                    it->second->getVerticesFrom( position );

                std::copy( partialResponse.begin( ), partialResponse.end( ), ii );
            } // if
        } // for

    } // else
    return response;
}

