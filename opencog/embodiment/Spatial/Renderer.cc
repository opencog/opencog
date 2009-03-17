#include "Renderer.h"
#include <iostream>
#include "LocalSpaceMap2D.h"
#include "VisibilityMap.h"

#include "HPASearch.h"

namespace Spatial {  

    const int Renderer::SCREEN_WIDTH = 1024;
    const int Renderer::SCREEN_HEIGHT = 1024;
  
    Renderer::Renderer( void ) {
        atexit(SDL_Quit);
        this->screen = SDL_CreateRGBSurface( SDL_SWSURFACE, SCREEN_WIDTH, SCREEN_HEIGHT, 32, 0, 0, 0, 0 );
        SDL_FillRect( this->screen, NULL, 0xffffffff );
    }

    Renderer::~Renderer(void) {
        SDL_FreeSurface( this->screen );
    }
  
  
    void Renderer::renderMap( LocalSpaceMap2D* map, unsigned int hpaClusters ) {
        unsigned int cellSize = SCREEN_WIDTH / map->xDim( );
        unsigned int row;
        unsigned int col;
        std::cout << "Number of cells: " << map->xDim( ) << std::endl;
        std::cout << "Cell size: " << cellSize << std::endl;

        std::vector<std::string> entitiesIds;
        map->getAllObjects( back_inserter( entitiesIds ) );

        unsigned int i;
        for( i = 0; i < entitiesIds.size( ); ++i ) {
            const Spatial::EntityPtr& entity = map->getEntity( entitiesIds[i] );
            const Spatial::Math::Vector3& entityPosition = entity->getPosition( );
            Spatial::Math::Vector3 target = entityPosition + ( entity->getDirection( ) * entity->getDimension( ).height );

            drawLine( map->snap( Spatial::Point( entityPosition.x, entityPosition.y ) ), 
                      map->snap( Spatial::Point( target.x, target.y ) ), cellSize, 0x000000ff );
        } // for
    

        // drawing grids
        for( row = 0; row < map->xDim( ); ++row ) {
            for( col = 0; col < map->xDim( ); ++col ) {
                GridPoint cell( col, row );
                Point point = map->unsnap( cell );
                //std::cout << "Row: " << row << " Col: " << col << " " << point.first << " " << point.second << std::endl;

                if ( map->gridOccupied_nonObstacle( cell ) ) {
                    drawTile( cell, cellSize, 0xaa000155 );
                } else if ( map->gridOccupied( cell ) ) {
                    drawTile( cell, cellSize, 0x00bb0099 );
                } else {
                    drawTile( cell, cellSize, 0xEFEFEF77 );
                } // else
            } // for
        } // for

        // drawing super entities boundaries
        unsigned int radius = 3;    
        const std::list<SuperEntityPtr>& superEntities = map->getSuperEntities( );
        std::list<SuperEntityPtr>::const_iterator it;
        for( it = superEntities.begin( ); it != superEntities.end( ); ++it ) {
            const std::list<Math::LineSegment>& edges = (*it)->getEdges( );
            std::list<Math::LineSegment>::const_iterator it2;
            for( it2 = edges.begin( ); it2 != edges.end( ); ++it2 ) {
                Spatial::GridPoint center = map->snap( Spatial::Point( it2->pointA.x, it2->pointA.y ) );
                drawCircle( center, radius, cellSize, 0x000000ff );
            } // for
        } // for

        if ( hpaClusters > 0 ) {
            // drawing HPA Quadtree
            HPASearch hpa( map, 1, hpaClusters );
            Spatial::Point start( map->unsnap( Spatial::GridPoint(0, 0) ) );
            Spatial::Point end( map->unsnap( Spatial::GridPoint( map->xDim( )-1, map->xDim( )-1 ) ) );
            hpa.processPath( Math::Vector2( start.first,start.second ), Math::Vector2( end.first,end.second ) );

            HPASearch::Level* level = hpa.getLevel( 1 );
            const HPASearch::Graph& graph = level->getAbstractGraph( );
      
            HPASearch::EdgeIterator it3;
            for( it3 = boost::edges( graph ); it3.first != it3.second; ++it3.first ) {
                HPASearch::VertexDescriptor v1 = boost::source( *it3.first, graph );
                Math::Vector2 p1 = boost::get( HPASearch::VertexPosition( ), graph, v1 );
                HPASearch::VertexDescriptor v2 = boost::target( *it3.first, graph );
                Math::Vector2 p2 = boost::get( HPASearch::VertexPosition( ), graph, v2 );
	
                //std::cout << p1.toString( ) << " " << p2.toString( ) << std::endl;
	
                drawLine( map->snap( Spatial::Point( p1.x, p1.y ) ), map->snap( Spatial::Point( p2.x, p2.y ) ), cellSize, 0xff0000ff );
            } // for
        } // if
      
    }
  
    void Renderer::renderVisMap( VisibilityMap* map ) {
        unsigned int row;
        unsigned int col;
        unsigned int numberOfCells = map->getNumberOfTilesPerRow( );
        unsigned int cellSize = (SCREEN_WIDTH / numberOfCells);
        std::cout << "Number of cells: " << numberOfCells << std::endl;
        std::cout << "Cell size: " << cellSize << std::endl;

        for( row = 0; row < numberOfCells; ++row ) {
            for( col = 0; col < numberOfCells; ++col ) {
                GridPoint cell( col, row );
                //std::cout << "Row: " << row << " Col: " << col << " " << map->getTile( row, col )->getCenter( ).toString( ) << std::endl;
                if ( !map->getTile( row, col )->isVisible( ) ) {
                    drawTile( cell, cellSize, 0xBBBBBB77 );
                } // if

            } // for
        } // for    
    }

    void Renderer::saveImageFile( const std::string& fileName ) {
        SDL_SaveBMP(this->screen, fileName.c_str( ) );
    }

    void Renderer::drawLine( const GridPoint& startPoint, const GridPoint& endPoint, unsigned int tileSide, unsigned int color ) {
        int startXPixel = (startPoint.first * tileSide);
        int endXPixel = (endPoint.first * tileSide);

        int startYPixel = SCREEN_HEIGHT - ( ( startPoint.second * tileSide ) + tileSide/2);
        int endYPixel = SCREEN_HEIGHT - ( ( endPoint.second * tileSide ) + tileSide/2);


        lineColor( this->screen, startXPixel, startYPixel, endXPixel, endYPixel, color );
    } 

    void Renderer::drawCircle( const GridPoint& center, unsigned int radius, unsigned int tileSide, unsigned int color ) {
        int centerX = (center.first * tileSide)+tileSide/2;
        int centerY = SCREEN_HEIGHT - (( center.second * tileSide )+tileSide/2);
        circleColor( this->screen, centerX, centerY, radius, color );
    }

    void Renderer::drawTile( const GridPoint& center, unsigned int side, unsigned int color ) {
        int startXPixel = (center.first * side);
        int endXPixel = startXPixel + side;
        int startYPixel = SCREEN_HEIGHT - ( center.second * side );
        int endYPixel = startYPixel - side;
        boxColor( this->screen, startXPixel, startYPixel, endXPixel, endYPixel, color );
    }

}; // Spatial

