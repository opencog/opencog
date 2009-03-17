#ifndef RENDERER_H
#define RENDERER_H

#include <SDL/SDL.h>
#include <SDL/SDL_gfxPrimitives.h>

#include <vector>
#include "Vector2.h"
#include "LocalSpaceMap2DUtil.h"

namespace Spatial {
    class LocalSpaceMap2D;
    class VisibilityMap;

    class Renderer {
    public:

        static const int SCREEN_WIDTH;
        static const int SCREEN_HEIGHT;

        Renderer( void );

        virtual ~Renderer( void );

        void renderMap( LocalSpaceMap2D* map, unsigned int hpaClusters = 0 );

        void renderVisMap( VisibilityMap* visMap );
    
        void saveImageFile( const std::string& fileName );
    
        void drawLine( const GridPoint& startPoint, const GridPoint& endPoint, unsigned int tileSide, unsigned int color );

        void drawCircle( const GridPoint& center, unsigned int radius, unsigned int tileSide, unsigned int color );

        void drawTile( const GridPoint& center, unsigned int side, unsigned int color );


    private:
        SDL_Surface* screen;
    };

}; // Spatial

#endif
