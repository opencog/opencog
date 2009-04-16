/*
 * opencog/spatial/Renderer.h
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

#ifndef _SPATIAL_RENDERER_H_
#define _SPATIAL_RENDERER_H_

#include <SDL/SDL.h>
#include <SDL/SDL_gfxPrimitives.h>

#include <vector>
#include "Vector2.h"
#include "LocalSpaceMap2DUtil.h"

namespace Spatial
{
class LocalSpaceMap2D;
class VisibilityMap;

class Renderer
{
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

} // Spatial

#endif
