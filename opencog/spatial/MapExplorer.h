/*
 * opencog/spatial/MapExplorer.h
 *
 * Copyright (C) 2009 Novamente LLC
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

#ifndef MAPEXPLORER_H
#define MAPEXPLORER_H

#include <opencog/spatial/3DSpaceMap/Octree3DMapManager.h>
#include <opencog/spatial/math/Quaternion.h>
#include <opencog/spatial/math/Rectangle.h>

#include <boost/thread/thread.hpp>

struct SDL_Surface;

namespace opencog 
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial 
    {
        /**
         * Map Explorer is a tool useful for debugging purposes.
         * it shows in 3d the current state of a given LocalSpaceMap.
         * The user can extract information about the elements
         * by clicking on it.
         */
        class MapExplorer 
        {
        public:
            MapExplorer( spatial::Octree3DMapManager* map, unsigned int screenWidth,
                         unsigned int screenHeight, bool fullScreen = false ) 
                throw(opencog::RuntimeException);
            
            virtual ~MapExplorer( void );
            
            void updateMap( spatial::Octree3DMapManager* map );
            
            // SDL key/mouse functions
            void keyPressed( int key );
            void keyReleased( int key );       
            void mouseMoved( int x, int y );
            
            /**
             * Initialize the simulation
             */
            void start( void );
            /**
             * Stop the simulation
             */
            void stop( void );
            /**
             * Wait until the simulation ends
             */
            void wait( void );

            /**
             * Update the elements of the simulation
             * Prepare the whole simulation with the
             * latest values of the LocalSpaceMap and
             * render the scene
             * @param elapsedTime the time passed since
             *                    the start of the simulation
             */
            bool update( long elapsedTime );

            /**
             * When this simulation is started a thread is then created
             * to let the other parts of the application keep going.
             * This operator is used by the boost::thread
             * to keep the simulation running.
             */
            void operator()( void ) throw(opencog::RuntimeException);

            /**
             * Running status checker.
             * @return true if running false otherwise
             */
            bool isRunning( void ) const 
            {
                return this->running;
            }

        private:

            // helper functions

            /**
             * Render a given entity in the given color
             * @param entity The entity that will be rendered
             * @param color the color used to render the entity
             */
            void renderEntity( const spatial::Entity3D& entity, unsigned int color );

            /**
             * Render a border of the entity to highlight it
             * 
             * @param entity The entity that will be highlighted
             */
            void renderEntitySelection( const spatial::Entity3D& entity );

            /**
             * Render a given text on a flat quad defined by a given
             * rectangle.
             * @param text A string containing the text that will be rendered
             * @param area A rectangle used to build the flat quad area
             *             where the text will be written on
             * @param color Text color
             * @param bgColor Background color
             * @param stretchTexture If true the texture will be stretched to
             *                       the rectangle area, otherwise it will be
             *                       applied as it is.
             * @param repeatTexture If the texture is lesser than the area,
             *                      it can be repeated along it
             */
            void renderText(const std::string& text, const spatial::math::Rectangle& area, 
                            unsigned int color, unsigned int bgColor, 
                            bool stretchTexture = false, bool repeatTexture = false );
            /**
             * Render the textual information that is shown on the screen.
             * i.e help, frames per second, entity info
             */
            void renderHUD( void );
        
            /**
             * Select the nearest to camera entity that is being
             * pointed by the cross (at the screen center)
             */
            void selectEntity( void );
        
            /**
             * Render all the LocalSpaceMap entities 
             */
            void renderEntities( void );        

            /**
             * Configure the Projection Matrix to be a custom
             * Frustum. The Frustum parameters were been hardcoded
             * inside the function body.
             */
            void resetFrustum( void );

            /**
             * Extract the red bits from a 32 bits color
             * @param color Unsigned int (32 bits) color
             * @return unsigned char a byte containing the red color
             */
            static unsigned char getRed( unsigned int color ) 
            {
                return (color&0xff000000) >> 24;
            }
            /**
             * Extract the green bits from a 32 bits color
             * @param color Unsigned int (32 bits) color
             * @return unsigned char a byte containing the green color
             */
            static unsigned char getGreen( unsigned int color ) 
            {
                return (color&0x00ff0000) >> 16;
            }
            /**
             * Extract the blue bits from a 32 bits color
             * @param color Unsigned int (32 bits) color
             * @return unsigned char a byte containing the blue color
             */
            static unsigned char getBlue( unsigned int color ) 
            {
                return (color&0x0000ff00)  >> 8;
            }
            /**
             * Extract the blue bits from a 32 bits color
             * @param color Unsigned int (32 bits) color
             * @return unsigned char a byte containing the alpha channel
             */
            static unsigned char getAlpha( unsigned int color ) 
            {
                return (color&0x000000ff);
            }


            /**
             * Create the textures that will be applyed to the floor
             */
            void generateFloorTextures( void );

            spatial::Octree3DMapManager* map;
            bool mapUpdated;

            unsigned int screenWidth;
            unsigned int screenHeight;
            unsigned int halfScreenWidth;
            unsigned int halfScreenHeight;
            bool fullScreen;
            bool running;
            SDL_Surface* screen;
        
            int leftRightDirection;
            int upDownDirection;
            spatial::math::Vector3 cameraTranslation;
            spatial::math::Quaternion cameraOrientation;
            bool turbo;

            double yRot,zRot;

            unsigned int floorTextureCheckerId;
            unsigned int floorTextureOccupancyId;
            unsigned int floorTextureId;
            unsigned char* floorTextureChecker;
            unsigned char* floorTextureOccupancy;

            long updateTime;
            long updateCounter;
            long currentFps;

            bool selectRequested;
            unsigned int mouseSelectX;
            unsigned int mouseSelectY;
            bool mouseRecentered;
            std::string selectedEntity;
            bool showHelp;

            boost::thread* service;
            boost::mutex mapUpdateMutex;
        };

    }; // spatial
}; // opencog

#endif // MAPEXPLORER_H
