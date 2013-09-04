/*
 * opencog/spatial/MapExplorerSystem.cc
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

#include <opencog/spatial/MapExplorer.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <SDL/SDL.h>
#include <SDL/SDL_opengl.h>
#include <SDL/SDL_gfxPrimitives.h>
#include <SDL/SDL_gfxPrimitives_font.h>

using namespace opencog;
using namespace opencog::spatial;


MapExplorer::MapExplorer( Octree3DMapManager* map, unsigned int screenWidth,
    unsigned int screenHeight, bool fullScreen ) 
        throw(opencog::RuntimeException) : 
             map( map ), mapUpdated( false ), screenWidth(screenWidth), screenHeight(screenHeight), 
             halfScreenWidth( screenWidth/2), halfScreenHeight(screenHeight/2),
             fullScreen( fullScreen ), running(false), screen(NULL), 
             leftRightDirection(0), upDownDirection(0),
             cameraTranslation(-((map->xMax( )-map->xMin())/2+map->xMin()),
                 -((map->yMax( )-map->yMin( ))/2+map->yMin()), -50 ),

             turbo(false), yRot(0), zRot(0),floorTextureCheckerId(0),
             floorTextureOccupancyId(0), floorTextureChecker(NULL), 
             floorTextureOccupancy(NULL),  service(NULL)
{

    if ( SDL_Init(SDL_INIT_VIDEO|SDL_INIT_TIMER) < 0 ) {
        throw opencog::RuntimeException( TRACE_INFO, "MapExplorer - Unable to "
            "init SDL: %s", SDL_GetError( ) );
    } // if

    SDL_ShowCursor(0);
}

MapExplorer::~MapExplorer( void ) 
{
    stop( );
    SDL_Quit();
}


void MapExplorer::operator()( void ) throw(opencog::RuntimeException) 
{

    if ( this->screen == NULL ) {
        // initialize screen

        Uint32 flags = SDL_OPENGL|SDL_HWSURFACE;
        if ( this->fullScreen ) {
            flags |= SDL_FULLSCREEN;
        } // if
        this->screen = SDL_SetVideoMode( this->screenWidth, this->screenHeight, 16, flags );
        if ( this->screen == NULL ) {
            throw opencog::RuntimeException( TRACE_INFO, "MapExplorer - Video initialization failed: %s", SDL_GetError( ) );
        } // if

        
        // configure OpenGL
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable (GL_DEPTH_TEST);
        glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );

        glViewport( 0, 0, this->screenWidth, this->screenHeight );        
        
        glMatrixMode( GL_PROJECTION );
        glLoadIdentity();
        resetFrustum( );
        glMatrixMode( GL_MODELVIEW );    

        this->showHelp = false;
        this->mouseRecentered = true;
        SDL_WarpMouse( this->halfScreenWidth, this->halfScreenHeight );

        generateFloorTextures( );

    } // if    
    
    Uint32 maxFps = 60;    
    Uint32 previousTick = SDL_GetTicks( );
    this->updateTime = previousTick;
    this->currentFps = 0;
    Uint32 milisecondsPerFrame = 1000/maxFps;

    // start the main loop
    while (this->running) {
        SDL_Event event;
        while(SDL_PollEvent(&event)) {
            switch( event.type ) {
            case SDL_KEYDOWN:
                keyPressed( event.key.keysym.sym );
                break;

            case SDL_KEYUP:
                keyReleased( event.key.keysym.sym );
                break;

            case SDL_QUIT:
                this->running = false;
                break;

            case SDL_MOUSEMOTION:
                mouseMoved( event.motion.x, event.motion.y );
                break;   

            case SDL_MOUSEBUTTONDOWN:
                if ( event.button.button == SDL_BUTTON_LEFT && !this->showHelp ) {
                    selectRequested = true;
                } // if
                break;               

            default:
                break;
            } // switch
        } // while
        
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glLoadIdentity( );

        if ( this->mapUpdated ) {
            generateFloorTextures( );
            this->mapUpdated = false;
        } // if

        /**
         * configure the axis to the following form:
         *
         *    +z  +x
         *     | /
         *     |/
         *  ---+---+y
         *    /|
         *   / |
         *  
         */
        glRotatef( -90, 1, 0, 0 );
        glRotatef( -90, 0, 0, 1 );

        // do the current user rotation
        glRotatef( yRot, 0, 1, 0 );
        glRotatef( zRot, 0, 0, 1 );
        
        // do the current user translation
        glTranslatef( cameraTranslation.x, cameraTranslation.y, cameraTranslation.z);
        this->running &= update( previousTick );
        SDL_GL_SwapBuffers();

        Uint32 elapsedTime = SDL_GetTicks( ) - previousTick;
        if ( elapsedTime < milisecondsPerFrame ) {
            SDL_Delay( milisecondsPerFrame - elapsedTime );
        } // if
        previousTick = SDL_GetTicks( );
    } // while

    SDL_FreeSurface( this->screen );
    this->screen = NULL;

}

bool MapExplorer::update( long elapsedTime ) 
{
    double STEP = turbo ? 180 : 30;

    math::Vector3 direction( std::cos(zRot/180.0*M_PI), -std::sin(zRot/180.0*M_PI), std::sin(yRot/180.0*M_PI) );
    
    if ( leftRightDirection == -1 ) {
        this->cameraTranslation += math::Vector3::Z_UNIT.crossProduct( direction ) * STEP;
    } else if ( leftRightDirection == 1 ) {
        this->cameraTranslation -= math::Vector3::Z_UNIT.crossProduct( direction ) * STEP;
    } // else if
    
    if ( upDownDirection == -1 ) {
        this->cameraTranslation += (direction * STEP);
    } else if ( upDownDirection == 1 ) {
        this->cameraTranslation -= (direction * STEP);
    } // else if

    { // map operations
        boost::mutex::scoped_lock lock( this->mapUpdateMutex );
        /**
         *    0,0------xMax,0
         *      |      |
         *      |      |
         *      |      |
         *      |      |
         *0,yMax ------xMax,yMax
         */    
        glEnable( GL_TEXTURE_2D );
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
        glBindTexture( GL_TEXTURE_2D, floorTextureId );
        
        glColor4f( 1.0, 1.0, 1.0, 1.0 );
        // draw ground
        glBegin( GL_QUADS );
           glTexCoord2f(0.0, 0.0);
           glVertex3f( map->xMin( ), map->yMin( ), 0 ); // bottom left
           glTexCoord2f(0.0, 1.0);
           glVertex3f( map->xMax( ), map->yMin( ), 0 ); // bottom right
           glTexCoord2f(1.0, 1.0);
           glVertex3f( map->xMax( ), map->yMax( ), 0 ); // top right
           glTexCoord2f(1.0, 0.0);
           glVertex3f( map->xMin( ), map->yMax( ), 0 ); // top left        
        glEnd( );
        glFlush();    
        glDisable( GL_TEXTURE_2D );

        if ( selectRequested ) {
            selectEntity( );
            selectRequested = false;
        } // if
        
        renderEntities( );
        
        renderHUD( );
    } // end block
    return this->running;
}


void MapExplorer::updateMap( Octree3DMapManager* map )
{
    boost::mutex::scoped_lock lock( this->mapUpdateMutex );
    this->map = map;
    if ( this->selectedEntity.length( ) > 0 && 
         !this->map->containsObject( this->selectedEntity ) ) {
        this->selectedEntity = "";
    } // if
    this->mapUpdated = true;
}

void MapExplorer::keyPressed( int key ) 
{
    switch( key ) {
    case SDLK_F1:
        this->showHelp = !this->showHelp;
    break;
    case SDLK_ESCAPE:
        // hide the hud
        this->showHelp = false;
        this->selectedEntity = "";
        break;
    case SDLK_q:
        this->running = false;
        break;
    case SDLK_a: // left
    case SDLK_LEFT:
        this->leftRightDirection = -1;
        break;
    case SDLK_d: // right
    case SDLK_RIGHT:
        this->leftRightDirection = 1;
        break;
    case SDLK_s: // down
    case SDLK_DOWN:
        this->upDownDirection = 1;
        break;
    case SDLK_w: // up
    case SDLK_UP:
        this->upDownDirection = -1;
        break;
        
    case SDLK_f:
        this->floorTextureId = ( this->floorTextureId == this->floorTextureCheckerId ) ?
            floorTextureOccupancyId : this->floorTextureCheckerId;
        break;
    case SDLK_RSHIFT:
    case SDLK_LSHIFT:
        turbo = true;
        break;
    } // switch
}

void MapExplorer::keyReleased( int key ) 
{
    switch( key ) {
    case SDLK_a: // left
    case SDLK_LEFT:
        this->leftRightDirection = 0;
        break;
    case SDLK_d: // right
    case SDLK_RIGHT:
        this->leftRightDirection = 0;
        break;
    case SDLK_s: // down
    case SDLK_DOWN:
        this->upDownDirection = 0;
        break;
    case SDLK_w: // up
    case SDLK_UP:
        this->upDownDirection = 0;
        break;
    case SDLK_RSHIFT:
    case SDLK_LSHIFT:
        turbo = false;
        break;
    } // switch

}

void MapExplorer::mouseMoved( int x, int y ) 
{

    if ( this->mouseRecentered ) {
        this->mouseRecentered = false;
        return;
    } // if

    int xRelative = x - this->halfScreenWidth;
    int yRelative = y - this->halfScreenHeight;   

    yRot += yRelative * 0.15;
    zRot += xRelative * 0.15;
    
    yRot += ( yRot < -360.0 ) ? 360.0 : (yRot > 360.0 ) ? -360.0 : 0.0;
    zRot += ( zRot < -360.0 ) ? 360.0 : (zRot > 360.0 ) ? -360.0 : 0.0;
    

    SDL_WarpMouse( this->halfScreenWidth, this->halfScreenHeight );
    this->mouseRecentered = true;
}


void MapExplorer::start( void ) 
{
    if ( this->running ) { // avoid errors when start is called twice
        return;
    } // if
    this->running = true;
    if ( this->service != NULL ) {
        delete this->service;
    } // if
    this->service = new boost::thread( boost::ref( *this ) );

}

void MapExplorer::wait( void )
{
    if ( this->running && this->service != NULL ) {
        this->service->join( );
    } // if
}

void MapExplorer::stop( void ) 
{
    this->running = false; 
    if ( this->service != NULL ) { // avoid errors when stop is called before start
        this->service->join( );
        delete this->service;
        this->service = NULL;
    } // if
}



void MapExplorer::resetFrustum( void ) 
{
    // planes
    float near = 1.0f;
    float far = 80000.0f;
    
    // frustum aperture angle
    float fovAngle = 60.0f;
    float aspect = this->screenWidth / this->screenHeight;
    
    float top  = std::tan(fovAngle * M_PI/360.0) * near;
    float bottom = -top;
    float left = aspect * bottom;
    float right = aspect * top;
    
    glFrustum(left, right, bottom, top, near, far );        

}

void MapExplorer::generateFloorTextures( void )
{
    bool occupancyActive = this->floorTextureId == this->floorTextureOccupancyId;
    int z = this->map->getFloorHeight();

    if ( this->floorTextureChecker != NULL ) {
        glDeleteTextures(1, &this->floorTextureCheckerId );
        this->floorTextureCheckerId = 0;
        delete [] this->floorTextureChecker;
        glDeleteTextures(1, &this->floorTextureOccupancyId );
        this->floorTextureOccupancyId = 0;  
        delete [] this->floorTextureOccupancy;
    } // if

    this->floorTextureChecker = 
        new unsigned char[this->map->xDim( )*this->map->yDim( )*4];
    this->floorTextureOccupancy = 
        new unsigned char[this->map->xDim( )*this->map->yDim( )*4];
    
    // create procedural textures of a checker and the occupancy grid to 
    // put on the ground
    unsigned int i,j;
    unsigned int checkerGranularity = 1;
    bool rowState = false;
    bool colState = false;
    unsigned char* cursorChecker = floorTextureChecker;
    unsigned char* cursorOccupancy = floorTextureOccupancy;
    for (i = 0; i < (unsigned int)this->map->xDim( ); ++i) {
        rowState = (i%checkerGranularity) == 0 ? !rowState : rowState;
        for (j = 0; j < (unsigned int)this->map->yDim( ); ++j) {
            colState = (j%checkerGranularity) == 0 ? !colState : colState;                
            {
                int c =  this->map->checkStandable( i, j ,z) ? 128 : 255;
                *cursorOccupancy++ = (unsigned char)c;
                *cursorOccupancy++ = (unsigned char)c;
                *cursorOccupancy++ = (unsigned char)c;
                *cursorOccupancy++ = (unsigned char)255;
            }
            {
                int c = rowState == colState ? 255 : 128;
                *cursorChecker++ = (unsigned char)c;
                *cursorChecker++ = (unsigned char)c;
                *cursorChecker++ = (unsigned char)c;
                *cursorChecker++ = (unsigned char)255;                
            }
        } // for
    } // for
 
    { // create textures 
        glDisable(GL_BLEND);
        glEnable(GL_TEXTURE_2D);

        // import textures into OpenGL
        glGenTextures( 1, &this->floorTextureCheckerId );
        glBindTexture( GL_TEXTURE_2D, this->floorTextureCheckerId  );
    
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, this->map->xDim( ), 
                     this->map->yDim( ), 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                     floorTextureChecker );    

        glGenTextures( 1, &this->floorTextureOccupancyId );
        glBindTexture( GL_TEXTURE_2D, this->floorTextureOccupancyId );
        
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, this->map->xDim( ), 
                     this->map->yDim( ), 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                     floorTextureOccupancy );

        this->floorTextureId = occupancyActive ? floorTextureOccupancyId : floorTextureCheckerId;     
    } // end block
}
