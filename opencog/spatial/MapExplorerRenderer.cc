/*
 * opencog/spatial/MapExplorerRenderer.cc
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
#include <SDL/SDL.h>
#include <SDL/SDL_opengl.h>
#include <SDL/SDL_gfxPrimitives.h>
#include <SDL/SDL_gfxPrimitives_font.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spatial/MapExplorer.h>
#include <opencog/spatial/3DSpaceMap/Entity3D.h>


using namespace opencog;
using namespace opencog::spatial;

/**
 * This file contains the implementation of the functions
 * that will render the elements on screen
 */

void MapExplorer::selectEntity( void ) 
{
    std::vector<std::string> entities;
    map->findAllEntities(std::back_inserter(entities));
    
    // use the GL_SELECT mode to get the nearest entity
    // that was hit by the cross at the screen center
    unsigned int buffer[entities.size()]; 
    glInitNames();
    glPushName(0);

    glSelectBuffer(entities.size(), buffer );
    glRenderMode(GL_SELECT);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    int viewport[4];
    glGetIntegerv (GL_VIEWPORT, viewport);

    double deltax = 5;
    double deltay = 5;

    // Translate and scale the picked region to the entire window
    glTranslatef((viewport[2] - 2 * ((double)this->halfScreenWidth - viewport[0])) / deltax, 
                 (viewport[3] - 2 * ((double)this->halfScreenHeight - viewport[1])) / deltay, 0);
    glScalef(viewport[2] / deltax, viewport[3] / deltay, 1.0);


    resetFrustum( );


    glMatrixMode(GL_MODELVIEW);
    // give a GL_SELECT name for each entity
    unsigned int i;
    for( i = 0; i < entities.size( ); ++i ) {
        const Entity3D* entity = map->getEntity( entities[i] );
        glPushName(i+1);
        renderEntity( *entity, 0x000000ff );
        glPopName();
    } // for
    
    glMatrixMode(GL_PROJECTION);
    glPopMatrix( );
    glMatrixMode(GL_MODELVIEW);

    int hits = glRenderMode(GL_RENDER);

    // get the nearest entity
    if ( hits > 0 ) {
        unsigned int name;
        unsigned int nearestToCamera = 0xffffffff;
        unsigned int *cursor = buffer;
        unsigned int *chosenEntity = NULL;
        
        for (i = 0; i < (unsigned int)hits; ++i) {
            name = *cursor;
            cursor++;
            if (*cursor < nearestToCamera ) {
                nearestToCamera = *cursor;
                chosenEntity = cursor+2;
            } // if
            cursor += name + 2;
        } // for
        
        const Entity3D* entity = map->getEntity( entities[*chosenEntity - 1] );
        if ( this->selectedEntity == entity->getEntityName( ) ) {
            this->selectedEntity = "";
        } else {
            this->selectedEntity = entity->getEntityName( );
        } // else
        
    } // if    
}

void MapExplorer::renderText(const std::string& text, const math::Rectangle& area, 
   unsigned int color, unsigned int bgColor, bool stretchTexture, bool repeatTexture )
{
    if ( text.length( ) == 0 ) {
        return;
    } // if
    
    // split the text to let a multiline rendering
    std::vector<std::string> lines;
    boost::split( lines, text, boost::is_any_of("\n") );

    // use as width the sentence with more characters
    int width = 0;
    unsigned int i;
    for( i = 0; i < lines.size( ); ++i ) {
        int candidateWidth = (lines[i].length() * 8)+1;
        if ( candidateWidth > width ) {
            width = candidateWidth;
        } // if
    } // for

    // the default font has a height of 8.
    int height = lines.size() * 9;

    SDL_Surface *texture = SDL_CreateRGBSurface(0, width, height, 32, 0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000);
    SDL_FillRect( texture, 0, bgColor );

    gfxPrimitivesSetFont(NULL,-1,-1);

    // print each line on the texture
    for( i = 0; i < lines.size( ); ++i ) {
        stringColor(texture, 1, (i*9)+1, lines[i].c_str( ), color );
    } // for

    glColor4ub( getRed(bgColor), getGreen(bgColor), getBlue(bgColor), getAlpha(bgColor) );

    double factorX = 1.0;
    double factorY = 1.0;

    if ( !stretchTexture ) {
        double areaWidth = std::abs(area.getLeftTopCorner( ).x - area.getRightTopCorner( ).x);
        double areaHeight = std::abs(area.getLeftTopCorner( ).y - area.getLeftBottomCorner( ).y);
        
        factorX = areaWidth / static_cast<double>( width );
        factorY = areaHeight / static_cast<double>( height );
    } // if

    unsigned int textureId;
    
    // Tell GL about our new texture
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, width, height, 0, GL_BGRA, 
                 GL_UNSIGNED_BYTE, texture->pixels );
    
    // GL_NEAREST looks horrible, if scaled...
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );	

    if ( !repeatTexture ) {
        // do not repeat the texture
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );
    } // if

    // prepare to render our texture
    glEnable(GL_TEXTURE_2D);    
    glEnable(GL_BLEND);
    // Draw a quad at location
    glBegin(GL_QUADS);
       glTexCoord2f(factorX, factorY);
       glVertex3f( area.getLeftBottomCorner( ).x, area.getLeftBottomCorner( ).y, area.getLeftBottomCorner( ).z );
       glTexCoord2f(0.0f, factorY); 
       glVertex3f( area.getRightBottomCorner( ).x, area.getRightBottomCorner( ).y, area.getRightBottomCorner( ).z );
       glTexCoord2f(0.0f, 0.0f); 
       glVertex3f( area.getRightTopCorner( ).x, area.getRightTopCorner( ).y, area.getRightTopCorner( ).z );
       glTexCoord2f(factorX, 0.0f); 
       glVertex3f( area.getLeftTopCorner( ).x, area.getLeftTopCorner( ).y, area.getLeftTopCorner( ).z );
    glEnd();
    glDisable(GL_BLEND);

    // Bad things happen if we delete the texture before it finishes
    glFinish();
    glDisable(GL_TEXTURE_2D);
    /// Clean up
    SDL_FreeSurface(texture);
    glDeleteTextures(1, &textureId);
}


void MapExplorer::renderEntities( void ) 
{
    std::vector<std::string> entities;
    map->findAllEntities(std::back_inserter(entities));

    std::list<const Entity3D*> obstacles;
    
    unsigned int i;
    for( i = 0; i < entities.size( ); ++i ) {
        const Entity3D* entity = map->getEntity( entities[i] );

        // draw the entity id above the object
        double radius = (entity->getWidth() + entity->getLength())/ 2.0;
        double lowerZ = entity->getPosition( ).z + entity->getHeight( )/2 + ( radius * 2.0);
        
        double panelWidth = entity->getWidth( ) / 2;
        double panelHeight = panelWidth / 2;

        math::Rectangle panel( 
            math::Vector3( entity->getPosition( ).x, entity->getPosition( ).y - panelWidth/2.0, lowerZ + panelHeight ),
            math::Vector3( entity->getPosition( ).x, entity->getPosition( ).y + panelWidth/2.0, lowerZ + panelHeight ),
            math::Vector3( entity->getPosition( ).x, entity->getPosition( ).y + panelWidth/2.0, lowerZ )
        );
        renderText( entity->getEntityName( ), panel , 0xff0000ff, 0xffffffff, true );


        if ( entity->getIsObstacle() ) {
            obstacles.push_back( entity );
        } else {
            glEnable (GL_BLEND);
            renderEntity( *entity, 0xffff00cc );
        } // else
    } // for

    // obstacles must be rendered after non obstacles
    std::list<const Entity3D*>::const_iterator it;
    for( it = obstacles.begin( ); it != obstacles.end( ); ++it ) {
        glEnable (GL_BLEND);
        renderEntity( **it, 0x0000ff4c );
    } // for
    glDisable(GL_BLEND);

}

void MapExplorer::renderEntity( const Entity3D& entity, unsigned int color )
{
    const std::vector<BlockVector> corners = entity.getBoundingBox( ).getAllCorners( );

    unsigned char red = getRed(color);
    unsigned char green = getGreen(color);
    unsigned char blue = getBlue(color);
    unsigned char alpha = getAlpha(color);

    // draw an orientation pointer
    BlockVector direction = entity.getDirection();
    BlockVector arrowStart = entity.getPosition( );
    math::Vector3 arrowEnd(arrowStart.x + direction.x, arrowStart.y + direction.y, arrowStart.z + direction.z);

    glLineWidth( 50 );
    glBegin( GL_LINES );
      glColor4ub( 1, 1, 1, alpha ); 
      glVertex3d( arrowStart.x, arrowStart.y, arrowStart.z );
      glColor4ub( red, green, blue, alpha );    
      glVertex3d( arrowEnd.x, arrowEnd.y, arrowEnd.z );    
    glEnd( );

    // draw the object itself
    glBegin( GL_TRIANGLES );    
      // back
      glVertex3d( corners[4].x, corners[4].y, corners[4].z );
      glVertex3d( corners[7].x, corners[7].y, corners[7].z );
      glVertex3d( corners[3].x, corners[3].y, corners[3].z );
      
      glVertex3d( corners[4].x, corners[4].y, corners[4].z );
      glVertex3d( corners[3].x, corners[3].y, corners[3].z );
      glVertex3d( corners[0].x, corners[0].y, corners[0].z );          
      // front
      glVertex3d( corners[6].x, corners[6].y, corners[6].z );
      glVertex3d( corners[5].x, corners[5].y, corners[5].z );
      glVertex3d( corners[1].x, corners[1].y, corners[1].z );
      
      glVertex3d( corners[6].x, corners[6].y, corners[6].z );
      glVertex3d( corners[1].x, corners[1].y, corners[1].z );
      glVertex3d( corners[2].x, corners[2].y, corners[2].z );            
      //left
      glVertex3d( corners[5].x, corners[5].y, corners[5].z );
      glVertex3d( corners[4].x, corners[4].y, corners[4].z );
      glVertex3d( corners[0].x, corners[0].y, corners[0].z );
      
      glVertex3d( corners[5].x, corners[5].y, corners[5].z );
      glVertex3d( corners[0].x, corners[0].y, corners[0].z );
      glVertex3d( corners[1].x, corners[1].y, corners[1].z );      
      // right
      glVertex3d( corners[7].x, corners[7].y, corners[7].z );
      glVertex3d( corners[6].x, corners[6].y, corners[6].z );
      glVertex3d( corners[2].x, corners[2].y, corners[2].z );
      
      glVertex3d( corners[3].x, corners[3].y, corners[3].z );
      glVertex3d( corners[2].x, corners[2].y, corners[2].z );
      glVertex3d( corners[7].x, corners[7].y, corners[7].z );            
      // top
      glVertex3d( corners[5].x, corners[5].y, corners[5].z );
      glVertex3d( corners[6].x, corners[6].y, corners[6].z );
      glVertex3d( corners[7].x, corners[7].y, corners[7].z );
      
      glVertex3d( corners[5].x, corners[5].y, corners[5].z );
      glVertex3d( corners[7].x, corners[7].y, corners[7].z );
      glVertex3d( corners[4].x, corners[4].y, corners[4].z );
      // bottom
      glVertex3d( corners[2].x, corners[2].y, corners[2].z );
      glVertex3d( corners[1].x, corners[1].y, corners[1].z );
      glVertex3d( corners[0].x, corners[0].y, corners[0].z );
      
      glVertex3d( corners[2].x, corners[2].y, corners[2].z );
      glVertex3d( corners[0].x, corners[0].y, corners[0].z );
      glVertex3d( corners[3].x, corners[3].y, corners[3].z );     
    glEnd( );
      

    if ( entity.getEntityName()  == this->selectedEntity ) {
        renderEntitySelection( entity );
    } // if

}

void MapExplorer::renderEntitySelection( const Entity3D& entity )
{
    const std::vector<BlockVector> corners = entity.getBoundingBox( ).getAllCorners( );

    // draw the edges of the object if it was selected by the user
    glLineWidth(5);
    glColor3f( 1, 1, 1);
        
    glBegin( GL_LINES );
       // back
       glVertex3d( corners[4].x, corners[4].y, corners[4].z );
       glVertex3d( corners[7].x, corners[7].y, corners[7].z );

       glVertex3d( corners[7].x, corners[7].y, corners[7].z );
       glVertex3d( corners[3].x, corners[3].y, corners[3].z );
          
       glVertex3d( corners[3].x, corners[3].y, corners[3].z );
       glVertex3d( corners[0].x, corners[0].y, corners[0].z );
          
       glVertex3d( corners[0].x, corners[0].y, corners[0].z );
       glVertex3d( corners[4].x, corners[4].y, corners[4].z );          
       // front
       glVertex3d( corners[6].x, corners[6].y, corners[6].z );
       glVertex3d( corners[5].x, corners[5].y, corners[5].z );
       
       glVertex3d( corners[5].x, corners[5].y, corners[5].z );
       glVertex3d( corners[1].x, corners[1].y, corners[1].z );
       
       glVertex3d( corners[1].x, corners[1].y, corners[1].z );
       glVertex3d( corners[2].x, corners[2].y, corners[2].z );
       
       glVertex3d( corners[2].x, corners[2].y, corners[2].z );
       glVertex3d( corners[6].x, corners[6].y, corners[6].z );          
       // top
       glVertex3d( corners[4].x, corners[4].y, corners[4].z );
       glVertex3d( corners[5].x, corners[5].y, corners[5].z );
       
       glVertex3d( corners[6].x, corners[6].y, corners[6].z );
       glVertex3d( corners[7].x, corners[7].y, corners[7].z );          
       // bottom
       glVertex3d( corners[2].x, corners[2].y, corners[2].z );
       glVertex3d( corners[3].x, corners[3].y, corners[3].z );
       
       glVertex3d( corners[1].x, corners[1].y, corners[1].z );
       glVertex3d( corners[0].x, corners[0].y, corners[0].z );
    glEnd( );

}

void MapExplorer::renderHUD( void ) {
    glDisable( GL_LIGHTING ); 
    glMatrixMode( GL_PROJECTION );    
    glPushMatrix( );

    // configure the projection matrix to render 2d
    glLoadIdentity();
    glOrtho(0, this->screenWidth, this->screenHeight, 0, -1, 1 );
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

    if ( SDL_GetTicks( ) - this->updateTime > 1000 ) {
        this->updateTime = SDL_GetTicks( );            
        this->currentFps = this->updateCounter;
        this->updateCounter = 0;
    } // if
    ++this->updateCounter;
        
    std::stringstream header;
    header << "help[F1] quit[q] fps[" << this->currentFps << "]";

    math::Rectangle panel( 
        math::Vector3( header.str().length( )* 8, 0 ), 
        math::Vector3( 0, 0 ), 
        math::Vector3( 0, 10 ) 
    );
    renderText( header.str( ), panel, 0x000000ff, 0xffffffc4, true ); 

    if ( !this->showHelp ) {
        // draw cross at the screen center
        // first the white part
        glColor3f( 1, 1, 1);
        glLineWidth(2);
        glBegin( GL_LINES );        
           glVertex2d( this->halfScreenWidth-8, this->halfScreenHeight );
           glVertex2d( this->halfScreenWidth+8, this->halfScreenHeight );
           glVertex2d( this->halfScreenWidth, this->halfScreenHeight-8 );
           glVertex2d( this->halfScreenWidth, this->halfScreenHeight+8 );
        glEnd( );
        // then the black one
        glColor3f( 0, 0, 0);
        glLineWidth(2);
        glBegin( GL_LINES );        
           glVertex2d( this->halfScreenWidth-10, this->halfScreenHeight );
           glVertex2d( this->halfScreenWidth+10, this->halfScreenHeight );
           glVertex2d( this->halfScreenWidth, this->halfScreenHeight-10 );
           glVertex2d( this->halfScreenWidth, this->halfScreenHeight+10 );
        glEnd( );

        if ( this->selectedEntity.length( ) > 0 ) {
            // draw the current selected entity info
            const Entity3D* entity = map->getEntity( this->selectedEntity );
            std::stringstream info;
            info << std::endl;
            info << "  Entity: " << entity->getEntityName() << std::endl;
            info << std::endl;
            info << "  Obstacle: " << ( entity->getIsObstacle() ? "yes" : "no" ) << std::endl;
            info << std::endl;
            info << "  <Position>" << std::endl;
            info << "  x[" << entity->getPosition( ).x << "]" << std::endl;
            info << "  y[" << entity->getPosition( ).y << "]" << std::endl;
            info << "  z[" << entity->getPosition( ).z << "]" << std::endl;   
            info << std::endl;
            info << "  <Direction>" << std::endl;
            info << "  x[" << entity->getDirection( ).x << "]" << std::endl;
            info << "  y[" << entity->getDirection( ).y << "]" << std::endl;
            info << "  z[" << entity->getDirection( ).z << "]" << std::endl;
//            info << std::endl;
//            info << "  <Dimension>" << std::endl;
//            info << "  width[" << entity->getDimension( ).width << "]"<< std::endl;
//            info << "  height[" << entity->getDimension( ).height << "]"<< std::endl;
//            info << "  length[" << entity->getDimension( ).length << "]"<< std::endl;
            info << std::endl;
            info << "  --------------------------- " << std::endl;
            info << "  Expansion radius: " << (entity->getWidth() + entity->getLength()) / 2 << std::endl;
            info << "  Up axis: +Z" << std::endl;
            info << "  Front axis: +X" << std::endl;
            info << "  Right axis: +Y" << std::endl;

            double panelWidth = 260;
            double panelHeight = 330;
                
            math::Rectangle panel( 
                math::Vector3( this->screenWidth, 0 ), 
                math::Vector3( this->screenWidth-panelWidth, 0 ), 
                math::Vector3( this->screenWidth-panelWidth, panelHeight )
            );
            renderText( info.str( ), panel, 0x000000ff, 0xffffffc4 );
        } // if
    } else {
        // draw the help screen
        this->selectedEntity = "";
        std::stringstream info;
        info << std::endl;
        info << "  Opencog Visual LocalSpaceMap Debugger v1.0" << std::endl;
        info << std::endl;
        info << "  Copyright (C) 2009 Novamente LLC" << std::endl;
        info << "  All Rights Reserved" << std::endl;
        info << "  Author: Samir Araujo <samir.araujo@gmail.com>" << std::endl;
        info << std::endl;
        info << "  ------------------------------------------------" << std::endl;
        info << std::endl;
        info << "  This visualizer can be  used to  check the state" << std::endl;
        info << "  of the agent's  LocalSpaceMap.  It can be useful" << std::endl;
        info << "  for debugging purposes. Use  the  Keys bellow to" << std::endl;
        info << "  control the flying camera." << std::endl;
        info << "  Put  the  cross  (screen center)  on  an  object" << std::endl;
        info << "  and  click  the left mouse button to inspect it." << std::endl;
        info << "  ------------------------------------------------" << std::endl;
        info << std::endl;
        info << "  <Keys>" << std::endl;
        info << "  up|w              = move forward" << std::endl;
        info << "  down|s            = move backward" << std::endl;
        info << "  left|a            = strafe left" << std::endl;
        info << "  right|d           = strafe right" << std::endl;
        info << "  <hold> shift      = motion speed boost" << std::endl;
        info << "  escape            = unselect entity,close help" << std::endl;
        info << "  mouse move        = look at" << std::endl;
        info << "  mouse left button = select entity" << std::endl;
        info << "  f                 = change the floor texture " << std::endl;
        info << "                      (occupancy or checker)" << std::endl;
        info << "  F1                = this help screen" << std::endl;        
        info << "  q                 = exit program" << std::endl;
        
        double panelWidth = 420;
        double panelHeight = 280;

        double horizontalOffset = this->halfScreenWidth - (panelWidth/2.0);
        double verticalOffset = this->halfScreenHeight - (panelHeight/2.0);
        math::Rectangle panel( 
            math::Vector3( horizontalOffset+panelWidth, verticalOffset ),
            math::Vector3( horizontalOffset, verticalOffset ), 
            math::Vector3( horizontalOffset, verticalOffset+panelHeight )
        );
        renderText( info.str( ), panel, 0x000000ff, 0xffffffc4 );
            
    } // else
    
    glMatrixMode( GL_PROJECTION );        
    glPopMatrix();
    glMatrixMode( GL_MODELVIEW );
    
}
