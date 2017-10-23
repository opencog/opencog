/*
 * opencog/spatial/VisibilityMap.cc
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

#include <opencog/spatial/VisibilityMap.h>
#include <opencog/spatial/math/Rectangle.h>

#include <opencog/util/Logger.h>
#include <iostream>
#include <fstream>

#include <boost/scoped_ptr.hpp>

using namespace opencog;
using namespace opencog::spatial;

/// Start Class Tile
VisibilityMap::Tile::Tile( double x, double y, int row, int col, const math::Vector3& normal, double tileSideSize ) :
        tileSideSize( tileSideSize ), visible( false ), discoveringTime( 0 ),
        row( row ), col( col ), normal( normal ), center( x, 0, y ) { }

const math::Vector3& VisibilityMap::Tile::getNormal( void )
{
    return this->normal;
}

const math::Vector3& VisibilityMap::Tile::getCenter( void )
{
    return this->center;
}

std::vector<math::Vector3> VisibilityMap::Tile::getCorners( void )
{
    std::vector<math::Vector3> corners;

    corners.push_back( math::Vector3( this->center.x - tileSideSize / 2, 0, this->center.z + tileSideSize / 2 ) );
    corners.push_back( math::Vector3( this->center.x + tileSideSize / 2, 0, this->center.z + tileSideSize / 2 ) );
    corners.push_back( math::Vector3( this->center.x + tileSideSize / 2, 0, this->center.z - tileSideSize / 2 ) );
    corners.push_back( math::Vector3( this->center.x - tileSideSize / 2, 0, this->center.z - tileSideSize / 2 ) );

    return corners;
}

void VisibilityMap::Tile::setVisibility( bool visible )
{
    this->visible = visible;
}

bool VisibilityMap::Tile::isVisible( void )
{
    return this->visible;
}

int VisibilityMap::Tile::getRow( void ) const
{
    return this->row;
}

int VisibilityMap::Tile::getCol( void ) const
{
    return this->col;
}


/// End Class Tile

/// Start Class TileVisitor

VisibilityMap::TileVisitor::TileVisitor( unsigned int areaNumber, unsigned int numberOfAreas )
{

    if ( numberOfAreas == 0 ) {
        throw opencog::InvalidParamException( "Visibility Map - The number of areas should be greater than 0 and lesser then tileSide. numberOfAreas[%d] ", TRACE_INFO, numberOfAreas );
    } // if

    this->areaNumber = areaNumber;
    this->numberOfAreas = numberOfAreas;
}

const VisibilityMap::TilePtr& VisibilityMap::TileVisitor::getLastValidTile( void ) const
{
    return this->lastValidTile;
}

VisibilityMap::VisibleTileVisitor::VisibleTileVisitor( unsigned int areaNumber, unsigned int numberOfAreas )
{
}

bool VisibilityMap::VisibleTileVisitor::operator( )( const VisibilityMap::TilePtr& tile )
{
    bool result = tile->isVisible( );
    if ( result ) {
        this->lastValidTile = tile;
    } // if
    return result;
}

VisibilityMap::HiddenTileVisitor::HiddenTileVisitor( unsigned int areaNumber, unsigned int numberOfAreas )
{
}

bool VisibilityMap::HiddenTileVisitor::operator( )( const VisibilityMap::TilePtr& tile )
{
    bool result = !tile->isVisible( );
    if ( result ) {
        this->lastValidTile = tile;
    } // if
    return result;
}


VisibilityMap::NearestTileVisitor::NearestTileVisitor( const spatial::math::Vector3& referencePosition, bool visibility, unsigned int areaNumber, unsigned int numberOfAreas ) :
        TileVisitor( areaNumber, numberOfAreas ), referencePosition( referencePosition ),
        visibility( visibility ), currentDistance( std::numeric_limits<double>::max( ) )
{
}

bool VisibilityMap::NearestTileVisitor::operator( )( const VisibilityMap::TilePtr& tile )
{

    // it will always return false to let visitor test all tiles before 
    // saying that there are one which are the nearest
    if ( tile->isVisible( ) != this->visibility ) {
        return false;
    } // if

    if ( lastValidTile == NULL ) {
        this->lastValidTile = tile;
        this->currentDistance = ( this->lastValidTile->getCenter( ) - referencePosition ).length( );
        //return true;
        return false;
    } // if

    double candidateDistance = ( tile->getCenter( ) - referencePosition ).length( );
    if ( candidateDistance < this->currentDistance ) {
        this->currentDistance = candidateDistance;
        this->lastValidTile = tile;
        //return true;
        return false;
    } // if
    return false;
}


/// End Class TileVisitor

/// Start Class VisibilityMap
VisibilityMap::VisibilityMap( const math::Vector3& minimumExtent, const math::Vector3& maximumExtent, unsigned int numberOfTiles ) : numberOfTiles( numberOfTiles ), minimumExtent( minimumExtent ), maximumExtent( maximumExtent )
{

    unsigned int x;
    unsigned int y;

    this->tileSideSize = ( maximumExtent.x - minimumExtent.x ) / static_cast<double>( numberOfTiles );

    this->tiles.reserve( numberOfTiles );
    unsigned int tilesCounter = 0;
    for ( y = 0; y < numberOfTiles; ++y ) {
        std::vector<VisibilityMap::TilePtr> rowTiles;
        rowTiles.reserve( numberOfTiles );
        for ( x = 0; x < numberOfTiles; ++x ) {
            double centerX = minimumExtent.x + (x * tileSideSize) + tileSideSize / 2;
            double centerY = minimumExtent.z + (y * tileSideSize) + tileSideSize / 2;
            ++tilesCounter;
            rowTiles.push_back( TilePtr( new VisibilityMap::Tile( centerX, centerY, y, x, math::Vector3( 0, 1, 0 ), this->tileSideSize ) ) );
        } // for
        tiles.push_back( rowTiles );
    } // for

    logger().debug("VisibilityMap - Created tiles #: %d", tilesCounter );

    logger().debug("VisibilityMap - Max extents: min[%s] max[%s]", minimumExtent.toString( ).c_str( ), maximumExtent.toString( ).c_str( ) );
    logger().debug("VisibilityMap - Total rows created: %d", tiles.size( ) );
    logger().debug("VisibilityMap - #number of tiles per side: %d, tile side size: %f", numberOfTiles, tileSideSize );
}

const VisibilityMap::TilePtr& VisibilityMap::getTile( const math::Vector3& position ) const
{
    if ( position.x >= this->minimumExtent.x &&
            position.x <= this->maximumExtent.x &&
            position.z >= this->minimumExtent.z &&
            position.z <= this->maximumExtent.z ) {
        // position is inside map
        double xOffset = std::fabs( position.x - this->minimumExtent.x );
        double zOffset = std::fabs( position.z - this->minimumExtent.z );

        int col = ((int)std::ceil( xOffset / tileSideSize )) - 1;
        int row = ((int)std::ceil( zOffset / tileSideSize )) - 1;

        return getTile( row, col );
    } // if
    throw opencog::NotFoundException( "Visibility Map - There is no tile at position[%s]", TRACE_INFO, position.toString( ).c_str( ) );
}

const VisibilityMap::TilePtr& VisibilityMap::getTile( unsigned int row, unsigned int column ) const
{
    if ( this->tiles.size( ) > row && this->tiles[row].size( ) > column ) {
        return this->tiles[row][column];
    } // if
    throw opencog::NotFoundException( "Visibility Map - There is no tile at row[%d] column[%d]", TRACE_INFO, row, column );
}

void VisibilityMap::resetTiles( void )
{
    unsigned int row;
    unsigned int col;
    for ( row = 0; row < this->tiles.size( ); ++row ) {
        std::vector<VisibilityMap::TilePtr>& rowTiles = this->tiles[row];
        for ( col = 0; col < rowTiles.size( ); ++col ) {
            rowTiles[col]->setVisibility( false );
        } // for
    } // for
}

bool VisibilityMap::hasHiddenTile( void )
{
    try {
        getNextHiddenTile( );
        return true;
    } catch ( ... ) {
        return false;
    } // catch
}

void  VisibilityMap::visitTiles( VisibilityMap::TileVisitor* visitor )
{
    unsigned int areasPerSide = static_cast<unsigned int>( std::sqrt( visitor->getNumberOfAreas( ) ) );
    unsigned int cellsPerAreaSide = this->tiles.size( ) / areasPerSide;

    unsigned int row;
    unsigned int col;
    unsigned int startCol = 0;
    unsigned int startRow = 0;
    unsigned int endRow = this->tiles.size( );
    unsigned int endCol = endRow;

    // TODO: FIX the case when numberOfAreas == 1
    if ( visitor->getNumberOfAreas( ) > 1 ) {
        startCol = ( visitor->getAreaNumber( ) % areasPerSide ) * cellsPerAreaSide;
        //startRow = ( areasPerSide - ( visitor->getAreaNumber( ) / areasPerSide ) ) * cellsPerAreaSide;
        startRow = ( visitor->getAreaNumber( ) / areasPerSide ) * cellsPerAreaSide;
        endRow = startRow + cellsPerAreaSide;
        endCol = startCol + cellsPerAreaSide;
    } // if

    //std::cout << " AreaNumber: " << visitor->getAreaNumber( ) << " NumberOfAreas: " << visitor->getNumberOfAreas( ) << " StartRow: " << startRow << " StartCol: " << startCol << " EndRow: " << endRow << " EndCol: " << endCol << " AreasPerSide: " << areasPerSide << " CellsPerAreaSide: " << cellsPerAreaSide << " NumberOfTiles: " << this->tiles.size( ) << std::endl;

    for ( row = startRow; row < endRow; ++row ) {
        std::vector<VisibilityMap::TilePtr>& rowTiles = this->tiles[row];
        for ( col = startCol; col < endCol; ++col ) {
            //std::cout << "Col: " << col << " Row: " << row << " r: " << (*visitor)( rowTiles[col] ) << std::endl;
            if ( (*visitor)( rowTiles[col] ) ) {
                return;
            } // if
        } // for
    } // for
}

const VisibilityMap::TilePtr& VisibilityMap::getNextHiddenTile( unsigned int areaNumber, unsigned int numberOfAreas )
{

    HiddenTileVisitor* visitor = new HiddenTileVisitor( areaNumber, numberOfAreas );
    visitTiles( visitor );
    const TilePtr& tile = visitor->getLastValidTile( );
    delete visitor;
    if ( tile.get( ) == NULL ) {
        throw opencog::NotFoundException( "Visibility Map - There is no hidden tiles at the given area", TRACE_INFO );
    } // if
    return tile;
}

unsigned int VisibilityMap::getNumberOfTiles( void ) const
{
    return this->numberOfTiles;
}

double VisibilityMap::getTileSideSize( void ) const
{
    return this->tileSideSize;
}

const VisibilityMap::TilePtr& VisibilityMap::getNextVisibleTile( unsigned int areaNumber, unsigned int numberOfAreas )
{
    VisibleTileVisitor* visitor = new VisibleTileVisitor( areaNumber, numberOfAreas );
    visitTiles( visitor );
    const TilePtr& tile = visitor->getLastValidTile( );
    delete visitor;
    if ( tile.get( ) == NULL ) {
        throw opencog::NotFoundException( "Visibility Map - There is no visible tiles at the given area", TRACE_INFO );
    } // if
    return tile;
}

const VisibilityMap::TilePtr&  VisibilityMap::getNearestHiddenTile( const spatial::math::Vector3& referencePosition, unsigned int areaNumber, unsigned int numberOfAreas )
{
    NearestTileVisitor* visitor = new NearestTileVisitor( referencePosition, false, areaNumber, numberOfAreas );
    visitTiles( visitor );
    const TilePtr& tile = visitor->getLastValidTile( );
    delete visitor;
    if ( tile.get( ) == NULL ) {
        throw opencog::NotFoundException( "Visibility Map - There is no visible tiles at the given area", TRACE_INFO );
    } // if
    return tile;
}

const VisibilityMap::TilePtr&  VisibilityMap::getNearestVisibleTile( const spatial::math::Vector3& referencePosition, unsigned int areaNumber, unsigned int numberOfAreas )
{
    NearestTileVisitor* visitor = new NearestTileVisitor( referencePosition, true, areaNumber, numberOfAreas );
    visitTiles( visitor );
    const TilePtr& tile = visitor->getLastValidTile( );
    delete visitor;
    if ( tile.get( ) == NULL ) {
        throw opencog::NotFoundException( "Visibility Map - There is no visible tiles at the given area", TRACE_INFO );
    } // if
    return tile;
}

spatial::math::Vector3 VisibilityMap::getAreaCenter( unsigned int areaNumber, unsigned int numberOfAreas )
{
    unsigned int areasPerSide = static_cast<unsigned int>( std::sqrt( numberOfAreas ) );
    unsigned int cellsPerAreaSide = this->tiles.size( ) / areasPerSide;

    unsigned int startCol = ( areaNumber % areasPerSide ) * cellsPerAreaSide;
    //unsigned int startRow = ( areasPerSide - (areaNumber / areasPerSide) ) * cellsPerAreaSide;
    unsigned int startRow = ( areaNumber / areasPerSide ) * cellsPerAreaSide;

    //  std::cout << "AreaNumber: "  << areaNumber << " # of areas: " << numberOfAreas << " StartCol: " << startCol << " StartRow: " << startRow << std::endl;
    unsigned int middleCell = cellsPerAreaSide / 2;
    return getTile( startRow + middleCell, startCol + middleCell )->getCenter( );
}

bool VisibilityMap::isInsideArea( const spatial::Entity& entity, unsigned int areaNumber, unsigned int numberOfAreas )
{

    unsigned int areasPerSide = static_cast<unsigned int>( std::sqrt( numberOfAreas ) );
    unsigned int cellsPerAreaSide = this->tiles.size( ) / areasPerSide;

    unsigned int startCol = 0;
    unsigned int startRow = 0;
    unsigned int endRow = this->tiles.size( ) - 1;
    unsigned int endCol = endRow;

    // TODO: FIX the case when numberOfAreas == 1
    if ( numberOfAreas > 1 ) {
        startCol = ( areaNumber % areasPerSide ) * cellsPerAreaSide;
        //startRow = ( areasPerSide - ( areaNumber / areasPerSide ) ) * cellsPerAreaSide;
        startRow = ( areaNumber / areasPerSide ) * cellsPerAreaSide;
        endRow = startRow + cellsPerAreaSide;
        endCol = startCol + cellsPerAreaSide;
    } // if

    //std::cout << "TargetPosition: " << entity.getPosition( ).toString( ) << " AreaNumber: " << areaNumber << " NumberOfAreas: " << numberOfAreas << " StartRow: " << startRow << " StartCol: " << startCol << " EndRow: " << endRow << " EndCol: " << endCol << " AreasPerSide: " << areasPerSide << " CellsPerAreaSide: " << cellsPerAreaSide << " NumberOfTiles: " << this->tiles.size( ) << std::endl;

    spatial::math::Vector3 center1 = getTile( startRow, startCol )->getCenter( );
    spatial::math::Vector3 center2 = getTile( endRow, endCol )->getCenter( );

    center1 -= tileSideSize / 2;
    center2 += tileSideSize / 2;

    center1.y = 0;
    center2.y = 0;

    //  std::cout << center1.toString( ) << std::endl;
    //  std::cout << center2.toString( ) << std::endl;
    //  std::cout << entity.getPosition( ).toString( ) << std::endl;
    //  std::cout << std::endl;

    math::Rectangle boundings( center1, spatial::math::Vector3( center2.x, 0, center1.z ), center2 );
    const std::vector<math::LineSegment>& corners = entity.getBoundingBox( ).getAllEdges( );
    // TODO: change default OAC coordinate systems to x, z, y instead of using just x and y
    unsigned int i;
    for ( i = 0; i < 4; ++i ) {
        if ( boundings.isInside( math::Vector3( corners[i].pointA.x, 0, corners[i].pointA.y ) ) ) {
            return true;
        } // if
    } // for
    return boundings.isInside( math::Vector3( entity.getPosition( ).x, 0, entity.getPosition( ).y ) );
}

const VisibilityMap::TilePtr& VisibilityMap::getNearestVisibleTileToPosition( const spatial::math::Vector3& referencePosition )
{

    const TilePtr& tile = getTile( referencePosition );
    if ( tile.get( ) == NULL ) {
        throw opencog::NotFoundException( "Visibility Map - The given position[%s] does not correspond to a tile.", TRACE_INFO, referencePosition.toString( ).c_str( ) );
    } // if

    if ( tile->isVisible( ) ) {
        return tile;
    } // if

    int step = 1;

    int direction = 3; // 0 - right | 1 - top | 2 - left | 3 - down

    int currRow = tile->getRow( );
    int currCol = tile->getCol( );

    int maxTiles = static_cast<int>( tiles.size( ) );

    int leftSteps = 1;
    int* current = &currCol;
    int signal = 0;

    while ( std::abs(step) < (maxTiles*2) + 1 ) {
        if ( leftSteps == 0 ) {
            switch ( direction ) {
            case 0: {
                direction = 1;
                current = &currRow;
                signal = 1;
            }
            break;
            case 1: {
                direction = 2;
                current = &currCol;
                signal = -1;
            }
            break;
            case 2: {
                direction = 3;
                current = &currRow;
                signal = -1;
            }
            break;
            case 3: {
                direction = 0;
                current = &currCol;
                signal = 1;
            }
            break;
            };

            leftSteps = step;

            if ( (direction % 2) == 1 ) {
                ++step;
            } // if

        } // if

        *current += signal;
        --leftSteps;

        if ( currRow < 0 || currCol < 0 || currRow > maxTiles - 1 || currCol > maxTiles - 1 ) {
            continue;
        } // if

        const TilePtr& currentTile = getTile( currRow, currCol );
        if ( currentTile->isVisible( ) ) {
            return currentTile;
        } // if
    } // while

    throw opencog::NotFoundException( "Visibility Map - There is no visible tiles near to the reference point[%s]", TRACE_INFO, referencePosition.toString( ).c_str( ) );
}

bool VisibilityMap::saveToFile( const std::string& fileName, const VisibilityMap& visMap )
{
    std::ofstream file( fileName.c_str( ), std::ios::trunc );
    if ( !file.is_open( ) ) {
        return false;
    } // if
    file << visMap.minimumExtent.toString( );
    file << " ";
    file << visMap.maximumExtent.toString( );
    file << " ";
    file << visMap.tiles.size( );
    file << " ";
    unsigned int row;
    unsigned int col;
    for ( row = 0; row < visMap.tiles.size( ); ++row ) {
        for ( col = 0; col < visMap.tiles[row].size( ); ++col ) {
            file << visMap.tiles[row][col]->isVisible( ) << " ";
        } // for
    } // for
    file.close( );
    return true;
}

VisibilityMapPtr VisibilityMap::loadFromFile( const std::string& fileName )
{

    std::ifstream file( fileName.c_str( ) );

    if ( !file.is_open( ) ) {
        throw opencog::NotFoundException( "VisibilityMap::loadFromFile - Cannot open file[%s]", TRACE_INFO, fileName.c_str( ) );
    } // if

    spatial::math::Vector3 minExtent;
    spatial::math::Vector3 maxExtent;
    unsigned int tilesSide;
    file >> minExtent.x >> minExtent.y >> minExtent.z;
    file >> maxExtent.x >> maxExtent.y >> maxExtent.z;

    //  std::cout << minExtent.toString( ) << " - " << maxExtent.toString( ) << std::endl;

    file >> tilesSide;
    VisibilityMapPtr visMap( new VisibilityMap( minExtent, maxExtent, tilesSide ) );


    unsigned int row;
    unsigned int col;
    for ( row = 0; row < tilesSide; ++row ) {
        for ( col = 0; col < tilesSide; ++col ) {
            bool visibility = false;
            file >> visibility;
            visMap->tiles[row][col]->setVisibility( visibility );
        } // for
    } // for

    file.close( );
    return visMap;
}



/// End Class VisibilityMap
