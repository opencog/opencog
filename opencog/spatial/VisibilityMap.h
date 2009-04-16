/*
 * opencog/spatial/VisibilityMap.h
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

#ifndef _SPATIAL_VISIBILITY_MAP_H_
#define _SPATIAL_VISIBILITY_MAP_H_

#include "Math/Vector3.h"
#include "Entity.h"
#include <opencog/util/exceptions.h>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace Spatial
{

/**
 * Visibility map is a class to keep track of the already visible areas.
 * When the agent is exploring new areas, the tiles corresponding to these areas
 * is been marked as visible.
 */
class VisibilityMap;
typedef boost::shared_ptr<VisibilityMap> VisibilityMapPtr;

class VisibilityMap
{

public:

    /**
     * A tile covers a specific area of the VisibilityMap
     */
    class Tile
    {
    public:

        Tile( double x, double y, int row, int col, const Math::Vector3& normal, double tileSideSize );

        inline virtual ~Tile( void ) { };

        std::vector<Math::Vector3> getCorners( void );

        void setVisibility( bool visible );

        bool isVisible( void );

        const Math::Vector3& getNormal( void );

        const Math::Vector3& getCenter( void );

        int getRow( void ) const;

        int getCol( void ) const;

    private:
        double tileSideSize;
        bool visible;
        long discoveringTime;
        int row;
        int col;
        Math::Vector3 normal;
        Math::Vector3 center;
    }; // Tile
    typedef boost::shared_ptr<Tile> TilePtr;



    class TileVisitor
    {
    public:

        TileVisitor( unsigned int areaNumber = 1, unsigned int numberOfAreas = 1 ) throw (opencog::InvalidParamException);

        inline virtual ~TileVisitor( void ) { };

        // visitor method
        virtual bool operator( )( const TilePtr& tile ) = 0;

        inline unsigned int getAreaNumber( void ) const {
            return this->areaNumber;
        }

        inline unsigned int getNumberOfAreas( void ) const {
            return this->numberOfAreas;
        }

        const TilePtr& getLastValidTile( void ) const;

    protected:

        unsigned int areaNumber;
        unsigned int numberOfAreas;
        TilePtr lastValidTile;
    }; // TileVisitor

    class HiddenTileVisitor : public TileVisitor
    {
    public:
        HiddenTileVisitor( unsigned int areaNumber = 1, unsigned int numberOfAreas = 1 ) throw (opencog::InvalidParamException);
        virtual ~HiddenTileVisitor( void ) { }

        virtual bool operator( )( const TilePtr& tile );
    }; // HiddenTileVisitor

    class VisibleTileVisitor : public TileVisitor
    {
    public:
        VisibleTileVisitor( unsigned int areaNumber = 1, unsigned int numberOfAreas = 1 ) throw (opencog::InvalidParamException);
        virtual ~VisibleTileVisitor( void ) { }

        virtual bool operator( )( const TilePtr& tile );
    }; // VisibleTileVisitor


    class NearestTileVisitor : public TileVisitor
    {
    public:
        // look for nearest tiles to a given referencePosition and which has a given visibility
        NearestTileVisitor( const Spatial::Math::Vector3& referencePosition, bool visibility, unsigned int areaNumber = 1, unsigned int numberOfAreas = 1 ) throw (opencog::InvalidParamException);

        virtual ~NearestTileVisitor( void ) { }

        virtual bool operator( )( const TilePtr& tile );

    private:
        Spatial::Math::Vector3 referencePosition;
        bool visibility;
        double currentDistance;
    }; // NearestTileVisitor


    const TilePtr& getTile( const Math::Vector3& position ) const throw( opencog::NotFoundException );

    const TilePtr& getTile( unsigned int row, unsigned int column ) const throw( opencog::NotFoundException );


    /**
                   row,col(n,n)
       +-----------+ (maxExtent)
       |           |
       |           |
       |           |
       |           |
       |(minExtent)|
       +-----------+
    row,col(0,0)
     */
    VisibilityMap( const Math::Vector3& minimumExtent, const Math::Vector3& maximumExtent, unsigned int numberOfTiles );

    inline ~VisibilityMap( void ) { };

    void resetTiles( void );

    bool hasHiddenTile( void );

    TilePtr& nextHiddenTile( void );

    unsigned int getNumberOfTiles( void ) const;

    double getTileSideSize( void ) const;

    void visitTiles( TileVisitor* visitor );

    inline unsigned int getNumberOfTilesPerRow( void ) const {
        return this->tiles.size( );
    };


    /**
     * area number
     * +-+-+-+
     * |0|1|2|
     * +-+-+-+
     * |3|4|5|
     * +-+-+-+
     * |6|7|n|
     */
    const TilePtr& getNextHiddenTile( unsigned int areaNumber = 0, unsigned int numberOfAreas = 1 ) throw (opencog::NotFoundException, opencog::InvalidParamException);

    const TilePtr& getNextVisibleTile( unsigned int areaNumber = 0, unsigned int numberOfAreas = 1 ) throw (opencog::NotFoundException, opencog::InvalidParamException);

    const TilePtr& getNearestHiddenTile( const Spatial::Math::Vector3& referencePosition, unsigned int areaNumber = 0, unsigned int numberOfAreas = 1 ) throw (opencog::NotFoundException);

    const TilePtr& getNearestVisibleTile( const Spatial::Math::Vector3& referencePosition, unsigned int areaNumber = 0, unsigned int numberOfAreas = 1 ) throw (opencog::NotFoundException);

    Spatial::Math::Vector3 getAreaCenter( unsigned int areaNumber, unsigned int numberOfAreas ) throw (opencog::NotFoundException);

    bool isInsideArea( const Spatial::Entity& entity, unsigned int areaNumber = 0, unsigned int numberOfAreas = 1 ) throw (opencog::NotFoundException);

    const TilePtr& getNearestVisibleTileToPosition( const Spatial::Math::Vector3& referencePosition ) throw (opencog::NotFoundException);

    static bool saveToFile( const std::string& fileName, const VisibilityMap& map );

    static VisibilityMapPtr loadFromFile( const std::string& fileName ) throw( opencog::NotFoundException );

private:
    unsigned int numberOfTiles;
    double tileSideSize;
    std::vector< std::vector<TilePtr> > tiles;
    Math::Vector3 minimumExtent;
    Math::Vector3 maximumExtent;
}; // VisibilityMap

} // Spatial

#endif // _SPATIAL_VISIBILITY_MAP_H_
