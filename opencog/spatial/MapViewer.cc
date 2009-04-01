/*
 * opencog/spatial/MapViewer.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
 * All Rights Reserved
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
#include <iostream>
#include "Renderer.h"
#include "HPASearch.h"
#include "VisibilityMap.h"

using namespace Spatial;

int main( int argc, char* argv[] ) {
  
  if ( argc < 11 ) {
    std::cout << "Usage: " << argv[0] << "xMin xMax yMin yMax xDim yDim agentRadius fileName.bmp mapFileName hpaClusters [vismap.bin]" << std::endl;
    std::cout << "xMin - Lesser map coord on axis X" << std::endl;
    std::cout << "xMax - Greater map coord on axis X" << std::endl;
    std::cout << "yMin - Lesser map coord on axis Y" << std::endl;
    std::cout << "yMax - Greater map coord on axis Y" << std::endl;
    std::cout << "xDim - Number of grid columns" << std::endl;
    std::cout << "yMax - Number of grid rows" << std::endl;
    std::cout << "agentRadius - The radius of the agent" << std::endl;
    std::cout << "mapFileName - The name of the LocalSpaceMap binary file" << std::endl;
    std::cout << "fileName.bmp - The name of the image file which will be generated" << std::endl;
    std::cout << "hpaClusters - The number of clusters to be printed inside the map (0 if hpa clusters must not be printed)" << std::endl;
    std::cout << "[vismap.bin] - Optional parameter. Visibility map binary file" << std::endl;

    std::cout << "Example1: " << argv[0] << " 165000.000 -37000.000 142000.0 270000.0 256 256 111.803 image.bmp map.bin 0" << std::endl;
    std::cout << "Example2: " << argv[0] << " -134000.000 -70000.000 186000.0 250000.0 256 256 111.803 image.bmp map.bin 16 vismap.bin" << std::endl;
    return 0;
  } // if

  float xMin = atof( argv[1] );
  float xMax = atof( argv[2] );
  float yMin = atof( argv[3] );
  float yMax = atof( argv[4] );
  unsigned int xDim = atoi( argv[5] );
  unsigned int yDim = atoi( argv[6] );
  float petRadius = atof( argv[7] );
  std::string imageFileName = argv[8];
  std::string mapFile = argv[9];
  unsigned int hpaClusters = atoi( argv[10] );
  std::string visMapFileName = ( argc == 12 ) ? argv[11] : "";
  
  try {

    Spatial::LocalSpaceMap2D map( xMin, xMax, xDim, yMin, yMax, yDim, petRadius );

    FILE* loadFile = fopen( mapFile.c_str( ), "r+b" );
    if ( !loadFile ) {
      throw opencog::NotFoundException( TRACE_INFO, "File not found: %s", mapFile.c_str( ) );
    } // if
    map.load( loadFile );
    fclose( loadFile );

    Renderer renderer;

    if ( visMapFileName.length( ) > 0 ) {
      VisibilityMapPtr visMap( VisibilityMap::loadFromFile( visMapFileName ) );
      renderer.renderVisMap( visMap.get( ) );
    } // if

    renderer.renderMap( &map, hpaClusters );

    renderer.saveImageFile( imageFileName );

  } catch( opencog::NotFoundException& ex ) {
    std::cerr << ex.getMessage( ) << std::endl;
  } catch( opencog::RuntimeException& ex ) {
    std::cerr << ex.getMessage( ) << std::endl;
  } // catch
  return 0;
}
