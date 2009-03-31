/*
 * opencog/embodiment/Spatial/Math/Dimension2.h
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
#ifndef DIMENSION2_H
#define DIMENSION2_H

#include "Prerequisites.h"
#include <sstream>

namespace Spatial {
  namespace Math {

    /**
     * class Dimension2
     * Dimension of a bidimensional object
     */
    class Dimension2 {
    public:

      inline Dimension2( float width = 1, float height = 1 ) throw ( opencog::InvalidParamException ) :
	width( width ), height( height ) { 
	if ( width < 0 || height < 0 ) {
	  throw opencog::InvalidParamException( TRACE_INFO, "Invalid dimension [negative parameter(s)]" );
	} // if
      }

      inline virtual ~Dimension2( ) { }

      inline bool operator==( const Dimension2& dimension ) const {
	return ( width == dimension.width && height == dimension.height );
      };

      inline std::string toString( void ) const {
	std::ostringstream response;
	response << width << " " << height;
	return response.str( );
      }

      
      float width;
      float height;
    };

  }; // Math
}; // Spatial

#endif // DIMENSION2_H
