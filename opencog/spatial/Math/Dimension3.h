/*
 * opencog/spatial/Math/Dimension3.h
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
#ifndef DIMENSION3_H
#define DIMENSION3_H

#include "Dimension2.h"

namespace Spatial {
  namespace Math {

    /**
     * class Dimension3
     * Representation of a Bounding Box (3-dimensions)
     */
    class Dimension3 : public Dimension2 {
    public:

      inline Dimension3( float width = 1, float height = 1, float length = 1 ) throw (opencog::InvalidParamException):
	Dimension2( width, height ), length( length ) { 
	if ( length < 0 ) {
	  throw opencog::InvalidParamException( TRACE_INFO, "Invalid dimension [negative length]" );
	} // if
      }

      inline bool operator==( const Dimension3& dimension ) const {
	return ( width == dimension.width && 
		 height == dimension.height && 
		 length == dimension.length );
      };

      inline virtual ~Dimension3( ) { }

      inline std::string toString( void ) const {
	std::ostringstream response;
	response << width << " " << height << " " << length;
	return response.str( );
      }


      float length;
    };

  }; // Math
}; // Spatial

#endif // DIMENSION3_H
