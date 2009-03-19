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
