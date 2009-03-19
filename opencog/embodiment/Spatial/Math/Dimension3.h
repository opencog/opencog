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
