#
# Locate Xerces-C include paths and libraries
# Xerces-C can be found at http://xml.apache.org/xerces-c/

# This module defines
# XERCESC_INCLUDE_DIR, where to find ptlib.h, etc.
# XERCESC_LIBRARIES, the libraries to link against to use pwlib.
# XERCESC_FOUND, If false, don't try to use pwlib.

FIND_PATH(XERCESC_INCLUDE_DIR xercesc/dom/DOM.hpp
  "[HKEY_CURRENT_USER\\software\\xerces-c\\src]"
  "[HKEY_CURRENT_USER\\xerces-c\\src]"
  $ENV{XERCESCROOT}/src/
  /usr/local/include
  /usr/include
)
	
FIND_LIBRARY(XERCESC_LIBRARIES
  NAMES
    xerces-c
  PATHS
    "[HKEY_CURRENT_USER\\software\\xerces-c\\lib]"
    "[HKEY_CURRENT_USER\\xerces-c\\lib]"
    $ENV{XERCESCROOT}/lib
    /usr/local/lib
    /usr/lib
)
	
# if the include a the library are found then we have it
SET( XERCESC_FOUND 0 )
IF(XERCESC_INCLUDE_DIR)
  IF(XERCESC_LIBRARIES)
    SET( XERCESC_FOUND 1 )
  ENDIF(XERCESC_LIBRARIES)
ENDIF(XERCESC_INCLUDE_DIR)

MARK_AS_ADVANCED(
  XERCESC_INCLUDE_DIR
  XERCESC_LIBRARIES
) 