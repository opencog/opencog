# - Find Protobuf includes and library
#
# This module defines
#  PROTOBUF_INCLUDE_DIR
#  PROTOBUF_LIBRARIES, the libraries to link against to use PROTOBUF.
#  PROTOBUF_LIB_DIR, the location of the libraries
#  PROTOBUF_FOUND, If false, do not try to use PROTOBUF
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

IF (PROTOBUF_LIBRARIES AND PROTOBUF_INCLUDE_DIR)
    SET(PROTOBUF_FIND_QUIETLY TRUE) # Already in cache, be silent
ENDIF (PROTOBUF_LIBRARIES AND PROTOBUF_INCLUDE_DIR)


FIND_PATH(PROTOBUF_INCLUDE_DIR protobuf
    /usr/include
    /usr/include/google
    /usr/local/include
    /usr/local/include/google
)

FIND_LIBRARY(PROTOBUF_LIBRARY NAMES protobuf PATHS
    /usr/lib
    /usr/local/lib
    /usr/local/lib/google
)

# Copy the results to the output variables.
IF (PROTOBUF_INCLUDE_DIR AND PROTOBUF_LIBRARY)
	SET(PROTOBUF_FOUND 1)
	SET(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARY})
	SET(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIR})
	
	MESSAGE(STATUS "Found these protobuf libs: ${PROTOBUF_LIBRARIES}")
	
ELSE (PROTOBUF_INCLUDE_DIR AND PROTOBUF_LIBRARY)
	SET(PROTOBUF_FOUND 0)
	SET(PROTOBUF_LIBRARIES)
	SET(PROTOBUF_INCLUDE_DIRS)
ENDIF (PROTOBUF_INCLUDE_DIR AND PROTOBUF_LIBRARY)

# Report the results.
IF (NOT PROTOBUF_FOUND)
    SET(PROTOBUF_DIR_MESSAGE "Protobuf was not found. Make sure PROTOBUF_LIBRARY and PROTOBUF_INCLUDE_DIR are set.")
	IF (NOT PROTOBUF_FIND_QUIETLY)
		MESSAGE(STATUS "${PROTOBUF_DIR_MESSAGE}")
	ELSE (NOT PROTOBUF_FIND_QUIETLY)
		IF (PROTOBUF_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${PROTOBUF_DIR_MESSAGE}")
		ENDIF (PROTOBUF_FIND_REQUIRED)
	ENDIF (NOT PROTOBUF_FIND_QUIETLY)
ENDIF (NOT PROTOBUF_FOUND)


MARK_AS_ADVANCED(
    PROTOBUF_INCLUDE_DIRS
    PROTOBUF_LIBRARIES
)
