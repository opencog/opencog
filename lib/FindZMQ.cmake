# - Find ZeroMQ library
#
# This module defines
#  ZMQ_LIBRARIES, the libraries to link against to use ZMQ.
#  ZMQ_LIB_DIR, the location of the libraries
#  ZMQ_FOUND, If false, do not try to use ZMQ
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

IF (ZMQ_LIBRARIES)
   SET(ZMQ_FIND_QUIETLY TRUE) # Already in cache, be silent
ENDIF (ZMQ_LIBRARIES)

FIND_LIBRARY(ZMQ_LIBRARY NAMES zmq PATHS
    /usr/lib
    /usr/local/lib
    /usr/local/lib/zmq
)

# Copy the results to the output variables.
IF (ZMQ_LIBRARY)
	SET(ZMQ_FOUND 1)
	SET(ZMQ_LIBRARIES ${ZMQ_LIBRARY})
	
	MESSAGE(STATUS "Found these zmq libs: ${ZMQ_LIBRARIES}")
	
ELSE (ZMQ_LIBRARY)
	SET(ZMQ_FOUND 0)
	SET(ZMQ_LIBRARIES)
ENDIF (ZMQ_LIBRARY)

# Report the results.
IF (NOT ZMQ_FOUND)
	SET(ZMQ_DIR_MESSAGE "ZeroMQ was not found. Make sure the ZMQ_LIBRARY environment variable is set.")
	IF (NOT ZMQ_FIND_QUIETLY)
		MESSAGE(STATUS "${ZMQ_DIR_MESSAGE}")
	ELSE (NOT ZMQ_FIND_QUIETLY)
		IF (ZMQ_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${ZMQ_DIR_MESSAGE}")
		ENDIF (ZMQ_FIND_REQUIRED)
	ENDIF (NOT ZMQ_FIND_QUIETLY)
ENDIF (NOT ZMQ_FOUND)


MARK_AS_ADVANCED(
    ZMQ_LIBRARIES
)
