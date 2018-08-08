# - Find ZeroMQ library
#
# This module checks for the required version number and defines
#  ZMQ_LIBRARIES, the libraries to link against to use ZMQ.
#  ZMQ_LIB_DIR, the location of the libraries
#  ZMQ_FOUND, If false, do not try to use ZMQ
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_LIBRARY(ZMQ_LIBRARY NAMES zmq PATHS
    /usr/lib
    /usr/local/lib
    /usr/local/lib/zmq
)

# Copy the results to the output variables.
IF (ZMQ_LIBRARY)
	SET(ZMQ_FOUND 1)
	SET(ZMQ_LIBRARIES ${ZMQ_LIBRARY})
	MESSAGE(STATUS "Found ZeroMQ library: ${ZMQ_LIBRARIES}")
ELSE (ZMQ_LIBRARY)
	SET(ZMQ_FOUND 0)
	SET(ZMQ_LIBRARIES)
ENDIF (ZMQ_LIBRARY)

# Check found version against required one
IF (ZMQ_FOUND)
    FIND_FILE(ZMQ_CFG_FILE zmq.h
            /usr/include
            /usr/local/include
    )
    IF (DEFINED ZMQ_CFG_FILE)
        FILE(READ "${ZMQ_CFG_FILE}" _ZMQ_VERSION_H_CONTENTS)
        STRING(REGEX MATCH "#define ZMQ_VERSION_MAJOR ([0-9]+)" _MATCH "${_ZMQ_VERSION_H_CONTENTS}")
        SET(ZMQ_VERSION_MAJOR ${CMAKE_MATCH_1})
        STRING(REGEX MATCH "#define ZMQ_VERSION_MINOR ([0-9]+)" _MATCH "${_ZMQ_VERSION_H_CONTENTS}")
        SET(ZMQ_VERSION_MINOR ${CMAKE_MATCH_1})
        STRING(REGEX MATCH "#define ZMQ_VERSION_PATCH ([0-9]+)" _MATCH "${_ZMQ_VERSION_H_CONTENTS}")
        SET(ZMQ_VERSION_PATCH ${CMAKE_MATCH_1})
        set (ZMQ_VERSION "${ZMQ_VERSION_MAJOR}.${ZMQ_VERSION_MINOR}.${ZMQ_VERSION_PATCH}")
        MESSAGE(STATUS "Detected ZeroMQ version number: ${ZMQ_VERSION}")
        IF (DEFINED ZMQ_VERSION AND ZMQ_VERSION VERSION_LESS ZMQ_FIND_VERSION)
            SET(ZMQ_FOUND FALSE)
            MESSAGE(STATUS "Installed version ${ZMQ_VERSION} of ZeroMQ does not meet the minimum required version of ${ZMQ_FIND_VERSION}")
        ENDIF ()
    ELSE ()
        MESSAGE(STATUS "Unable to find zmq.h header file")
        SET(ZMQ_FOUND 0)
    ENDIF ()
ENDIF ()

# Report the results.
IF (NOT ZMQ_FOUND)
	SET(ZMQ_DIR_MESSAGE "Required version of ZeroMQ was not found. Make sure the ZMQ_LIBRARY environment variable is set.")
	MESSAGE(STATUS "${ZMQ_DIR_MESSAGE}")	
        IF (ZMQ_FIND_REQUIRED)
                MESSAGE(FATAL_ERROR "${ZMQ_DIR_MESSAGE}")
        ENDIF (ZMQ_FIND_REQUIRED)	
ENDIF (NOT ZMQ_FOUND)

MARK_AS_ADVANCED(
    ZMQ_LIBRARIES
)
