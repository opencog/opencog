# - Find MOSES library
#
# This module checks for the required version number and defines
#  MOSES_LIBRARIES, the libraries to link against to use MOSES.
#  MOSES_LIB_DIR, the location of the libraries
#  MOSES_FOUND, If false, do not try to use MOSES
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_LIBRARY(MOSES_LIBRARY NAMES moses PATHS
    /usr/lib
    /usr/local/lib
    PATH_SUFFIXES moses
)

FIND_LIBRARY(MOSES_EXEC_LIBRARY NAMES moses_exec PATHS
    PATH_SUFFIXES moses
)

FIND_LIBRARY(COMBOANT_LIBRARY NAMES comboant PATHS
    PATH_SUFFIXES moses
)

FIND_LIBRARY(COMBOREDUCT_LIBRARY NAMES comboreduct PATHS
    PATH_SUFFIXES moses
)

FIND_LIBRARY(FEATURE_SELECTION_LIBRARY NAMES feature_selection PATHS
    PATH_SUFFIXES moses
)

# Copy the results to the output variables.
IF (MOSES_LIBRARY)
	SET(MOSES_FOUND 1)
	SET(MOSES_LIBRARIES ${MOSES_LIBRARY} ${MOSES_EXEC_LIBRARY}
		${COMBOANT_LIBRARY} ${COMBOREDUCT_LIBRARY}
		${FEATURE_SELECTION_LIBRARY})
	MESSAGE(STATUS "Found MOSES library: ${MOSES_LIBRARIES}")
ELSE (MOSES_LIBRARY)
	SET(MOSES_FOUND 0)
	SET(MOSES_LIBRARIES)
ENDIF (MOSES_LIBRARY)

# Check found version against required one
IF (MOSES_FOUND)
    FIND_FILE(MOSES_CFG_FILE moses.h
            /usr/include
            /usr/local/include
    )
#    IF (DEFINED MOSES_CFG_FILE)
#        FILE(READ "${MOSES_CFG_FILE}" _MOSES_VERSION_H_CONTENTS)
#        STRING(REGEX MATCH "#define MOSES_VERSION_MAJOR ([0-9])" _MATCH "${_MOSES_VERSION_H_CONTENTS}")
#        SET(MOSES_VERSION_MAJOR ${CMAKE_MATCH_1})
#        STRING(REGEX MATCH "#define MOSES_VERSION_MINOR ([0-9])" _MATCH "${_MOSES_VERSION_H_CONTENTS}")
#        SET(MOSES_VERSION_MINOR ${CMAKE_MATCH_1})
#        STRING(REGEX MATCH "#define MOSES_VERSION_PATCH ([0-9])" _MATCH "${_MOSES_VERSION_H_CONTENTS}")
#        SET(MOSES_VERSION_PATCH ${CMAKE_MATCH_1})
#        set (MOSES_VERSION "${MOSES_VERSION_MAJOR}.${MOSES_VERSION_MINOR}.${MOSES_VERSION_PATCH}")
#        MESSAGE(STATUS "Detected MOSES version number: ${MOSES_VERSION}")
#        IF (DEFINED MOSES_VERSION AND MOSES_VERSION VERSION_LESS MOSES_FIND_VERSION)
#            SET(MOSES_FOUND FALSE)
#            MESSAGE(STATUS "Installed version ${MOSES_VERSION} of MOSES does not meet the minimum required version of ${MOSES_FIND_VERSION}")
#        ENDIF ()
#    ELSE ()
#        MESSAGE(STATUS "Unable to find moses.h header file")
#        SET(MOSES_FOUND 0)
#    ENDIF ()
ENDIF ()

# Report the results.
IF (NOT MOSES_FOUND)
	SET(MOSES_DIR_MESSAGE "Required version of MOSES was not found. Make sure the MOSES_LIBRARY environment variable is set.")
	MESSAGE(STATUS "${MOSES_DIR_MESSAGE}")	
        IF (MOSES_FIND_REQUIRED)
                MESSAGE(FATAL_ERROR "${MOSES_DIR_MESSAGE}")
        ENDIF (MOSES_FIND_REQUIRED)	
ENDIF (NOT MOSES_FOUND)

MARK_AS_ADVANCED(
    MOSES_LIBRARIES
)
