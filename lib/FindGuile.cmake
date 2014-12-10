# Copyright (c) 2008, 2014 OpenCog.org (http://opencog.org)
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# - Try to find Guile; Once done this will define
#
# GUILE_FOUND - system has the GUILE library
# GUILE_INCLUDE_DIRS - the GUILE include directory
# GUILE_LIBRARIES - The libraries needed to use GUILE


# Look for the header file
# Look for guile-2.2 first, then 2.0, then 1.8
# Macports for OSX puts things in /opt/local
FIND_PATH(GUILE_INCLUDE_DIR libguile.h 
	/usr/include/guile/2.2
	/usr/local/include/guile/2.2
	/opt/local/include/guile/2.2

	/usr/include/guile/2.0
	/usr/local/include/guile/2.0
	/opt/local/include/guile/2.0

	/usr/include/libguile
	/usr/local/include/libguile
	/opt/local/include/guile

	/usr/include
	/usr/local/include
)

# Look for the library
FIND_LIBRARY(GUILE_LIBRARY NAMES guile-2.2 guile-2.0 guile PATHS 
	/usr/lib
	/usr/local/lib
	/opt/local/lib
)

# Copy the results to the output variables.
IF(GUILE_INCLUDE_DIR AND GUILE_LIBRARY)
	SET(GUILE_FOUND 1)
	SET(GUILE_LIBRARIES ${GUILE_LIBRARY})
	SET(GUILE_INCLUDE_DIRS ${GUILE_INCLUDE_DIR})
ELSE(GUILE_INCLUDE_DIR AND GUILE_LIBRARY)
	SET(GUILE_FOUND 0)
	SET(GUILE_LIBRARIES)
	SET(GUILE_INCLUDE_DIRS)
ENDIF(GUILE_INCLUDE_DIR AND GUILE_LIBRARY)


# check guile's version if we're using cmake >= 2.6
IF(GUILE_INCLUDE_DIR)
	SET(GUILE_VERSION_MAJOR 0)
	SET(GUILE_VERSION_MINOR 0)
	SET(GUILE_VERSION_PATCH 0)

	IF(NOT EXISTS "${GUILE_INCLUDE_DIR}/libguile/version.h")
		MESSAGE(FATAL_ERROR "Found ${GUILE_INCLUDE_DIR}/libguile.h but not version.h; check your guile installation!")
	ENDIF(NOT EXISTS "${GUILE_INCLUDE_DIR}/libguile/version.h")

	# Extract the libguile version from the 'version.h' file
	SET(GUILE_MAJOR_VERSION 0)
	FILE(READ "${GUILE_INCLUDE_DIR}/libguile/version.h" _GUILE_VERSION_H_CONTENTS)

	STRING(REGEX MATCH "#define SCM_MAJOR_VERSION[	 ]+([0-9])" _MATCH "${_GUILE_VERSION_H_CONTENTS}")
	SET(GUILE_VERSION_MAJOR ${CMAKE_MATCH_1})
	STRING(REGEX MATCH "#define SCM_MINOR_VERSION[	 ]+([0-9]+)" _MATCH "${_GUILE_VERSION_H_CONTENTS}")
	SET(GUILE_VERSION_MINOR ${CMAKE_MATCH_1})
	STRING(REGEX MATCH "#define SCM_MICRO_VERSION[	 ]+([0-9]+)" _MATCH "${_GUILE_VERSION_H_CONTENTS}")
	SET(GUILE_VERSION_PATCH ${CMAKE_MATCH_1})

	SET(GUILE_VERSION "${GUILE_VERSION_MAJOR}.${GUILE_VERSION_MINOR}.${GUILE_VERSION_PATCH}")

	# Check found version against required one
	IF (DEFINED Guile_FIND_VERSION AND ${GUILE_VERSION} VERSION_LESS Guile_FIND_VERSION)
		SET(GUILE_FOUND FALSE)
	ELSE ()
		SET(GUILE_FOUND TRUE)
	ENDIF ()

ENDIF(GUILE_INCLUDE_DIR)

IF(GUILE_FOUND AND GUILE_VERSION_MAJOR EQUAL 2)
	ADD_DEFINITIONS(-DHAVE_GUILE2)
ENDIF(GUILE_FOUND AND GUILE_VERSION_MAJOR EQUAL 2)

# Report the results.
IF (Guile_FIND_VERSION)
	SET(_GUILE_VERSION_MESSAGE_STRING "(${GUILE_VERSION} >= ${Guile_FIND_VERSION})")
ENDIF (Guile_FIND_VERSION)

IF(GUILE_FOUND)
	IF(NOT GUILE_FIND_QUIETLY)
		MESSAGE(STATUS "Guile ${_GUILE_VERSION_MESSAGE_STRING} was found.")
	ENDIF(NOT GUILE_FIND_QUIETLY)
ELSE(GUILE_FOUND)
	SET(GUILE_DIR_MESSAGE "Guile ${_GUILE_VERSION_MESSAGE_STRING} was not found. Make sure GUILE_LIBRARY and GUILE_INCLUDE_DIR are set.")
	IF(NOT GUILE_FIND_QUIETLY)
		MESSAGE(STATUS "${GUILE_DIR_MESSAGE}")
	ELSE(NOT GUILE_FIND_QUIETLY)
		IF(GUILE_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${GUILE_DIR_MESSAGE}")
		ENDIF(GUILE_FIND_REQUIRED)
	ENDIF(NOT GUILE_FIND_QUIETLY)
ENDIF(GUILE_FOUND)

MARK_AS_ADVANCED(GUILE_INCLUDE_DIR GUILE_LIBRARY)
