# - Try to find Guile; Once done this will define
#
# GUILE_FOUND - system has the GUILE library
# GUILE_INCLUDE_DIRS - the GUILE include directory
# GUILE_LIBRARIES - The libraries needed to use GUILE
#
# XXX --- need to have guile-1.8, since the code uses some API
# functions that are not in guile-1.6 However, its not clear
# how to use CMake to skip searching for guile-1.6


# Copyright (c) 2008, OpenCog.org (http://opencog.org)
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# Look for the header file
FIND_PATH(GUILE_INCLUDE_DIR libguile.h /usr/include /usr/local/include /usr/include/libguile /usr/local/include/libguile)

# Look for the library
FIND_LIBRARY(GUILE_LIBRARY NAMES guile PATH /usr/lib /usr/local/lib)

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

# Report the results.
IF(NOT GUILE_FOUND)
	SET(GUILE_DIR_MESSAGE "Guile was not found. Make sure GUILE_LIBRARIES and GUILE_INCLUDE_DIRS are set.")
	IF(NOT GUILE_FIND_QUIETLY)
		MESSAGE(STATUS "${GUILE_DIR_MESSAGE}")
	ELSE(NOT GUILE_FIND_QUIETLY)
		IF(GUILE_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${GUILE_DIR_MESSAGE}")
		ENDIF(GUILE_FIND_REQUIRED)
	ENDIF(NOT GUILE_FIND_QUIETLY)
ENDIF(NOT GUILE_FOUND)

MARK_AS_ADVANCED(GUILE_INCLUDE_DIR GUILE_LIBRARY)
