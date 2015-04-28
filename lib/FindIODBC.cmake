# Copyright (c) 2008, OpenCog.org (http://opencog.org)
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# Try to find the IODBC library; Once done this will define
#
# IODBC_FOUND        - system has the IODBC library
# IODBC_INCLUDE_DIRS - the IODBC include directory
# IODBC_LIBRARIES    - The libraries needed to use IODBC



# Look for the header file
FIND_PATH(IODBC_INCLUDE_DIR iodbcunix.h /usr/include /usr/local/include /usr/include/iodbc /usr/local/include/iodbc /usr/include/libiodbc /usr/local/include/libiodbc)

# Look for the library
FIND_LIBRARY(IODBC_LIBRARY 
	NAMES 
		iodbc 
	PATHS
		/usr/lib
		/usr/local/lib
)

# Copy the results to the output variables.
IF (IODBC_INCLUDE_DIR AND IODBC_LIBRARY)
	SET(IODBC_FOUND 1)
	SET(IODBC_LIBRARIES ${IODBC_LIBRARY})
	SET(IODBC_INCLUDE_DIRS ${IODBC_INCLUDE_DIR})
ELSE (IODBC_INCLUDE_DIR AND IODBC_LIBRARY)
	SET(IODBC_FOUND 0)
	SET(IODBC_LIBRARIES)
	SET(IODBC_INCLUDE_DIRS)
ENDIF (IODBC_INCLUDE_DIR AND IODBC_LIBRARY)

# Report the results.
IF (NOT IODBC_FOUND)
	SET(IODBC_DIR_MESSAGE "IODBC was not found. Make sure IODBC_LIBRARY and IODBC_INCLUDE_DIR are set.")
	IF (NOT IODBC_FIND_QUIETLY)
		MESSAGE(STATUS "${IODBC_DIR_MESSAGE}")
	ELSE (NOT IODBC_FIND_QUIETLY)
		IF (IODBC_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${IODBC_DIR_MESSAGE}")
		ENDIF (IODBC_FIND_REQUIRED)
	ENDIF (NOT IODBC_FIND_QUIETLY)
ENDIF (NOT IODBC_FOUND)

MARK_AS_ADVANCED(IODBC_INCLUDE_DIRS)
MARK_AS_ADVANCED(IODBC_LIBRARIES)
