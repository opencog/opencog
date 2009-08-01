# Copyright (c) 2008, OpenCog.org (http://opencog.org)
# Copyright (c) 2009, Linas Vepstas
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
# - Try to find HyperTable; Once done this will define
#
# HYPERTABLE_FOUND - system has the HYPERTABLE library
# HYPERTABLE_INCLUDE_DIRS - the HYPERTABLE include directory
# HYPERTABLE_LIBRARIES - The libraries needed to use HYPERTABLE


# Look for the header file
FIND_PATH(HYPERTABLE_INCLUDE_DIR Hypertable/Lib/Client.h 
	/usr/include
	/usr/local/include 
	/usr/hypertable/0.9.2.4/include
	/usr/local/hypertable/0.9.2.4/include
	/opt/hypertable/0.9.2.4/include
)

# Look for the libraries
set(HYPER_LIB_PATHS 
		/usr/lib 
		/usr/local/lib
		/usr/hypertable/0.9.2.4/lib
		/usr/local/hypertable/0.9.2.4/lib
		/opt/hypertable/0.9.2.4/lib
)

FIND_LIBRARY(HYPERTABLE_LIBRARY NAMES HyperCommon PATHS ${HYPER_LIB_PATHS})
FIND_LIBRARY(HYPER_DFS_BROKER NAMES HyperDfsBroker PATHS ${HYPER_LIB_PATHS})

# XXX Unclear -- do we need to find *all* of these?
# libHyperCommon.so  libHyperDfsBroker.so  libHyperRanger.so  libHypertable.so
# libHyperComm.so    libHyperDfsCmds.so    libHyperspace.so   libHyperTools.so


# Copy the results to the output variables.
IF(HYPERTABLE_INCLUDE_DIR AND HYPERTABLE_LIBRARY)
	SET(HYPERTABLE_FOUND 1)
	SET(HYPERTABLE_LIBRARIES ${HYPERTABLE_LIBRARY} ${HYPER_DFS_BROKER})
	SET(HYPERTABLE_INCLUDE_DIRS ${HYPERTABLE_INCLUDE_DIR})

	MESSAGE(STATUS "Found these hypertable libs: ${HYPERTABLE_LIBRARIES}")

ELSE(HYPERTABLE_INCLUDE_DIR AND HYPERTABLE_LIBRARY)
	SET(HYPERTABLE_FOUND 0)
	SET(HYPERTABLE_LIBRARIES)
	SET(HYPERTABLE_INCLUDE_DIRS)
ENDIF(HYPERTABLE_INCLUDE_DIR AND HYPERTABLE_LIBRARY)

# Report the results.
IF(NOT HYPERTABLE_FOUND)
	SET(HYPERTABLE_DIR_MESSAGE "HyperTable was not found. Make sure HYPERTABLE_LIBRARY and HYPERTABLE_INCLUDE_DIR are set.")
	IF(NOT HYPERTABLE_FIND_QUIETLY)
		MESSAGE(STATUS "${HYPERTABLE_DIR_MESSAGE}")
	ELSE(NOT HYPERTABLE_FIND_QUIETLY)
		IF(HYPERTABLE_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${HYPERTABLE_DIR_MESSAGE}")
		ENDIF(HYPERTABLE_FIND_REQUIRED)
	ENDIF(NOT HYPERTABLE_FIND_QUIETLY)
ENDIF(NOT HYPERTABLE_FOUND)

MARK_AS_ADVANCED(
	HYPERTABLE_INCLUDE_DIRS
	HYPERTABLE_LIBRARIES
)
