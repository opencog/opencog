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
	/usr/hypertable/current/include
	/usr/local/hypertable/current/include
	/opt/hypertable/current/include
)

# Look for the config file .. this is needed by the C++ code ...
FIND_FILE(HYPERTABLE_CONFIG_FILE conf/hypertable.cfg
	/usr/include
	/usr/local/include 
	/usr/hypertable/current
	/usr/local/hypertable/current
	/opt/hypertable/current
)

# Assume that the base path is the install path
FIND_PATH(HYPERTABLE_INSTALL_DIR conf/hypertable.cfg
	/usr/include
	/usr/local/include 
	/usr/hypertable/current
	/usr/local/hypertable/current
	/opt/hypertable/current
)


# Look for the libraries
set(HYPER_LIB_PATHS 
		/usr/lib 
		/usr/local/lib
		/usr/hypertable/current/lib
		/usr/local/hypertable/current/lib
		/opt/hypertable/current/lib
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
	MESSAGE(STATUS "Found hypertable install path: ${HYPERTABLE_INSTALL_DIR}")
	MESSAGE(STATUS "Found hypertable config file: ${HYPERTABLE_CONFIG_FILE}")

	ADD_DEFINITIONS(-DHYPERTABLE_INSTALL_DIR=\\"${HYPERTABLE_INSTALL_DIR}/\\")
	ADD_DEFINITIONS(-DHYPERTABLE_CONFIG_FILE=\\"${HYPERTABLE_CONFIG_FILE}\\")

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
