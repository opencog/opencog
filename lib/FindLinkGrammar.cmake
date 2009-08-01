# - Try to find the link-grammar library; Once done this will define
#
#  LINK_GRAMMAR_FOUND - system has the link-grammar library
#  LINK_GRAMMAR_INCLUDE_DIRS - the link-grammar include directory
#  LINK_GRAMMAR_LIBRARIES - The libraries needed to use link-grammar
#  LINK_GRAMMAR_DATA_DIR - the dir where you will find the dictionaries

# Copyright (c) 2008, OpenCog.org (http://opencog.org)
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# Look for the header file
FIND_PATH(LINK_GRAMMAR_INCLUDE_DIR link-grammar/link-includes.h)
FIND_PATH(LINK_GRAMMAR_DATA_DIR 4.0.dict
	PATHS 
		/usr/share/link-grammar/en/
		/usr/local/share/link-grammar/en/)

# Look for the library
FIND_LIBRARY(LINK_GRAMMAR_LIBRARY
	NAMES
		link-grammar
	PATHS
		/usr/lib 
		/usr/local/lib
		/opt/local/lib)

# Copy the results to the output variables.
IF (LINK_GRAMMAR_INCLUDE_DIR AND LINK_GRAMMAR_LIBRARY AND LINK_GRAMMAR_DATA_DIR)
	SET(LINK_GRAMMAR_FOUND 1)
	SET(LINK_GRAMMAR_LIBRARIES ${LINK_GRAMMAR_LIBRARY})
	SET(LINK_GRAMMAR_INCLUDE_DIRS ${LINK_GRAMMAR_INCLUDE_DIR})
ELSE (LINK_GRAMMAR_INCLUDE_DIR AND LINK_GRAMMAR_LIBRARY AND LINK_GRAMMAR_DATA_DIR)
	SET(LINK_GRAMMAR_FOUND 0)
	SET(LINK_GRAMMAR_LIBRARIES)
	SET(LINK_GRAMMAR_INCLUDE_DIRS)
ENDIF (LINK_GRAMMAR_INCLUDE_DIR AND LINK_GRAMMAR_LIBRARY AND LINK_GRAMMAR_DATA_DIR)

# Report the results.
IF (NOT LINK_GRAMMAR_FOUND)
	SET(LINK_GRAMMAR_DIR_MESSAGE
		"link-grammar was not found. Make sure LINK_GRAMMAR_LIBRARY, LINK_GRAMMAR_INCLUDE_DIR and LINK_GRAMMAR_DATA_DIR are set.")
	IF (NOT LINK_GRAMMAR_FIND_QUIETLY)
		MESSAGE(STATUS "${LINK_GRAMMAR_DIR_MESSAGE}")
	ELSE (NOT LINK_GRAMMAR_FIND_QUIETLY)
		IF (LINK_GRAMMAR_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${LINK_GRAMMAR_DIR_MESSAGE}")
		ENDIF (LINK_GRAMMAR_FIND_REQUIRED)
	ENDIF (NOT LINK_GRAMMAR_FIND_QUIETLY)
ENDIF (NOT LINK_GRAMMAR_FOUND)

MARK_AS_ADVANCED(
	LINK_GRAMMAR_INCLUDE_DIR
	LINK_GRAMMAR_LIBRARY
)
