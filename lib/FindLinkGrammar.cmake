# - Try to find the link-grammar library; Once done this will define
#
#  LINK_GRAMMAR_FOUND - system has the link-grammar library
#  LINK_GRAMMAR_INCLUDE_DIRS - the link-grammar include directory
#  LINK_GRAMMAR_LIBRARIES - The libraries needed to use link-grammar
#  LINK_GRAMMAR_DATA_DIR - the dir where you will find the dictionaries

# Copyright (c) 2008 OpenCog.org (http://opencog.org)
# Copyright (c) 2014 Linas Vepstas
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
# 1. Redistributions of source code must retain the copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote products
#    derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Look for the header file
FIND_PATH(LINK_GRAMMAR_INCLUDE_DIR link-grammar/link-includes.h)
FIND_PATH(LINK_GRAMMAR_DATA_DIR 4.0.dict
	PATHS 
		/usr/share/link-grammar/en/
		/usr/local/share/link-grammar/en/
		/opt/local/share/link-grammar/en/
	PATH_SUFFIXES share/link-grammar/en/ )

# Look for the library
FIND_LIBRARY(LINK_GRAMMAR_LIBRARY
	NAMES
		link-grammar
	PATHS
		/usr/lib 
		/usr/local/lib
		/opt/link-grammar/lib
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

# check link-grammar's version if we're using cmake >= 2.6
IF(LINK_GRAMMAR_INCLUDE_DIR AND NOT CMAKE_MAJOR_VERSION LESS 2 AND NOT CMAKE_MINOR_VERSION LESS 6)
	SET(LG_VERSION_MAJOR 0)
	SET(LG_VERSION_MINOR 0)
	SET(LG_VERSION_PATCH 0)

	# Extract the version from the 'version.h' file
	FILE(READ "${LINK_GRAMMAR_INCLUDE_DIR}/link-grammar/link-features.h" _LG_FEAT_H_CONTENTS)

	STRING(REGEX MATCH "#define LINK_MAJOR_VERSION[	 ]+([0-9]+)" _MATCH "${_LG_FEAT_H_CONTENTS}")
	SET(LG_VERSION_MAJOR ${CMAKE_MATCH_1})
	STRING(REGEX MATCH "#define LINK_MINOR_VERSION[	 ]+([0-9]+)" _MATCH "${_LG_FEAT_H_CONTENTS}")
	SET(LG_VERSION_MINOR ${CMAKE_MATCH_1})
	STRING(REGEX MATCH "#define LINK_MICRO_VERSION[	 ]+([0-9]+)" _MATCH "${_LG_FEAT_H_CONTENTS}")
	SET(LG_VERSION_PATCH ${CMAKE_MATCH_1})

	SET(LG_VERSION "${LG_VERSION_MAJOR}.${LG_VERSION_MINOR}.${LG_VERSION_PATCH}")

	# Check found version against required one
	IF (DEFINED LinkGrammar_FIND_VERSION AND ${LG_VERSION} VERSION_LESS LinkGrammar_FIND_VERSION)
		SET(LINK_GRAMMAR_FOUND FALSE)
	ELSE ()
		SET(LINK_GRAMMAR_FOUND TRUE)
	ENDIF ()
ENDIF(LINK_GRAMMAR_INCLUDE_DIR AND NOT CMAKE_MAJOR_VERSION LESS 2 AND NOT CMAKE_MINOR_VERSION LESS 6)

IF (LinkGrammar_FIND_VERSION)
	SET(_LG_VERSION_MESSAGE_STRING "(${LG_VERSION} >= ${LinkGrammar_FIND_VERSION})")
ENDIF (LinkGrammar_FIND_VERSION)

# Report the results.
IF (LINK_GRAMMAR_FOUND)
	IF (NOT LINK_GRAMMAR_FIND_QUIETLY)
		MESSAGE(STATUS "Link Grammar ${_LG_VERSION_MESSAGE_STRING} found.")
	ENDIF (NOT LINK_GRAMMAR_FIND_QUIETLY)
ELSE (LINK_GRAMMAR_FOUND)
	SET(LINK_GRAMMAR_DIR_MESSAGE
		"link-grammar${_LG_VERSION_MESSAGE_STRING} was not found. Make sure LINK_GRAMMAR_LIBRARY, LINK_GRAMMAR_INCLUDE_DIR and LINK_GRAMMAR_DATA_DIR are set.")
	IF (NOT LINK_GRAMMAR_FIND_QUIETLY)
		MESSAGE(STATUS "${LINK_GRAMMAR_DIR_MESSAGE}")
	ELSE (NOT LINK_GRAMMAR_FIND_QUIETLY)
		IF (LINK_GRAMMAR_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${LINK_GRAMMAR_DIR_MESSAGE}")
		ENDIF (LINK_GRAMMAR_FIND_REQUIRED)
	ENDIF (NOT LINK_GRAMMAR_FIND_QUIETLY)
ENDIF (LINK_GRAMMAR_FOUND)

MARK_AS_ADVANCED(
	LINK_GRAMMAR_INCLUDE_DIR
	LINK_GRAMMAR_LIBRARY
)
