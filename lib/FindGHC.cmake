# Copyright (c) 2015 OpenCog.org (http://opencog.org)
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# - Try to find GHC; Once done this will define
#
# GHC_FOUND - system has the GHC Haskell compiler


FIND_PROGRAM(GHC_EXECUTABLE ghc)

IF (DEFINED GHC_EXECUTABLE)
	SET(GHC_FOUND TRUE)

	# Non-working attempt to tell CMake what the .hs extension means.
	# Again, this doesn't work ....
	ADD_CUSTOM_COMMAND(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.o
		COMMAND "${GHC_EXECUTABLE}"
#				-o ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.o
		${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.hs
		DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.hs
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	)

ELSE ()
	SET(GHC_FOUND FALSE)
ENDIF ()


IF(GHC_FOUND)
	IF(NOT GHC_FIND_QUIETLY)
		MESSAGE(STATUS "GHC was found.")
	ENDIF(NOT GHC_FIND_QUIETLY)
ELSE(GHC_FOUND)
	SET(GHC_DIR_MESSAGE "GHC was not found.")
	IF(NOT GHC_FIND_QUIETLY)
		MESSAGE(STATUS "${GHC_DIR_MESSAGE}")
	ELSE(NOT GHC_FIND_QUIETLY)
		IF(GHC_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${GHC_DIR_MESSAGE}")
		ENDIF(GHC_FIND_REQUIRED)
	ENDIF(NOT GHC_FIND_QUIETLY)
ENDIF(GHC_FOUND)
