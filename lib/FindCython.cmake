# Find the Cython compiler.
#
# This code sets the following variables:
#
#  CYTHON_EXECUTABLE
#
# See also UseCython.cmake

#=============================================================================
# Copyright 2011 Kitware, Inc.
# Copyright (c) 2008, 2014 OpenCog.org (http://opencog.org)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#=============================================================================

# Use the Cython executable that lives next to the Python executable
# if it is a local installation.
find_package(PythonInterp)
if(PYTHONINTERP_FOUND)
	get_filename_component( _python_path ${PYTHON_EXECUTABLE} PATH)
	find_program( CYTHON_EXECUTABLE NAMES cython cython.bat
		HINTS ENV PATH ${_python_path})
else()
	find_program( CYTHON_EXECUTABLE NAMES cython cython.bat)
endif()


include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Cython REQUIRED_VARS CYTHON_EXECUTABLE)


# Check cython's version if we're using cmake >= 2.6
IF(CYTHON_EXECUTABLE AND NOT CMAKE_MAJOR_VERSION LESS 2 AND NOT CMAKE_MINOR_VERSION LESS 6)
	SET(CYTH_VERSION_MAJOR 0)
	SET(CYTH_VERSION_MINOR 0)
	SET(CYTH_VERSION_PATCH 0)

	# Extract the cython version from the --version flag
	execute_process(COMMAND ${CYTHON_EXECUTABLE} --version
 	                ERROR_VARIABLE _CVERNO)

	STRING(REGEX MATCH ".+([0-9]+)\\.([0-9]+)\\.([0-9]+)" _MATCH "${_CVERNO}")
	SET(CYTH_VERSION_MAJOR ${CMAKE_MATCH_1})
	SET(CYTH_VERSION_MINOR ${CMAKE_MATCH_2})
	SET(CYTH_VERSION_PATCH ${CMAKE_MATCH_3})

	# Check found version against required one
	# Use nested less...equal so that version 2.0.0 is newer than version 1.9.1 is newer than 1.8.6
	IF (Cython_FIND_VERSION_MAJOR AND ${CYTH_VERSION_MAJOR} LESS Cython_FIND_VERSION_MAJOR)
		SET(CYTHON_FOUND FALSE)
	ELSE (Cython_FIND_VERSION_MAJOR AND ${CYTH_VERSION_MAJOR} LESS Cython_FIND_VERSION_MAJOR)
		IF (Cython_FIND_VERSION_MAJOR AND ${CYTH_VERSION_MAJOR} EQUAL Cython_FIND_VERSION_MAJOR)
			IF (Cython_FIND_VERSION_MINOR AND ${CYTH_VERSION_MINOR} LESS Cython_FIND_VERSION_MINOR)
				SET(CYTHON_FOUND FALSE )
			ELSE (Cython_FIND_VERSION_MINOR AND ${CYTH_VERSION_MINOR} LESS Cython_FIND_VERSION_MINOR)
				IF (Cython_FIND_VERSION_MINOR AND ${CYTH_VERSION_MINOR} EQUAL Cython_FIND_VERSION_MINOR)
					IF (Cython_FIND_VERSION_PATCH AND ${CYTH_VERSION_PATCH} LESS Cython_FIND_VERSION_PATCH)
						SET(CYTHON_FOUND FALSE )
					ENDIF (Cython_FIND_VERSION_PATCH AND ${CYTH_VERSION_PATCH} LESS Cython_FIND_VERSION_PATCH)
				ENDIF (Cython_FIND_VERSION_MINOR AND ${CYTH_VERSION_MINOR} EQUAL Cython_FIND_VERSION_MINOR)
			ENDIF (Cython_FIND_VERSION_MINOR AND ${CYTH_VERSION_MINOR} LESS Cython_FIND_VERSION_MINOR)
		ENDIF (Cython_FIND_VERSION_MAJOR AND ${CYTH_VERSION_MAJOR} EQUAL Cython_FIND_VERSION_MAJOR)
	ENDIF (Cython_FIND_VERSION_MAJOR AND ${CYTH_VERSION_MAJOR} LESS Cython_FIND_VERSION_MAJOR)
ENDIF(CYTHON_EXECUTABLE AND NOT CMAKE_MAJOR_VERSION LESS 2 AND NOT CMAKE_MINOR_VERSION LESS 6)

IF (DEFINED Cython_FIND_VERSION_MAJOR)
	SET(_CYTH_VERSION_MESSAGE_STRING " (>=${Cython_FIND_VERSION_MAJOR}.${Cython_FIND_VERSION_MINOR}.${Cython_FIND_VERSION_PATCH})")
ENDIF (DEFINED Cython_FIND_VERSION_MAJOR)


# Report the results.
IF (CYTHON_FOUND)
	IF (NOT CYTHON_FIND_QUIETLY)
		MESSAGE(STATUS "Cython ${_CYTH_VERSION_MESSAGE_STRING} found.")
	ENDIF (NOT CYTHON_FIND_QUIETLY)
ELSE (CYTHON_FOUND)
	SET(CYTHON_DIR_MESSAGE
		"Cython ${_CYTH_VERSION_MESSAGE_STRING} was not found. Make sure CYTHON_EXECUTABLE is set.")
	IF (NOT CYTHON_FIND_QUIETLY)
		MESSAGE(STATUS "${CYTHON_DIR_MESSAGE}")
	ELSE (NOT CYTHON_FIND_QUIETLY)
		IF (CYTHON_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${CYTHON_DIR_MESSAGE}")
		ENDIF (CYTHON_FIND_REQUIRED)
	ENDIF (NOT CYTHON_FIND_QUIETLY)
ENDIF (CYTHON_FOUND)

MARK_AS_ADVANCED(
	CYTHON_EXECUTABLE
)
