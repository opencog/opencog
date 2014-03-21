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

	STRING(REGEX MATCH "[^0-9]?[0-9]+\\.[0-9]+\\.?[0-9]?" CYTH_VERSION "${_CVERNO}")

	# Check found version against required one
	IF (DEFINED Cython_FIND_VERSION AND ${CYTH_VERSION} VERSION_LESS Cython_FIND_VERSION)
		SET(CYTHON_FOUND FALSE)
	ELSE ()
		SET(CYTHON_FOUND TRUE)
	ENDIF ()
ENDIF(CYTHON_EXECUTABLE AND NOT CMAKE_MAJOR_VERSION LESS 2 AND NOT CMAKE_MINOR_VERSION LESS 6)

IF (DEFINED Cython_FIND_VERSION)
	SET(_CYTH_VERSION_MESSAGE_STRING "(${CYTH_VERSION} >= ${Cython_FIND_VERSION})")
ENDIF (DEFINED Cython_FIND_VERSION)

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
