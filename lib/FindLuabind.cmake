# - Find LUABIND includes and library
#
# This module defines
#  LUABIND_INCLUDE_DIR
#  LUABIND_LIBRARIES, the libraries to link against to use LUABIND.
#  LUABIND_LIB_DIR, the location of the libraries
#  LUABIND_FOUND, If false, do not try to use LUABIND
#
# Copyright © 2007, Matt Williams
# Changes for LUABIND detection by Garvek, 2008
# Changes for version detection by Welter, 2009
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

IF (LUABIND_LIBRARIES AND LUABIND_INCLUDE_DIR)
   SET(LUABIND_FIND_QUIETLY TRUE) # Already in cache, be silent
ENDIF (LUABIND_LIBRARIES AND LUABIND_INCLUDE_DIR)


FIND_PATH(LUABIND_INCLUDE_DIR luabind/luabind.hpp)

FIND_LIBRARY(LUABIND_LIBRARIES NAMES luabind)

IF(NOT LUABIND_LIBRARIES)
   FIND_LIBRARY(LUABIND_LIBRARIES NAMES luabindd)
ENDIF(NOT LUABIND_LIBRARIES)

IF( LUABIND_INCLUDE_DIR AND LUABIND_LIBRARIES)
   SET(LUABIND_FOUND TRUE)
   INCLUDE(CheckLibraryExists)
   CHECK_LIBRARY_EXISTS(${LUABIND_LIBRARIES} open "" LUABIND_NEED_PREFIX)
ELSE(LUABIND_INCLUDE_DIR AND LUABIND_LIBRARIES)
   SET(LUABIND_FOUND FALSE)
ENDIF( LUABIND_INCLUDE_DIR AND LUABIND_LIBRARIES)

# check luabind's version if we're using cmake >= 2.6
IF(LUABIND_FOUND AND NOT CMAKE_MAJOR_VERSION LESS 2 AND NOT CMAKE_MINOR_VERSION LESS 6)
    # check version, if required
    #MESSAGE(STATUS "Checking luabind version, if required") 
    IF (Luabind_FIND_VERSION_MAJOR OR Luabind_FIND_VERSION_MINOR OR Luabind_FIND_VERSION_PATCH) 
        # Extract the luabind version from the 'version.hpp' file
        IF (NOT EXISTS "${LUABIND_INCLUDE_DIR}/luabind/version.hpp")
            SET(LUABIND_FOUND FALSE)
        ELSE (NOT EXISTS "${LUABIND_INCLUDE_DIR}/luabind/version.hpp")
            FILE(READ "${LUABIND_INCLUDE_DIR}/luabind/version.hpp" _LUABIND_VERSION_H_CONTENTS)
            STRING(REGEX MATCH "# *define LUABIND_VERSION[   ]+([0-9]+)" _MATCH "${_LUABIND_VERSION_H_CONTENTS}")
            SET(LUABIND_VERSION ${CMAKE_MATCH_1})
            MESSAGE(STATUS "LUABIND_VERSION(from version.hpp file)=${LUABIND_VERSION}") 

            #Sets the required version
            SET(LUABIND_FIND_VERSION "")
            IF (Luabind_FIND_VERSION_MAJOR AND Luabind_FIND_VERSION_MAJOR GREATER 0) 
                SET(LUABIND_FIND_VERSION ${Luabind_FIND_VERSION_MAJOR})
            ENDIF (Luabind_FIND_VERSION_MAJOR AND Luabind_FIND_VERSION_MAJOR GREATER 0) 
            IF (Luabind_FIND_VERSION_MINOR) 
                SET(LUABIND_FIND_VERSION "${LUABIND_FIND_VERSION}${Luabind_FIND_VERSION_MINOR}")
                IF (Luabind_FIND_VERSION_PATCH) 
# TODO: Check how many digits VERSION_PATCH has. For now, assuming it has only 1 digit (so, a zero is added bofore it)
                    SET(LUABIND_FIND_VERSION "${LUABIND_FIND_VERSION}0${Luabind_FIND_VERSION_PATCH}")
                ELSE (Luabind_FIND_VERSION_PATCH) 
                    SET(LUABIND_FIND_VERSION "${LUABIND_FIND_VERSION}00")
                ENDIF (Luabind_FIND_VERSION_PATCH) 
            ELSE (Luabind_FIND_VERSION_MINOR) 
                SET(LUABIND_FIND_VERSION "${LUABIND_FIND_VERSION}000")
            ENDIF (Luabind_FIND_VERSION_MINOR) 

           #Finally, check version agains required one
           IF (${LUABIND_VERSION} LESS ${LUABIND_FIND_VERSION})
               SET(LUABIND_FOUND FALSE)
           ENDIF (${LUABIND_VERSION} LESS ${LUABIND_FIND_VERSION})
        ENDIF (NOT EXISTS "${LUABIND_INCLUDE_DIR}/luabind/version.hpp")
    ENDIF (Luabind_FIND_VERSION_MAJOR OR Luabind_FIND_VERSION_MINOR OR Luabind_FIND_VERSION_PATCH) 
ENDIF(LUABIND_FOUND AND NOT CMAKE_MAJOR_VERSION LESS 2 AND NOT CMAKE_MINOR_VERSION LESS 6)

# Report the results.
IF(NOT LUABIND_FOUND)
        IF (Luabind_FIND_VERSION_MAJOR)
                SET(_LUABIND_VERSION_MESSAGE_STRING " (>=${Luabind_FIND_VERSION_MAJOR}.${Luabind_FIND_VERSION_MINOR}.${Luabind_FIND_VERSION_PATCH})")
        ENDIF (Luabind_FIND_VERSION_MAJOR)
        SET(LUABIND_DIR_MESSAGE "Luabind${_LUABIND_VERSION_MESSAGE_STRING} was not found. Make sure LUABIND_LIBRARY and LUABIND_INCLUDE_DIR are set.")
        IF(NOT LUABIND_FIND_QUIETLY)
                MESSAGE(STATUS "${LUABIND_DIR_MESSAGE}")
        ELSE(NOT LUABIND_FIND_QUIETLY)
                IF(LUABIND_FIND_REQUIRED)
                        MESSAGE(FATAL_ERROR "${LUABIND_DIR_MESSAGE}")
                ENDIF(LUABIND_FIND_REQUIRED)
        ENDIF(NOT LUABIND_FIND_QUIETLY)
ENDIF(NOT LUABIND_FOUND)


IF(LUABIND_FOUND)
  IF (NOT LUABIND_FIND_QUIETLY)
    MESSAGE(STATUS "Found Luabind library: ${LUABIND_LIBRARIES}")
    MESSAGE(STATUS "Found Luabind headers: ${LUABIND_INCLUDE_DIR}")
  ENDIF (NOT LUABIND_FIND_QUIETLY)
ELSE(LUABIND_FOUND)
  IF(LUABIND_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could NOT find LUABIND")
  ENDIF(LUABIND_FIND_REQUIRED)
ENDIF(LUABIND_FOUND)

MARK_AS_ADVANCED(LUABIND_INCLUDE_DIR LUABIND_LIBRARIES)
