# - Find cpprest library
#
# This module checks for the required version number and defines
#  cpprest_LIBRARIES, the libraries to link against to use cpprest.
#  cpprest_LIB_DIR, the location of the libraries
#  cpprest_FOUND, If false, do not try to build PatternMiner
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

FIND_LIBRARY(cpprest_LIBRARY NAMES cpprest PATHS
    /usr/lib
    /usr/local/lib
)

# Check cpprest's version 
IF (cpprest_LIBRARY)
       FIND_FILE(cpprest_version_FILE version.h
                 /usr/include/cpprest/
                 /usr/local/include/cpprest/
                )
        IF (DEFINED cpprest_version_FILE)
            FILE(READ "${cpprest_version_FILE}" _CPPREST_VERSION_H_CONTENTS)
            STRING(REGEX MATCH "#define CPPREST_VERSION_MAJOR ([0-9]+)" _MATCH "${_CPPREST_VERSION_H_CONTENTS}")
            SET(cpprest_VERSION_MAJOR ${CMAKE_MATCH_1})
            STRING(REGEX MATCH "#define CPPREST_VERSION_MINOR ([0-9]+)" _MATCH "${_CPPREST_VERSION_H_CONTENTS}")
            SET(cpprest_VERSION_MINOR ${CMAKE_MATCH_1})
            set (cpprest_VERSION "${cpprest_VERSION_MAJOR}.${cpprest_VERSION_MINOR}")
            MESSAGE(STATUS "Detected cpprest version number: ${cpprest_VERSION}")
            IF (DEFINED cpprest_VERSION AND cpprest_VERSION VERSION_LESS cpprest_FIND_VERSION)
                SET(cpprest_FOUND 0)
	        SET(cpprest_LIBRARIES)
                MESSAGE(STATUS "Installed version ${cpprest_VERSION} of cpprest does not meet the minimum required version of ${cpprest_FIND_VERSION}")
            ELSE ()
                SET(cpprest_FOUND 1)
	        SET(cpprest_LIBRARIES ${cpprest_LIBRARY})
	        MESSAGE(STATUS "Found libcpprest library: ${cpprest_LIBRARIES}")
            ENDIF ()
        ELSE ()
            SET(cpprest_FOUND 0)
	    SET(cpprest_LIBRARIES )
	    MESSAGE(STATUS "Unkown cpprest version: unable to find version.h in cpprest/include/")
        ENDIF ()
ELSE (cpprest_LIBRARY)
	SET(cpprest_FOUND 0)
	SET(cpprest_LIBRARIES)
ENDIF (cpprest_LIBRARY)

MARK_AS_ADVANCED(
    cpprest_LIBRARIES
)
