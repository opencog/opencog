# - Try to find the XMLParse library; Once done this will define
#
#  CSOCKETS_FOUND - system has the OpenSSL library
#  CSOCKETS_INCLUDE_DIR - the OpenSSL include directory
#  CSOCKETS_LIBRARIES - The libraries needed to use OpenSSL

# Copyright (c) 2008, OpenCog.org (http://opencog.org)
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# Look for the header file
FIND_PATH(CSOCKETS_INCLUDE_DIR sockets-config.h /usr/include /usr/include/Sockets /usr/local/include /usr/local/include/Sockets)
#MARK_AS_ADVANCED(CSOCKETS_INCLUDE_DIR)

# Look for the library
FIND_LIBRARY(CSOCKETS_LIBRARY NAMES Sockets PATH /usr/lib /usr/lib/Sockets /usr/local/lib /usr/local/lib/Sockets)
#MARK_AS_ADVANCED(CSOCKETS_LIBRARY)

# Copy the results to the output variables.
IF(CSOCKETS_INCLUDE_DIR AND CSOCKETS_LIBRARY)
  SET(CSOCKETS_FOUND 1)
  SET(CSOCKETS_LIBRARIES ${CSOCKETS_LIBRARY})
  SET(CSOCKETS_INCLUDE_DIRS ${CSOCKETS_INCLUDE_DIR})
ELSE(CSOCKETS_INCLUDE_DIR AND CSOCKETS_LIBRARY)
  SET(CSOCKETS_FOUND 0)
  SET(CSOCKETS_LIBRARIES)
  SET(CSOCKETS_INCLUDE_DIRS)
ENDIF(CSOCKETS_INCLUDE_DIR AND CSOCKETS_LIBRARY)

# Report the results.
IF(NOT CSOCKETS_FOUND)
  SET(CSOCKETS_DIR_MESSAGE
    "Csockets was not found. Make sure CSOCKETS_LIBRARY and CSOCKETS_INCLUDE_DIR are set.")
  IF(NOT CSOCKETS_FIND_QUIETLY)
    MESSAGE(STATUS "${CSOCKETS_DIR_MESSAGE}")
  ELSE(NOT CSOCKETS_FIND_QUIETLY)
    IF(CSOCKETS_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "${CSOCKETS_DIR_MESSAGE}")
    ENDIF(CSOCKETS_FIND_REQUIRED)
  ENDIF(NOT CSOCKETS_FIND_QUIETLY)
ENDIF(NOT CSOCKETS_FOUND)
