# - Find SIGAR
# Find the native SIGAR includes and library
#
#  SIGAR_INCLUDE_DIR - where to find SIGAR.h, etc.
#  SIGAR_LIBRARIES   - List of libraries when using SIGAR.
#  SIGAR_FOUND       - True if SIGAR found.
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.


if (SIGAR_INCLUDE_DIR)
  # Already in cache, be silent
  set(SIGAR_FIND_QUIETLY TRUE)
endif ()

find_path(SIGAR_INCLUDE_DIR sigar.h
  /opt/local/include
  /usr/local/include
  /usr/include
)

# SIGAR support a lot more platforms than listed here.
# cf. sigar.hyperic.com
set(SIGAR_NAMES sigar-x86-linux sigar-x86_64-linux sigar-amd64-linux sigar-universal-macosx sigar)
find_library(SIGAR_LIBRARY
  NAMES ${SIGAR_NAMES}
  PATHS /usr/lib /usr/local/lib /opt/local/lib
)

if (SIGAR_INCLUDE_DIR AND SIGAR_LIBRARY)
  set(SIGAR_FOUND TRUE)
  set(SIGAR_LIBRARIES ${SIGAR_LIBRARY} ${CMAKE_DL_LIBS})
else ()
  set(SIGAR_FOUND FALSE)
  set(SIGAR_LIBRARIES)
endif ()

if (SIGAR_FOUND)
  message(STATUS "Found SIGAR: ${SIGAR_LIBRARIES}")
else ()
  message(STATUS "Not Found SIGAR: ${SIGAR_LIBRARY}")
  if (SIGAR_FIND_REQUIRED)
    message(STATUS "Looked for SIGAR libraries named ${SIGAR_NAMES}.")
    message(FATAL_ERROR "Could NOT find SIGAR library")
  endif ()
endif ()

mark_as_advanced(
  SIGAR_LIBRARY
  SIGAR_INCLUDE_DIR
)
