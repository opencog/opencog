# Try to find UUID
# Once done this will define
#  UUID_FOUND
#  UUID_INCLUDE_DIRS
#  UUID_LIBRARIES
#  UUID_DEFINITIONS

find_package(PkgConfig)
pkg_check_modules(PC_UUID QUIET libuuid)
set(UUID_DEFINITIONS ${PC_UUID_CFLAGS_OTHER})

find_path(UUID_INCLUDE_DIR uuid/uuid.h HINTS ${PC_UUID_INCLUDEDIR} ${PC_UUID_INCLUDE_DIRS} PATH_SUFFIXES uuid)

find_library(UUID_LIBRARY NAMES uuid libuuid HINTS ${PC_UUID_LIBDIR} ${PC_UUID_LIBRARY_DIRS})

set(UUID_LIBRARIES ${UUID_LIBRARY})
set(UUID_INCLUDE_DIRS ${UUID_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(UUID DEFAULT_MSG UUID_LIBRARY UUID_INCLUDE_DIR)

mark_as_advanced(UUID_INCLUDE_DIR UUID_LIBRARY)
