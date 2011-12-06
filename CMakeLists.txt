#
# Master Opencog CMake file.
#
# General organization:
# -- check for different compilers, OS'es
# -- search for various required & optional libraries/tools
# -- decide what to build based on above results.
# -- configure various config files.
# -- print pretty summary
#
include(lib/Summary.cmake)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
IF (COMMAND CMAKE_POLICY)
	CMAKE_POLICY(SET CMP0003 NEW)
	CMAKE_POLICY(SET CMP0005 OLD)
ENDIF (COMMAND CMAKE_POLICY)

PROJECT(opencog)

################################
# uncomment to be in Release mode
################################
#SET(CMAKE_BUILD_TYPE Release)
##
# uncomment to be in coverage testing mode
#SET(CMAKE_BUILD_TYPE Coverage)
#
# uncomment to build in profile mode
#SET(CMAKE_BUILD_TYPE Profile)

IF (CMAKE_BUILD_TYPE STREQUAL "")
	SET(CMAKE_BUILD_TYPE Debug)
ENDIF (CMAKE_BUILD_TYPE STREQUAL "")

# add the 'lib' dir to cmake's module search path
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/lib/")

# ===============================================================
# Detect different compilers and OS'es, tweak flags as necessary.

IF (CMAKE_COMPILER_IS_GNUCXX)
  IF (APPLE)
    SET(CMAKE_C_FLAGS "-Wall -Wno-long-long -Wno-conversion")
    SET(CMAKE_C_FLAGS_DEBUG "-O0 -g")
    SET(CMAKE_C_FLAGS_RELEASE "-O2 -g0")
    # Vital to do this otherwise unresolved symbols everywhere:
    SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,-flat_namespace,-undefined,dynamic_lookup")
    SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-flat_namespace,-undefined,dynamic_lookup")
  ELSE (APPLE)
    SET(CMAKE_C_FLAGS "-Wall -fPIC")
    SET(CMAKE_C_FLAGS_DEBUG "-g -fstack-protector -pg")
    SET(CMAKE_C_FLAGS_RELEASE "-O2 -fstack-protector")
  ENDIF (APPLE)

  # 1) -Wno-variadic-macros is to avoid warnings regarding using
  # variadic in macro OC_ASSERT (the warning warns that this is only
  # available from C99, lol!)
  #
  # 2) -fopenmp for multithreading support
  #
  # 3) -std=gnu++0x for C++0x and GNU extensions support
  SET(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wno-variadic-macros -fopenmp -std=gnu++0x")

  SET(CMAKE_CXX_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})
  SET(CMAKE_CXX_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})

  # Options for generating gcov code coverage output
  SET(CMAKE_C_FLAGS_COVERAGE "-O0 -g -fprofile-arcs -ftest-coverage -fno-inline")
  SET(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_C_FLAGS_COVERAGE} -fno-default-inline")
  # Might be needed for some combinations of ln and gcc
  IF (CMAKE_BUILD_TYPE STREQUAL "Coverage")
  LINK_LIBRARIES(gcov)
  ENDIF (CMAKE_BUILD_TYPE STREQUAL "Coverage")

  SET(CMAKE_C_FLAGS_PROFILE "-O0 -pg")
  SET(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_C_FLAGS_PROFILE}")
ENDIF (CMAKE_COMPILER_IS_GNUCXX)

IF (CYGWIN)
   ADD_DEFINITIONS(-DCYGWIN)
   SET(WIN32 1)
ENDIF (CYGWIN)

IF (WIN32)
   ADD_DEFINITIONS(-DWIN32)
ENDIF (WIN32)

# this is for the case when the code is compiled under windows but not
# under cygwin
IF (WIN32 AND NOT UNIX)
   ADD_DEFINITIONS(-DWIN32_NOT_UNIX)
ENDIF (WIN32 AND NOT UNIX)

# check dependencies
IF (WIN32 AND NOT UNIX)
	FIND_PACKAGE(PThreads REQUIRED)
	FIND_PACKAGE(STLPort REQUIRED)
	INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/win32" ${PTHREADS_INCLUDE_DIR} ${STLPORT_INCLUDE_DIR})
	LINK_LIBRARIES(${PTHREADS_LIBRARY} ${STLPORT_LIBRARIES})
	ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE)
ELSE (WIN32 AND NOT UNIX)
	LINK_LIBRARIES(pthread)
	IF (NOT APPLE AND NOT WIN32)
	    SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--enable-new-dtags")
	ENDIF (NOT APPLE AND NOT WIN32)
ENDIF (WIN32 AND NOT UNIX)

ADD_DEFINITIONS(-DPROJECT_SOURCE_DIR=\\"${CMAKE_SOURCE_DIR}\\"
                -DPROJECT_BINARY_DIR=\\"${CMAKE_BINARY_DIR}\\")

# ===================================================================
# Check for existance of various required, optional packages.

# Check for boost. We need dynamic-linked, threaded libs by default.
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREADED ON)

# Required boost packages
# hypertable requires boost iostreams
FIND_PACKAGE(Boost 1.45 COMPONENTS date_time filesystem iostreams regex serialization signals system thread REQUIRED)

# Optional boost packages; can build without these.
# Arghhh. Except cmake is treating these as required, not optional. #$%**&
FIND_PACKAGE(Boost 1.45 COMPONENTS python program_options QUIET)
FIND_PACKAGE(Boost 1.45 COMPONENTS math_c99 QUIET)

IF(Boost_PROGRAM_OPTIONS_FOUND)
  MESSAGE(STATUS "Found Boost::program_options")
ELSE(Boost_PROGRAM_OPTIONS_FOUND)
  MESSAGE(STATUS "Boost program_options missing: needed for Spatial tools (MapTools).")
ENDIF(Boost_PROGRAM_OPTIONS_FOUND)

IF(Boost_MATH_C99_FOUND)
	MESSAGE(STATUS "Found Boost::math")
ELSE(Boost_MATH_C99_FOUND)
	MESSAGE(STATUS "Boost math missing: needed to run combo learning tests.")
ENDIF(Boost_MATH_C99_FOUND)

FIND_PACKAGE(CURL)
IF (NOT CURL_FOUND)
	MESSAGE(STATUS "libcurl missing: needed for ubigraph module.")
ENDIF (NOT CURL_FOUND)

# XML parsing library
FIND_PACKAGE(EXPAT)
IF (EXPAT_FOUND)
	ADD_DEFINITIONS(-DHAVE_EXPAT)
	SET(HAVE_EXPAT 1)
ELSE (EXPAT_FOUND)
	MESSAGE(STATUS "Expat XML parser missing: needed for hypertable support.")
ENDIF (EXPAT_FOUND)

# Gnu Scientific Library
FIND_PACKAGE(GSL)
IF (GSL_FOUND)
	ADD_DEFINITIONS(-DHAVE_GSL)
	SET(HAVE_GSL 1)
ENDIF (GSL_FOUND)

# This is required for Guile
FIND_LIBRARY(GMP_LIBRARY gmp)
FIND_PATH(GMP_INCLUDE_DIR gmp.h)

# Gnu Guile scheme interpreter
# Versions prior to 1.8.6 have assorted serious problems with
# multi-threading.  If you need guile, you need this version.
FIND_PACKAGE(Guile 1.8.6)
IF (GUILE_FOUND AND GMP_LIBRARY AND GMP_INCLUDE_DIR)
	ADD_DEFINITIONS(-DHAVE_GUILE)
	SET(HAVE_GUILE 1)
	SET(GUILE_DIR_MESSAGE "Guile was found.")
ELSE (GUILE_FOUND AND GMP_LIBRARY AND GMP_INCLUDE_DIR)
	SET(GUILE_DIR_MESSAGE "Guile was not found; the scheme shell will not be built.\nTo over-ride, make sure GUILE_LIBRARIES and GUILE_INCLUDE_DIRS are set.")
ENDIF (GUILE_FOUND AND GMP_LIBRARY AND GMP_INCLUDE_DIR)
MESSAGE(STATUS "${GUILE_DIR_MESSAGE}")

# Hypertable library, for atomspace persistence
FIND_PACKAGE(HyperTable)
IF (HYPERTABLE_FOUND)
	ADD_DEFINITIONS(-DHAVE_HYPERTABLE)
	SET(HAVE_HYPERTABLE 1)
	MESSAGE(STATUS "Hypertable was found.")

	# Don't bother looking for SIGAR unlesss hyperable was found.
	FIND_PACKAGE(SIGAR)
	IF (SIGAR_FOUND)
	ELSE (SIGAR_FOUND)
        MESSAGE(WARNING "SIGAR was not found! It is needed for Hypertable support.")
	ENDIF (SIGAR_FOUND)

ELSE (HYPERTABLE_FOUND)
	# The FindHypertable.cmake file already prints a message.
	# MESSAGE(STATUS "Hypertable was not found; hypertable support will not be built.\n   To over-ride, make sure HYPERTABLE_LIBRARIES and HYPERTABLE_INCLUDE_DIRS\n   are set.")
	MESSAGE(STATUS "Hypertable not found; the experimental distributed persistence DB needs it.")

ENDIF (HYPERTABLE_FOUND)

# OpenGL needed for visualization...
FIND_PACKAGE(OpenGL)

IF (OPENGL_FOUND)
	MESSAGE(STATUS "OpenGL was found.")
ELSE (OPENGL_FOUND)
	MESSAGE(STATUS "OpenGL missing: needed for Spatial tools (MapExplorer, MapTools, etc.).")
ENDIF (OPENGL_FOUND)

# At this time, there is no reason to build with memcached for
# FIND_PACKAGE(LibMemCached)

# Google Protobuf library
# The protocol buffer compiler is a separately installable package, and
# it is also needed for building embodiment PAI. So check for that, too.
FIND_PACKAGE(Protobuf)
IF (PROTOBUF_FOUND
		AND PROTOBUF_LIBRARY
		AND PROTOBUF_INCLUDE_DIR
		AND PROTOBUF_PROTOC_EXECUTABLE)

	ADD_DEFINITIONS(-DHAVE_PROTOBUF)
	SET(HAVE_PROTOBUF 1)
	SET(PROTOBUF_DIR_MESSAGE "Protobuf was found.")

ELSE (PROTOBUF_FOUND
		AND PROTOBUF_LIBRARY
		AND PROTOBUF_INCLUDE_DIR
		AND PROTOBUF_PROTOC_EXECUTABLE)

	# If we are here, then we are missing either the libraries, or the
	# compiler. React appropriately.
	IF (PROTOBUF_PROTOC_EXECUTABLE)
		SET(PROTOBUF_DIR_MESSAGE "Protobuf was not found; the new message decoding functions would not work.\n   To over-ride, make sure PROTOBUF_LIBRARIES and PROTOBUF_INCLUDE_DIRS are set.")
	ELSE (PROTOBUF_PROTOC_EXECUTABLE)
		SET(PROTOBUF_DIR_MESSAGE "Protobuf compiler was not found; embodiment PAI can't be built without it.\n   To over-ride, make sure PROTOBUF_PROTOC_EXECUTABLE is set.")
	ENDIF (PROTOBUF_PROTOC_EXECUTABLE)

ENDIF (PROTOBUF_FOUND
		AND PROTOBUF_LIBRARY
		AND PROTOBUF_INCLUDE_DIR
		AND PROTOBUF_PROTOC_EXECUTABLE)

MESSAGE(STATUS "${PROTOBUF_DIR_MESSAGE}")

# Python bindings using Cython
# Needed for testing
FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)
# Cython is used to generate bindings
FIND_PROGRAM(CYTHON_EXECUTABLE cython)
FIND_PROGRAM(NOSETESTS_EXECUTABLE nosetests)
IF (CYTHON_EXECUTABLE AND PYTHONLIBS_FOUND)
   # find python destination dir for python bindings
   ADD_DEFINITIONS(-DHAVE_CYTHON)
   SET(HAVE_CYTHON 1)
   EXECUTE_PROCESS(
	   COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import *; print get_python_lib()"
		OUTPUT_VARIABLE PYTHON_DEST
   )

   # replace new line at end
   STRING(REPLACE "\n" "" PYTHON_DEST "${PYTHON_DEST}")
   IF ("${PYTHON_DEST}" STREQUAL "")
	   MESSAGE(FATAL_ERROR "Python destination dir not found")
   ELSE ("${PYTHON_DEST}" STREQUAL "")
	   MESSAGE(STATUS "Python destination dir found: ${PYTHON_DEST}" )
   ENDIF ("${PYTHON_DEST}" STREQUAL "")
ELSE (CYTHON_EXECUTABLE AND PYTHONLIBS_FOUND)
	MESSAGE(STATUS "Neither Cython or Python libraries not found.")
ENDIF (CYTHON_EXECUTABLE AND PYTHONLIBS_FOUND)

# SDL needed for the spatial tools
FIND_PACKAGE(SDL)

IF (SDL_FOUND)
	MESSAGE(STATUS "SDL was found.")
ELSE (SDL_FOUND)
	MESSAGE(STATUS "SDL missing: needed for Spatial tools (MapExplorer, MapTools, etc.).")
ENDIF (SDL_FOUND)

FIND_PACKAGE(SDL_gfx)

IF (SDLGFX_FOUND)
	MESSAGE(STATUS "SDL_gfx was found")
ELSE (SDLGFX_FOUND)
	MESSAGE(STATUS "SDL_gfx missing: needed for Spatial tools (MapExplorer, MapTools, etc.).")
ENDIF (SDLGFX_FOUND)

FIND_PACKAGE(TCL)

# /usr/bin/tclsh is expected for some embodiment tests, but is
# not otherwise important for anything...
IF (TCL_TCLSH)
	MESSAGE(STATUS "TCL shell was found here: ${TCL_TCLSH}")
ELSE (TCL_TCLSH)
	MESSAGE(STATUS "TCL shell missing: needed for some embodiment tests.")
ENDIF (TCL_TCLSH)

# Enable the hypergraph persistence module, if unixodbc is found.
# In principle, iODBC could be used; however, iODBC is buggy when
# used with UTF8/unicode node names, and fails miserably. Thus,
# only UnixODBC is supported at this time.
#
# FIND_PACKAGE(IODBC QUIET)
# IF (IODBC_FOUND)
# 	ADD_DEFINITIONS(-DHAVE_SQL_STORAGE)
# 	SET(ODBC_FOUND 1)
# 	SET(ODBC_INCLUDE_DIRS ${IODBC_INCLUDE_DIRS})
# 	SET(ODBC_LIBRARIES ${IODBC_LIBRARIES})
# ENDIF (IODBC_FOUND)

FIND_PACKAGE(UnixODBC QUIET)
IF (UnixODBC_FOUND)
	ADD_DEFINITIONS(-DHAVE_SQL_STORAGE)
    SET(HAVE_SQL_STORAGE 1)
	SET(ODBC_FOUND 1)
	SET(ODBC_INCLUDE_DIRS ${UnixODBC_INCLUDE_DIRS})
	SET(ODBC_LIBRARIES ${UnixODBC_LIBRARIES})
	SET(ODBC_DIR_MESSAGE "UnixODBC was found.")
ELSE (UnixODBC_FOUND)
	SET(ODBC_FOUND 0)
	SET(ODBC_DIR_MESSAGE "UnixODBC was not found; persistence subsystem will not be built.\nTo over-ride, make sure UnixODBC_LIBRARIES and UnixODBC_INCLUDE_DIRS are set.")
ENDIF (UnixODBC_FOUND)
MESSAGE(STATUS "${ODBC_DIR_MESSAGE}")

# These are required for building the embodiment code
FIND_PACKAGE(XercesC)
IF (XERCESC_FOUND)
 # using COMPARE_VERSION_STRINGS instead of COMPARE_VERSION for staying compatible with cmake 2.4
 IF ("${XERCESC_VERSION}" VERSION_LESS "3.0.0")
   MESSAGE(STATUS "Found Xerces ${XERCESC_VERSION} -- ${XERCESC_LIBRARIES}")
   INCLUDE_DIRECTORIES(${XERCESC_INCLUDE})
   SET(XERCESC_VERSION_OK TRUE)
 ELSE ("${XERCESC_VERSION}" VERSION_LESS "3.0.0")
   MESSAGE(WARNING "Xerces ${XERCESC_VERSION} not supported. Only versions 2.* are supported. Embodiment won't be built.")
   SET(XERCESC_VERSION_OK FALSE)
 ENDIF ("${XERCESC_VERSION}" VERSION_LESS "3.0.0")
ENDIF (XERCESC_FOUND)

# XML-RPC, used by the ubigraph module
FIND_PACKAGE(XMLRPC COMPONENTS client)
IF (NOT XMLRPC_FOUND)
    MESSAGE(STATUS "libxmlrpc missing: needed for ubigraph module.")
ENDIF (NOT XMLRPC_FOUND)

# ZeroMQ library 2.1.2
FIND_PACKAGE(ZMQ)
IF (ZMQ_FOUND AND ZMQ_LIBRARY AND ZMQ_INCLUDE_DIR)
	ADD_DEFINITIONS(-DHAVE_ZMQ)
    SET(HAVE_ZMQ 1)
	SET(ZMQ_DIR_MESSAGE "ZeroMQ was found.")
ELSE (ZMQ_FOUND AND ZMQ_LIBRARY AND ZMQ_INCLUDE_DIR)
	SET(ZMQ_DIR_MESSAGE "ZeroMQ was not found; the message system used by GUI monitor for OAC will not be built.\nTo over-ride, make sure ZMQ_LIBRARIES and ZMQ_INCLUDE_DIRS are set.")
ENDIF (ZMQ_FOUND AND ZMQ_LIBRARY AND ZMQ_INCLUDE_DIR)
MESSAGE(STATUS "${ZMQ_DIR_MESSAGE}")

# ==========================================================
# Decide what to build, based on the packages found.

IF(Boost_FOUND)
    SET(HAVE_UTIL 1)
    SET(HAVE_COMBOREDUCT 1)
    SET(HAVE_MOSES 1)
    SET(HAVE_FEATURE_SELECTION 1)
ENDIF(Boost_FOUND)

IF(GSL_FOUND)
    SET(HAVE_ATOMSPACE 1)
    SET(HAVE_ATTENTION 1)
    SET(HAVE_PLN 1)
    SET(HAVE_DIMEMBED 1)
ENDIF(GSL_FOUND)

# Always want to build spatial dir for the AtomSpace
# as it depends on libSpaceMap and guards against compiling
# stuff requiring opengl and sdl in it's own CMakeLists.txt
IF(HAVE_ATOMSPACE)
    SET(HAVE_SPATIAL 1)
ELSE(HAVE_ATOMSPACE)
    IF(OPENGL_FOUND AND SDL_FOUND AND SDLGFX_FOUND)
        SET(HAVE_SPATIAL 1)
    ENDIF(OPENGL_FOUND AND SDL_FOUND AND SDLGFX_FOUND)
ENDIF(HAVE_ATOMSPACE)

IF(EXPAT_FOUND AND HAVE_ATOMSPACE)
    SET(HAVE_PERSIST 1)
ENDIF(EXPAT_FOUND AND HAVE_ATOMSPACE)

IF(HAVE_PERSIST AND HAVE_ATOMSPACE)
    SET(HAVE_SERVER 1)
ENDIF(HAVE_PERSIST AND HAVE_ATOMSPACE)

IF(HAVE_SERVER) # because web depends on server
    SET(HAVE_WEB 1)
ENDIF(HAVE_SERVER)


IF(XERCESC_VERSION_OK AND EXPAT_FOUND AND HAVE_PROTOBUF)
   SET(HAVE_EMBODIMENT 1)
ENDIF(XERCESC_VERSION_OK AND EXPAT_FOUND AND HAVE_PROTOBUF)

IF (CURL_FOUND AND XMLRPC_FOUND)
	ADD_DEFINITIONS(-DHAVE_UBIGRAPH)
	SET(HAVE_UBIGRAPH 1)
ENDIF (CURL_FOUND AND XMLRPC_FOUND)

# At this time, there is no reason to build with memcached for
# any "normal" use, so just stub it out.
# IF (LIBMEMCACHED_FOUND)
#	ADD_DEFINITIONS(-DHAVE_LIBMEMCACHED)
# ENDIF (LIBMEMCACHED_FOUND)

# ===================================================================
# global includes

# set confdir and datadir
IF (NOT DEFINED CONFDIR)
    SET (CONFDIR "${CMAKE_INSTALL_PREFIX}/etc")
ENDIF (NOT DEFINED CONFDIR)
IF (NOT DEFINED DATADIR)
    SET (DATADIR "${CMAKE_INSTALL_PREFIX}/share/opencog")
ENDIF (NOT DEFINED DATADIR)
ADD_DEFINITIONS(-DCONFDIR=\\"${CONFDIR}\\")
ADD_DEFINITIONS(-DDATADIR=\\"${DATADIR}\\")

# include custom opencog cmake macros
INCLUDE("${CMAKE_SOURCE_DIR}/lib/OpenCogMacros.cmake")

# small hack to handle unixes that use "/usr/lib64" instead of "/usr/lib" as the
# default lib path on 64 bit archs
IF (NOT DEFINED LIB_DIR_SUFFIX)
	EXECUTE_PROCESS(COMMAND ${CMAKE_CXX_COMPILER} -print-search-dirs OUTPUT_VARIABLE PRINT_SEARCH_DIRS_OUTPUT)
	STRING(REGEX MATCH "\r?\nlibraries:.*\r?\n" COMPILER_LIB_SEARCH_DIRS ${PRINT_SEARCH_DIRS_OUTPUT})
	STRING(REGEX MATCH "/lib64/:|/lib64:|/lib64\n" HAS_LIB64 ${COMPILER_LIB_SEARCH_DIRS})
	IF (NOT ${HAS_LIB64} STREQUAL "")
		SET(LIB_DIR_SUFFIX "64")
	ENDIF (NOT ${HAS_LIB64} STREQUAL "")
ENDIF (NOT DEFINED LIB_DIR_SUFFIX)

# set default include paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${Boost_INCLUDE_DIRS})

# rpath handling
SET(CMAKE_SKIP_BUILD_RPATH	FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/opencog")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

ADD_SUBDIRECTORY(opencog)
ADD_SUBDIRECTORY(lib)

ADD_CUSTOM_TARGET(tests)
ADD_SUBDIRECTORY(tests EXCLUDE_FROM_ALL)
IF (CMAKE_BUILD_TYPE STREQUAL "Coverage")
    # doing coverage stuff while running tests if this is the Coverage build
    ADD_CUSTOM_TARGET(test
        # TODO lcov should be found by cmake first
        # TODO set it up so that we can pick to run coverage per test, or
        # combined across all tests (the latter is MUCH faster). Use a define?
        # There is coverage specific stuff in AddCxxTest.cmake now...
        # -
        # Depends on cogserver because RESTFulTest needs it
        # and cmake has no way to add non-test dependencies to tests
        DEPENDS tests cogserver
        WORKING_DIRECTORY tests
        COMMAND ${CMAKE_CTEST_COMMAND} --force-new-ctest-process $(ARGS)
        # This script combines the coverage analysis of each test,
        # then creates html in tests/lcov
        # Note: this should now be run separately...
        #COMMAND ${PROJECT_SOURCE_DIR}/scripts/combine_lcov.sh
        COMMENT "Running tests with coverage..."
    )
ELSE (CMAKE_BUILD_TYPE STREQUAL "Coverage")
    # If this is a build with coverage enabled then test normally
    ADD_CUSTOM_TARGET(test
        DEPENDS tests
        WORKING_DIRECTORY tests
        COMMAND ${CMAKE_CTEST_COMMAND} --force-new-ctest-process $(ARGS)
        COMMENT "Running tests..."
    )
ENDIF (CMAKE_BUILD_TYPE STREQUAL "Coverage")

ADD_SUBDIRECTORY(examples EXCLUDE_FROM_ALL)
IF (NOT WIN32)
	ADD_CUSTOM_TARGET (examples
		COMMAND ${CMAKE_BUILD_TOOL}
		WORKING_DIRECTORY examples
		COMMENT "Building examples"
	)
ENDIF (NOT WIN32)

ADD_CUSTOM_TARGET(cscope
    COMMAND find opencog examples tests -name '*.cc' -o -name '*.h' -o -name '*.cxxtest' -o -name '*.scm' > ${CMAKE_SOURCE_DIR}/cscope.files
    COMMAND cscope -b
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating CScope database"
)

# packaging
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Cognition Framework")
SET(CPACK_PACKAGE_NAME "opencog")
SET(CPACK_PACKAGE_VENDOR "opencog.org")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
SET(CPACK_PACKAGE_VERSION_MAJOR "0")
SET(CPACK_PACKAGE_VERSION_MINOR "1")
SET(CPACK_PACKAGE_VERSION_PATCH "4")
SET(CPACK_BINARY_STGZ "OFF")
SET(CPACK_BINARY_TBZ2 "OFF")
SET(CPACK_BINARY_TGZ "OFF")
SET(CPACK_BINARY_TZ "OFF")
SET(CPACK_SOURCE_STGZ "OFF")
SET(CPACK_SOURCE_TBZ2 "ON")
SET(CPACK_SOURCE_TGZ "OFF")
SET(CPACK_SOURCE_TZ "OFF")
SET(CPACK_GENERATOR "TBZ2")
SET(CPACK_SOURCE_GENERATOR "TBZ2")
SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}")
SET(CPACK_SOURCE_IGNORE_FILES
	"~$"
	".a$"
	".so$"
	".log$"
	".tar.z$"
	".tar.gz$"
	".tar.bz2$"
	"/Makefile$"
	"/CMakeFiles/"
	"/CMakeCache.txt$"
	"/DartTestfile.txt$"
	"/cmake_install.cmake$"
	"/Testfile.cmake$"
	"/CTestTestfile.cmake$"
	"/CTestTestfile.cmake$"
	"UTest$"
	"UTest.cpp$"
	"/_CPack_Packages/"
	"/CPackConfig.cmake$"
	"/CPackSourceConfig.cmake$"
	"/sniff$"
	"/install_manifest.txt$"
	"${CPACK_PACKAGE_FILE_NAME}"
	"^${PROJECT_SOURCE_DIR}/debian/"
	"^${PROJECT_SOURCE_DIR}/.*spec$"
	"^${PROJECT_SOURCE_DIR}/vendor/"
	"^${PROJECT_SOURCE_DIR}/.bzr/"
	"^${PROJECT_SOURCE_DIR}/.bzrignore$"
	"^${PROJECT_BINARY_DIR}/opencog/server/cogserver$"
	"^${PROJECT_BINARY_DIR}/examples/server/derivedserver$"
	"^${PROJECT_BINARY_DIR}/examples/hopfield/hopfield$"
)
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
IF (WIN32 AND NOT UNIX)
  # There is a bug in NSI that does not handle full unix paths properly. Make
  # sure there is at least one set of four (4) backlasshes.
  SET(CPACK_PACKAGE_ICON "${CMake_SOURCE_DIR}\\\\lib\\\\opencog.ico")
  SET(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\cogserver.exe")
  SET(CPACK_NSIS_DISPLAY_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY} The Open Cognition Framework")
  SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\www.opencog.org")
  SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.opencog.org")
  SET(CPACK_NSIS_CONTACT "gustavo.gama@gmail.com")
  SET(CPACK_NSIS_MODIFY_PATH ON)
ELSE(WIN32 AND NOT UNIX)
  SET(CPACK_STRIP_FILES "bin/cogserver")
  SET(CPACK_SOURCE_STRIP_FILES "")
ENDIF(WIN32 AND NOT UNIX)
SET(CPACK_PACKAGE_EXECUTABLES "cogserver" "The Open Cognition Framework")
INCLUDE(CPack)

# documentation
FIND_PACKAGE(Doxygen)
IF (DOXYGEN_FOUND)
    # doc dir has no CMakeList.txt
    #ADD_SUBDIRECTORY(doc EXCLUDE_FROM_ALL)
	EXECUTE_PROCESS(
		WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
		COMMAND bzr nick
		OUTPUT_VARIABLE BZR_NICK OUTPUT_STRIP_TRAILING_WHITESPACE)
	EXECUTE_PROCESS(
		WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
		COMMAND bzr revno
		OUTPUT_VARIABLE BZR_REVNO OUTPUT_STRIP_TRAILING_WHITESPACE)
	CONFIGURE_FILE(
		${PROJECT_SOURCE_DIR}/doc/doxygen.conf.in
		${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	ADD_CUSTOM_TARGET (
		doxygen
		COMMAND ${DOXYGEN} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating documentation"
	)
ELSE (DOXYGEN_FOUND)
    MESSAGE(STATUS "Doxygen not found, you won't be able to generate API documentation.")
ENDIF (DOXYGEN_FOUND)

# Show a summary of what we got


SUMMARY_ADD("Attention" "Agents for attention allocation dynamics" HAVE_ATTENTION)
SUMMARY_ADD("AtomSpace" "A weighted and typed hypergraph database" HAVE_ATOMSPACE)
SUMMARY_ADD("ComboReduct" "Library for reduction of combo program trees" HAVE_COMBOREDUCT)
SUMMARY_ADD("CythonBinding" "Cython (python) bindings" HAVE_CYTHON)
SUMMARY_ADD("Dim. Embedding" "Compute dimensional embedding of the AtomSpace" HAVE_DIMEMBED)
SUMMARY_ADD("Doxygen" "Code documentation" DOXYGEN_FOUND)
SUMMARY_ADD("Embodiment" "Architecture for embodiment" HAVE_EMBODIMENT)
SUMMARY_ADD("FeatureSelection" "MOSES based feature selection tool" HAVE_FEATURE_SELECTION)
SUMMARY_ADD("Hypertable" "HyperTable for scalable persistance (experimental)" HAVE_HYPERTABLE)
SUMMARY_ADD("MOSES" "Meta-Optimizing Semantic Evolutionary Search" HAVE_MOSES)
SUMMARY_ADD("Persist" "Interfaces for persistance of the AtomSpace" HAVE_PERSIST)
SUMMARY_ADD("PLN" "Probalistic Logic Networks" HAVE_PLN)
SUMMARY_ADD("SchemeBinding" "Scheme bindings and shell" HAVE_GUILE)
SUMMARY_ADD("Server" "The CogServer" HAVE_SERVER)
SUMMARY_ADD("Spatial" "Spatial tools" HAVE_SPATIAL)
SUMMARY_ADD("Ubigraph" "Visualisation of the AtomSpace and PLN's BIT" HAVE_UBIGRAPH)
SUMMARY_ADD("Util" "General utility library" HAVE_UTIL)
SUMMARY_ADD("WebInterface" "Web server and REST API" HAVE_WEB)
SUMMARY_SHOW()
