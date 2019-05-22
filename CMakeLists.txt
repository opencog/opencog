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

# cogutils already requires 2.8.12.2, so may as well ask for that.
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.2)
IF (COMMAND CMAKE_POLICY)
	CMAKE_POLICY(SET CMP0003 NEW)
ENDIF (COMMAND CMAKE_POLICY)

IF(CMAKE_VERSION VERSION_GREATER 3.0.2)
	CMAKE_POLICY(SET CMP0037 OLD)
ENDIF(CMAKE_VERSION VERSION_GREATER 3.0.2)

PROJECT(opencog)

# ----------------------------------------------------------
# User-modifiable options. Feel free to change these!
#
# uncomment to be in Release mode [default]
# SET(CMAKE_BUILD_TYPE Release)

# uncomment to build in debug mode
# SET(CMAKE_BUILD_TYPE Debug)

# uncomment to be in coverage testing mode
# SET(CMAKE_BUILD_TYPE Coverage)

# uncomment to build in profile mode
# SET(CMAKE_BUILD_TYPE Profile)

# uncomment to build in release mode with debug information
# SET(CMAKE_BUILD_TYPE RelWithDebInfo)

# default build type
IF (CMAKE_BUILD_TYPE STREQUAL "")
	SET(CMAKE_BUILD_TYPE Release)
ENDIF (CMAKE_BUILD_TYPE STREQUAL "")

MESSAGE(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

ADD_DEFINITIONS(-DPROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
                -DPROJECT_BINARY_DIR="${CMAKE_BINARY_DIR}")

# ===============================================================
# Detect different compilers and OS'es, tweak flags as necessary.

IF (CMAKE_COMPILER_IS_GNUCXX)
	# Atomspace is already asking for version 5.0, so ask for that.
	IF (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0)
		MESSAGE(FATAL_ERROR "GCC version must be at least 5.0!")
	ENDIF (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.0)

	IF (APPLE)
		SET(CMAKE_C_FLAGS "-Wall -Wno-long-long -Wno-conversion")
		SET(CMAKE_C_FLAGS_DEBUG "-O0 -g")
		SET(CMAKE_C_FLAGS_PROFILE "-O0 -pg")
		SET(CMAKE_C_FLAGS_RELEASE "-O2 -g0")
		# Vital to do this otherwise unresolved symbols everywhere:
		SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,-flat_namespace,-undefined,dynamic_lookup")
		SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-flat_namespace,-undefined,dynamic_lookup")

		# The Apple linker does not support the --no-as-needed flag.
		SET(NO_AS_NEEDED "")

	ELSE (APPLE)
		SET(CMAKE_C_FLAGS "-Wall -fPIC")
		# SET(CMAKE_C_FLAGS "-Wl,--copy-dt-needed-entries")
		SET(CMAKE_C_FLAGS_DEBUG "-ggdb3 -fstack-protector")
		SET(CMAKE_C_FLAGS_PROFILE "-O2 -g3 -fstack-protector -pg")
		SET(CMAKE_C_FLAGS_RELEASE "-O2 -g -fstack-protector")

		SET(NO_AS_NEEDED "-Wl,--no-as-needed")
	ENDIF (APPLE)

	# 1) -Wno-variadic-macros is to avoid warnings regarding using
	# variadic in macro OC_ASSERT (the warning warns that this is only
	# available from C99, lol!)
	#
	# 2) -fopenmp for multithreading support
	#
	# 3) -std=gnu++11 for C++11 and GNU extensions support
	SET(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wno-variadic-macros -fopenmp -std=gnu++11")

	SET(CMAKE_CXX_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})
	SET(CMAKE_CXX_FLAGS_PROFILE ${CMAKE_C_FLAGS_PROFILE})
	SET(CMAKE_CXX_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})

	# Options for generating gcov code coverage output
	SET(CMAKE_C_FLAGS_COVERAGE "-O0 -g -fprofile-arcs -ftest-coverage -fno-inline")
	SET(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_C_FLAGS_COVERAGE} -fno-default-inline")
	# Might be needed for some combinations of ln and gcc
	IF (CMAKE_BUILD_TYPE STREQUAL "Coverage")
		LINK_LIBRARIES(gcov)
	ENDIF (CMAKE_BUILD_TYPE STREQUAL "Coverage")
ENDIF (CMAKE_COMPILER_IS_GNUCXX)

# ----------------------------------------------------
# Do the windows build.

IF (WIN32)
	ADD_DEFINITIONS(-DWIN32)
	IF (CYGWIN)
		ADD_DEFINITIONS(-DCYGWIN)
	#   SET(WIN32 1)
	ENDIF (CYGWIN)

	# It is "not unix" when the code is compiled under windows but not
	# under cygwin
	IF (NOT UNIX)
		ADD_DEFINITIONS(-DWIN32_NOT_UNIX)
		FIND_PACKAGE(PThreads REQUIRED)
		FIND_PACKAGE(STLPort REQUIRED)
		INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/win32" ${PTHREADS_INCLUDE_DIR} ${STLPORT_INCLUDE_DIR})
		LINK_LIBRARIES(${PTHREADS_LIBRARY} ${STLPORT_LIBRARIES})
		ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE)
	ELSE (NOT UNIX)
		LINK_LIBRARIES(pthread)
		IF (CYGWIN)
			SET(CMAKE_SHARED_LINKER_FLAGS "-Wl")
		ELSE (CYGWIN)
			SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--enable-new-dtags")
		ENDIF (CYGWIN)
	ENDIF (NOT UNIX)
ENDIF (WIN32)

# ===============================================================
# Check for existance of various required, optional packages.
# Listed in alphabetical order, more or less.
# CogUtil must come first, because it supplies various FindXXX macros.

# Add the 'lib' dir to cmake's module search path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/lib/")

# Cogutil
FIND_PACKAGE(CogUtil 2.0.1 CONFIG REQUIRED)
IF (COGUTIL_FOUND)
    MESSAGE(STATUS "CogUtil found.")
    ADD_DEFINITIONS(-DHAVE_COGUTIL)
    SET(HAVE_COGUTIL 1)
ELSE (COGUTIL_FOUND)
    MESSAGE(FATAL_ERROR "CogUtil missing: it is needed!")
ENDIF (COGUTIL_FOUND)

# add the 'cmake' directory from cogutil to search path
list(APPEND CMAKE_MODULE_PATH  ${COGUTIL_DATA_DIR}/cmake)

include(${COGUTIL_DATA_DIR}/cmake/Summary.cmake)

# ===================================================================
# Check for existance of various required, optional packages.

# AtomSpace
FIND_PACKAGE(AtomSpace 5.0.3 CONFIG REQUIRED)
IF (ATOMSPACE_FOUND)
	MESSAGE(STATUS "AtomSpace found.")
	ADD_DEFINITIONS(-DHAVE_ATOMSPACE)
	SET(HAVE_ATOMSPACE 1)
ELSE (ATOMSPACE_FOUND)
	MESSAGE(FATAL_ERROR "AtomSpace missing: it is needed!")
ENDIF (ATOMSPACE_FOUND)

# ----------------------------------------------------------
# Check for boost. We need dynamic-linked, threaded libs by default.
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREADED ON)
SET(MIN_BOOST 1.46)

# Required boost packages
FIND_PACKAGE(Boost ${MIN_BOOST} COMPONENTS date_time filesystem program_options regex serialization system thread REQUIRED)

IF(Boost_FOUND)
	SET(Boost_FOUND_SAVE 1)
ELSE(Boost_FOUND)
	MESSAGE(FATAL_ERROR "Boost ${MIN_BOOST} or newer is needed to build OpenCog!")
ENDIF(Boost_FOUND)

# Opencog won't compile with Boost 1.51, some kind of conflict with
# hash functions, see github bugs 1 and 36
IF(105100 EQUAL ${Boost_VERSION})
	MESSAGE(FATAL_ERROR "Boost version 1.51 will not work with OpenCog.  Please use a different version.")
ENDIF(105100 EQUAL ${Boost_VERSION})
MESSAGE(STATUS "Boost version ${Boost_VERSION} found.")

# Optional boost packages; can build without these.
FIND_PACKAGE(Boost ${MIN_BOOST} COMPONENTS python program_options QUIET)
FIND_PACKAGE(Boost ${MIN_BOOST} COMPONENTS math_c99 QUIET)

# Arghhh. Except cmake is treating above as required, not optional. #$%**&
IF(Boost_FOUND_SAVE)
	SET(Boost_FOUND 1)
ENDIF(Boost_FOUND_SAVE)

# ----------------------------------------------------------
# cpprest
# XXX FIXME This is required only by the soon-to-be obsolete
# PatternMiner. Remove this when the (old) PatternMiner is gone.
FIND_PACKAGE(cpprest 2.6)
IF (cpprest_FOUND AND cpprest_LIBRARY)
	ADD_DEFINITIONS(-DHAVE_cpprest)
	SET(HAVE_cpprest 1)
	SET(cpprest_DIR_MESSAGE "cpprest was found.")
ELSE (cpprest_FOUND AND cpprest_LIBRARY)
        SET(cpprest_DIR_MESSAGE "cpprest was not found; Distributed version of PatternMiner will not be built.\nTo over-ride, make sure that the environment variable cpprest_LIBRARY is set.\nInstallation instructions: https://casablanca.codeplex.com/")
ENDIF (cpprest_FOUND AND cpprest_LIBRARY)
MESSAGE(STATUS "${cpprest_DIR_MESSAGE}")

# ----------------------------------------------------------
# Needed for unit tests

FIND_PACKAGE(Cxxtest)
IF (NOT CXXTEST_FOUND)
	MESSAGE(STATUS "CxxTest missing: needed for unit tests.")
ENDIF (NOT CXXTEST_FOUND)

# ----------------------------------------------------------
# Glasgow Haskell compiler
FIND_PACKAGE(GHC)
IF (GHC_FOUND)
	ADD_DEFINITIONS(-DHAVE_GHC)
	SET(HAVE_GHC 1)
ELSE (GHC_FOUND)
	SET(GHC_DIR_MESSAGE "GHC was not found; Haskell bindings will not be built.")
	MESSAGE(STATUS "${GHC_DIR_MESSAGE}")
ENDIF (GHC_FOUND)

FIND_PACKAGE(Stack)
IF (STACK_FOUND)
	ADD_DEFINITIONS(-DHAVE_STACK)
	SET(HAVE_STACK 1)
	SET(STACK_FOUND_MESSAGE "The Haskell Tool Stack found.")
ELSE (STACK_FOUND)
	SET(STACK_FOUND_MESSAGE "The Haskell Tool Stack not found.")
ENDIF (STACK_FOUND)
MESSAGE(STATUS "${STACK_FOUND_MESSAGE}")

# ----------------------------------------------------------
# This is required for Guile
FIND_LIBRARY(GMP_LIBRARY gmp)
FIND_PATH(GMP_INCLUDE_DIR gmp.h)

# Gtk-3 required for visualization
FIND_PACKAGE(GTK3)
IF (GTK3_FOUND)
	# MESSAGE(STATUS "GTK3 found.")
	ADD_DEFINITIONS(-DHAVE_GTK)
	SET(HAVE_GTK 1)
ELSE (GTK3_FOUND)
	MESSAGE(STATUS "GTK missing: needed for the the gtk-visualizer.")
ENDIF (GTK3_FOUND)

# Gnu Guile scheme interpreter
# Version 2.2.2 is needed for mrs io ports, compilation, atomics, nlp
FIND_PACKAGE(Guile 2.2.2)
IF (GUILE_FOUND AND GMP_LIBRARY AND GMP_INCLUDE_DIR)
	ADD_DEFINITIONS(-DHAVE_GUILE)
	SET(HAVE_GUILE 1)
	INCLUDE_DIRECTORIES(${GUILE_INCLUDE_DIR})
ELSE (GUILE_FOUND AND GMP_LIBRARY AND GMP_INCLUDE_DIR)
	SET(GUILE_DIR_MESSAGE "Guile was not found; the scheme shell will not be built.\nTo over-ride, make sure GUILE_LIBRARIES and GUILE_INCLUDE_DIRS are set.")
	MESSAGE(STATUS "${GUILE_DIR_MESSAGE}")
ENDIF (GUILE_FOUND AND GMP_LIBRARY AND GMP_INCLUDE_DIR)

# ----------------------------------------------------------
# Lapack is needed for BLOPEX
FIND_PACKAGE(LAPACK)
IF (LAPACK_FOUND)
	# MESSAGE(STATUS "LAPACK found.")
	ADD_DEFINITIONS(-DHAVE_LAPACK)
	SET(HAVE_LAPACK 1)
ELSE (LAPACK_FOUND)
	MESSAGE(STATUS "LAPACK missing: needed for blopex-based clustering.")
ENDIF (LAPACK_FOUND)

# ----------------------------------------------------------
# Link-Grammar is needed for several NLP subsystems
# Only 5.4.0 or newer has the required API in it.
FIND_PACKAGE(LinkGrammar 5.4.0)
IF (LINK_GRAMMAR_FOUND)
	SET(HAVE_LINK_GRAMMAR 1)
	INCLUDE_DIRECTORIES(${LINK_GRAMMAR_INCLUDE_DIRS})
ELSE (LINK_GRAMMAR_FOUND)
	MESSAGE(STATUS "Link Grammar missing: needed for NLP.")
ENDIF (LINK_GRAMMAR_FOUND)

FIND_PACKAGE(UUID)
IF (UUID_FOUND)
	SET(HAVE_UUID 1)
	INCLUDE_DIRECTORIES(${UUID_INCLUDE_DIRS})
	ADD_DEFINITIONS(${UUID_DEFINITIONS})
ELSE (UUID_FOUND)
	MESSAGE(STATUS "UUID library missing: needed for NLP.")
ENDIF (UUID_FOUND)

# ----------------------------------------------------------
# Octomap
FIND_PACKAGE(Octomap)
IF(OCTOMAP_FOUND AND OCTOMAP_LIBRARY)
  ADD_DEFINITIONS(-DHAVE_OCTOMAP)
  SET(HAVE_OCTOMAP 1)
  INCLUDE_DIRECTORIES(${OCTOMAP_INCLUDE_DIR})
  SET(OCTOMAP_DIR_MESSAGE "Octomap was found.")
ELSE (OCTOMAP_FOUND AND OCTOMAP_LIBRARY)
  SET(OCTOMAP_DIR_MESSAGE "Octomap missing: needed for the SpaceTime map.")
ENDIF(OCTOMAP_FOUND AND OCTOMAP_LIBRARY)

# ----------------------------------------------------------
# Python and Cython
#
# NOTE: Python interpreter is needed for runing python unit tests,
# and for running the FindCython module.
#
# Search for Python3 first, and use that, if found. Else use Python2.
# To use Python2 only from the build directory run the following
# rm CMakeCache.txt && cmake -DCMAKE_DISABLE_FIND_PACKAGE_Python3Interp=TRUE ..

FIND_PACKAGE(Python3Interp)
IF (3.4.0 VERSION_LESS "${PYTHON3_VERSION_STRING}")
	SET (HAVE_PY_INTERP 1)
	SET (PYTHON_VER python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR})
	MESSAGE(STATUS "Python ${PYTHON3_VERSION_STRING} interpreter found.")
ENDIF()

IF (NOT HAVE_PY_INTERP)
	FIND_PACKAGE(PythonInterp)
	IF (2.7.0 VERSION_LESS ${PYTHON_VERSION_STRING})
		SET (HAVE_PY_INTERP 1)
		SET (PYTHON_VER python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR})
		MESSAGE(STATUS "Python ${PYTHON_VERSION_STRING} interpreter found.")
	ENDIF()
ENDIF()

FIND_PACKAGE(PythonLibs)
IF (PYTHONLIBS_FOUND AND
     ((PYTHON3INTERP_FOUND AND 3.4.0 VERSION_LESS ${PYTHONLIBS_VERSION_STRING})
     OR
     (2.7.0 VERSION_LESS ${PYTHONLIBS_VERSION_STRING})))
	SET (HAVE_PY_LIBS 1)
	MESSAGE(STATUS "Python ${PYTHONLIBS_VERSION_STRING} libraries found.")
ELSE()
	MESSAGE(STATUS "Python libraries NOT found.")
ENDIF()

# Cython is used to generate python bindings.
IF(HAVE_PY_INTERP)
	FIND_PACKAGE(Cython 0.23.0)

	IF (CYTHON_FOUND AND HAVE_PY_LIBS)
		# Find python destination dir for python bindings because it may
		# differ on each operating system.
		ADD_DEFINITIONS(-DHAVE_CYTHON)
		SET(HAVE_CYTHON 1)

		# Hack together some reasonable install location.
		SET(PYTHON_DEST
			"lib${LIB_DIR_SUFFIX}/${PYTHON_VER}/dist-packages/opencog")
		MESSAGE(STATUS
			"Python install dir: ${CMAKE_INSTALL_PREFIX}/${PYTHON_DEST}" )

	ELSE (CYTHON_FOUND AND HAVE_PY_LIBS)
		IF(NOT CYTHON_FOUND)
			MESSAGE(STATUS "Cython executable not found.")
		ENDIF(NOT CYTHON_FOUND)
	ENDIF (CYTHON_FOUND AND HAVE_PY_LIBS)

	# Nosetests will find and automatically run python tests.
	IF (PYTHON3INTERP_FOUND)
		FIND_PROGRAM(NOSETESTS_EXECUTABLE nosetests3)
	ELSE ()
		FIND_PROGRAM(NOSETESTS_EXECUTABLE nosetests-2.7)
	ENDIF ()
	IF (NOSETESTS_EXECUTABLE AND CYTHON_FOUND AND HAVE_PY_LIBS)
		SET(HAVE_NOSETESTS 1)
		MESSAGE(STATUS "Using nosetests executable " ${NOSETESTS_EXECUTABLE})
	ENDIF (NOSETESTS_EXECUTABLE AND CYTHON_FOUND AND HAVE_PY_LIBS)
ENDIF(HAVE_PY_INTERP)

# ----------------------------------------------------------
# Optional, currently needed only to hush up DRD in util/Logger.cc
FIND_PACKAGE(VALGRIND)
IF (VALGRIND_FOUND)
	MESSAGE(STATUS "VALGRIND was found.")
	IF (VALGRIND_INCLUDE_DIR)
		MESSAGE(STATUS "VALGRIND devel headers found.")
		ADD_DEFINITIONS(-DHAVE_VALGRIND)
	ELSE (VALGRIND_INCLUDE_DIR)
		MESSAGE(STATUS "VALGRIND devel headers NOT FOUND: needed for thread debugging.")
	ENDIF (VALGRIND_INCLUDE_DIR)
ELSE (VALGRIND_FOUND)
	MESSAGE(STATUS "VALGRIND missing: needed for thread debugging.")
ENDIF (VALGRIND_FOUND)

# -----------------------------------------------------------------------------
# XXX FIXME. The jsoncpp, TBB and ZeroMQ packages are needed by
# the deprecated and/or obsolete AtomSpacePublisher module.
# I'm not sure, but I suspect that module doesn't work.  It's
# not maintained by anyone; it's certainly out of date, and doesn't
# support any of the modern features. So ....!? FIXME

# jsoncpp
FIND_PACKAGE(PkgConfig)
IF(PKG_CONFIG_FOUND)
  pkg_check_modules(JSONCPP jsoncpp)
  IF(JSONCPP_FOUND)
    ADD_DEFINITIONS(-DHAVE_JSONCPP)
    SET(HAVE_JSONCPP 1)
    SET(JSONCPP_DIR_MESSAGE "JsonCPP was found")
  ELSE(JSONCPP_FOUND)
    SET(JSONCPP_DIR_MESSAGE "JsonCPP was not found; the AtomSpace Publisher Module needs JsonCPP to publish events in JSON.\nIf you need to receive AtomSpace events in JSON, install JsonCPP using your package manager \nsuch as apt or build from source: https://github.com/open-source-parsers/jsoncpp/wiki/Building")
  ENDIF(JSONCPP_FOUND)
ENDIF(PKG_CONFIG_FOUND)
MESSAGE(STATUS "${JSONCPP_DIR_MESSAGE}")

# Threaded Building Blocks (Intel TBB) library
FIND_PACKAGE(TBB)
IF (TBB_FOUND)
	SET(HAVE_TBB 1)
ELSE (TBB_FOUND)
	SET(TBB_DIR_MESSAGE "Intel TBB (Threaded Building Blocks) was not found; the AtomSpace Publisher module will not be built. Installation instructions: https://www.threadingbuildingblocks.org/download")
ENDIF (TBB_FOUND)
MESSAGE(STATUS "${TBB_DIR_MESSAGE}")

# ZeroMQ
FIND_PACKAGE(ZMQ 3.2.4)
IF (ZMQ_FOUND AND ZMQ_LIBRARY)
	ADD_DEFINITIONS(-DHAVE_ZMQ)
	SET(HAVE_ZMQ 1)
	SET(ZMQ_DIR_MESSAGE "ZeroMQ was found.")
ELSE (ZMQ_FOUND AND ZMQ_LIBRARY)
	SET(ZMQ_DIR_MESSAGE "ZeroMQ was not found; the AtomSpace Publisher module, and the message system used by GUI monitor for OAC will not be built.\nTo over-ride, make sure that the environment variable ZMQ_LIBRARY is set.\nInstallation instructions: http://zeromq.org/intro:get-the-software")
ENDIF (ZMQ_FOUND AND ZMQ_LIBRARY)
MESSAGE(STATUS "${ZMQ_DIR_MESSAGE}")

# ===================================================================
# Global includes

# -------------------------------------------------
# Include configuration.

# Set default include paths.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${Boost_INCLUDE_DIRS}
	${COGUTIL_INCLUDE_DIR} ${ATOMSPACE_INCLUDE_DIR})

# Macros that define how atom types get declared.
IF (NOT DEFINED ATOMSPACE_DATA_DIR)
	SET (ATOMSPACE_DATA_DIR "${COGUTIL_DATA_DIR}")
ENDIF (NOT DEFINED ATOMSPACE_DATA_DIR)

INCLUDE("${ATOMSPACE_DATA_DIR}/cmake/OpenCogMacros.cmake")
INCLUDE("${ATOMSPACE_DATA_DIR}/cmake/OpenCogFunctions.cmake")

# -------------------------------------------------
# Library configuration

# Small hack to handle unixes that use "/usr/lib64" instead of
# "/usr/lib" as the default lib path on 64 bit archs.
IF (NOT DEFINED LIB_DIR_SUFFIX)
	EXECUTE_PROCESS(COMMAND ${CMAKE_CXX_COMPILER} -print-search-dirs OUTPUT_VARIABLE PRINT_SEARCH_DIRS_OUTPUT)
	STRING(REGEX MATCH "\r?\nlibraries:.*\r?\n" COMPILER_LIB_SEARCH_DIRS ${PRINT_SEARCH_DIRS_OUTPUT})
	IF (NOT ${COMPILER_LIB_SEARCH_DIRS} STREQUAL "")
		STRING(REGEX MATCH "/lib64/:|/lib64:|/lib64\n" HAS_LIB64 ${COMPILER_LIB_SEARCH_DIRS})
		IF (NOT ${HAS_LIB64} STREQUAL "")
			SET(LIB_DIR_SUFFIX "64")
		ENDIF (NOT ${HAS_LIB64} STREQUAL "")
	ENDIF (NOT ${COMPILER_LIB_SEARCH_DIRS} STREQUAL "")
ENDIF (NOT DEFINED LIB_DIR_SUFFIX)

# RPATH handling (see https://cmake.org/Wiki/CMake_RPATH_handling)
# Note: RPATH only supported under Linux!
SET(CMAKE_SKIP_BUILD_RPATH	FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH
	"${CMAKE_INSTALL_PREFIX}/lib/opencog"
	"${CMAKE_INSTALL_PREFIX}/lib/opencog/modules")

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# -------------------------------------------------
# Install configuration

# Only list install files that have actually changed.
SET(CMAKE_INSTALL_MESSAGE "LAZY")

# Set confdir and datadir
IF (NOT DEFINED CONFDIR)
	SET (CONFDIR "${CMAKE_INSTALL_PREFIX}/etc")
ENDIF (NOT DEFINED CONFDIR)

IF (NOT DEFINED DATADIR)
	SET (DATADIR "${CMAKE_INSTALL_PREFIX}/share/opencog")
ENDIF (NOT DEFINED DATADIR)

ADD_DEFINITIONS(-DCONFDIR="${CONFDIR}")
ADD_DEFINITIONS(-DDATADIR="${DATADIR}")

# (re?)define MAN_INSTALL_DIR
SET (MAN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/man")

# ==========================================================
# Decide what to build, based on the packages found.

IF(HAVE_ATOMSPACE AND HAVE_COGUTIL)
	SET(HAVE_ATTENTION 1)
	SET(HAVE_DIMEMBED 1)
	SET(HAVE_STATISTICS 1)
ENDIF(HAVE_ATOMSPACE AND HAVE_COGUTIL)

IF(HAVE_ATOMSPACE)
	SET(HAVE_SERVER 1)
ENDIF(HAVE_ATOMSPACE)

IF(HAVE_ATOMSPACE AND HAVE_GUILE AND HAVE_LINK_GRAMMAR AND HAVE_UUID)
	SET(HAVE_NLP 1)
ENDIF(HAVE_ATOMSPACE AND HAVE_GUILE AND HAVE_LINK_GRAMMAR AND HAVE_UUID)

IF(HAVE_SERVER AND HAVE_TBB AND HAVE_ZMQ AND HAVE_JSONCPP)
  SET(HAVE_EVENT_PUBLISHING_DEPENDENCIES 1)
ENDIF(HAVE_SERVER AND HAVE_TBB AND HAVE_ZMQ AND HAVE_JSONCPP)

ADD_SUBDIRECTORY(lib)
ADD_SUBDIRECTORY(opencog)

IF (CXXTEST_FOUND)
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
			COMMAND ${CMAKE_CTEST_COMMAND} --force-new-ctest-process --output-on-failure $(ARGS)

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
			COMMAND ${CMAKE_CTEST_COMMAND} --force-new-ctest-process --output-on-failure $(ARGS)
			COMMENT "Running tests..."
		)
	ENDIF (CMAKE_BUILD_TYPE STREQUAL "Coverage")
ENDIF (CXXTEST_FOUND)

ADD_SUBDIRECTORY(examples EXCLUDE_FROM_ALL)

IF (NOT WIN32)
	ADD_CUSTOM_TARGET (examples
		# using CMAKE_BUILD_TOOL results in teh cryptic error message
		# warning: jobserver unavailable: using -j1.  Add `+' to parent make rule.
		# This is because make doesn't know how to pass jobserver args to
		# the submake.  So, instead, just use $(MAKE) (with round parens)
		# -- that will do the right thing.
		# COMMAND ${CMAKE_BUILD_TOOL}
		COMMAND $(MAKE)
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

ADD_SUBDIRECTORY(experiments EXCLUDE_FROM_ALL)

IF (NOT WIN32)
	ADD_CUSTOM_TARGET (experiments
		# using CMAKE_BUILD_TOOL results in teh cryptic error message
		# warning: jobserver unavailable: using -j1.  Add `+' to parent make rule.
		# This is because make doesn't know how to pass jobserver args to
		# the submake.  So, instead, just use $(MAKE) (with round parens)
		# -- that will do the right thing.
		# COMMAND ${CMAKE_BUILD_TOOL}
		COMMAND $(MAKE)
		WORKING_DIRECTORY experiments
		COMMENT "Building experiments"
	)
ENDIF (NOT WIN32)

# ===================================================================
# Packaging
## Architecture the package is for.
## TODO: Will give error on non debian distros, fix it.
EXECUTE_PROCESS(COMMAND  dpkg --print-architecture
	OUTPUT_VARIABLE PACKAGE_ARCHITECTURE
	OUTPUT_STRIP_TRAILING_WHITESPACE)
STRING(TIMESTAMP UTC_DATE %Y%m%d UTC)
# If 'sudo make install' is run before 'make package', then install_manifest.txt
# will be owned by root. Creating the file during configuration stage ensures
# that it is owned by the builder thus avoiding 'Permission denied' error when
# packaging.
FILE(WRITE "${PROJECT_BINARY_DIR}/install_manifest.txt")
## It doesn't have a header-file declaring the version similar to cogutil and
## atomspace.
SET(SEMANTIC_VERSION 0.1.4)

## Cpack configuration
SET(CPACK_GENERATOR "DEB")
SET(CPACK_PACKAGE_CONTACT "opencog@googlegroups.com")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
SET(CPACK_PACKAGE_DIRECTORY "${CMAKE_BINARY_DIR}/packages")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Cognition Framework")
SET(CPACK_PACKAGE_NAME "opencog-dev")
SET(CPACK_PACKAGE_VENDOR "opencog.org")
SET(CPACK_PACKAGE_VERSION "${SEMANTIC_VERSION}-${UTC_DATE}")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
SET(CPACK_PACKAGE_FILE_NAME
	"${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${PACKAGE_ARCHITECTURE}")
SET(CPACK_PACKAGING_INSTALL_PREFIX "/usr/local")
SET(CPACK_PACKAGE_EXECUTABLES "cogserver" "The Open Cognition Framework")

## Debian specific configurations
SET(DEPENDENCY_LIST
	"guile-2.2-dev (>= 2.2.2)"
	"python-dev (>= 2.7.5)"
	"libboost-date-time-dev (>= ${MIN_BOOST})"
	"libboost-filesystem-dev (>= ${MIN_BOOST})"
	"libboost-program-options-dev (>= ${MIN_BOOST})"
	"libboost-regex-dev (>= ${MIN_BOOST})"
	"libboost-serialization-dev (>= ${MIN_BOOST})"
	"libboost-thread-dev (>= ${MIN_BOOST})"
	"libboost-system-dev (>= ${MIN_BOOST})"
	"libboost-random-dev (>= ${MIN_BOOST})"
	"libstdc++6 (>= 4.7)"
	"libcogutil-dev (>= 2.0.2)"
	"atomspace-dev (>= 5.0.3)"
)

STRING(REPLACE ";" ", " MAIN_DEPENDENCIES "${DEPENDENCY_LIST}")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "${MAIN_DEPENDENCIES}")
SET(CPACK_DEBIAN_PACKAGE_SECTION "libdevel")
SET(CPACK_DEBIAN_PACKAGE_HOMEPAGE "http://opencog.org")
INCLUDE(CPack)

# ===================================================================
# documentation
FIND_PACKAGE(Doxygen)
ADD_SUBDIRECTORY(doc EXCLUDE_FROM_ALL)

# ===================================================================
# Show a summary of what we found, what we will do.

SUMMARY_ADD("Attention" "Agents for attention allocation dynamics" HAVE_ATTENTION)
SUMMARY_ADD("Blopex" "BLOPEX block eigenvalue solver" HAVE_LAPACK)
SUMMARY_ADD("Cython bindings" "Cython (python) bindings" HAVE_CYTHON)
SUMMARY_ADD("Dim. Embedding" "Compute dimensional embedding of the AtomSpace" HAVE_DIMEMBED)
SUMMARY_ADD("Doxygen" "Code documentation" DOXYGEN_FOUND)
SUMMARY_ADD("GTK Visualizer" "GTK3-based Atomspace Visualizer" HAVE_GTK)
SUMMARY_ADD("Python tests" "Python bindings nose tests" HAVE_NOSETESTS)
SUMMARY_ADD("REST Events" "REST Atomspace Event Publisher module"
	HAVE_EVENT_PUBLISHING_DEPENDENCIES)
SUMMARY_ADD("Server" "The CogServer" HAVE_SERVER)
SUMMARY_ADD("SpaceTime" "3D Space-Time object tracker" HAVE_OCTOMAP)
SUMMARY_ADD("Unit tests" "Unit tests" CXXTEST_FOUND)
SUMMARY_ADD("NLP" "Natural Language Processing" HAVE_NLP)
SUMMARY_ADD("PatternMiner" "Pattern Miner" HAVE_ATOMSPACE)
SUMMARY_ADD("Haskell codes" "Logic via Lojban" HAVE_STACK AND BUILD_LOJBAN)

SUMMARY_SHOW()
