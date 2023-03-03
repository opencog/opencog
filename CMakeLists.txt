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
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

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

# Add the 'cmake' directory from cogutil to search path
LIST(APPEND CMAKE_MODULE_PATH ${COGUTIL_DATA_DIR}/cmake)
INCLUDE(OpenCogGccOptions)
INCLUDE(OpenCogLibOptions)
INCLUDE(OpenCogInstallOptions)
INCLUDE(Summary)

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

# CogServer
FIND_PACKAGE(CogServer 1.0.0 CONFIG)
IF (COGSERVER_FOUND)
	MESSAGE(STATUS "CogServer found.")
	ADD_DEFINITIONS(-DHAVE_SERVER)
	SET(HAVE_SERVER 1)
ELSE (COGSERVER_FOUND)
	MESSAGE(STATUS "CogServer was not found.")
ENDIF (COGSERVER_FOUND)

# AttentionBank
FIND_PACKAGE(AttentionBank 1.0.0 CONFIG)
IF (ATTENTIONBANK_FOUND)
	MESSAGE(STATUS "AttentionBank found.")
	ADD_DEFINITIONS(-DHAVE_BANK)
	SET(HAVE_BANK 1)
ELSE (ATTENTIONBANK_FOUND)
	MESSAGE(STATUS "AttentionBank was not found. OpenPsi and Ghost will not be built.")
ENDIF (ATTENTIONBANK_FOUND)

# URE
FIND_PACKAGE(URE 1.0.0 CONFIG)
IF (URE_FOUND)
	MESSAGE(STATUS "URE found.")
	ADD_DEFINITIONS(-DHAVE_URE)
	SET(HAVE_URE 1)
ELSE (URE_FOUND)
	MESSAGE(STATUS "URE was not found. OpenPsi will not be built.")
ENDIF (URE_FOUND)

# PLN
FIND_PACKAGE(PLN CONFIG)
IF (PLN_FOUND)
	MESSAGE(STATUS "PLN found.")
	ADD_DEFINITIONS(-DHAVE_PLN)
	SET(HAVE_PLN 1)
ELSE (PLN_FOUND)
	MESSAGE(STATUS "PLN was not found. Ghost will not be built.")
ENDIF (PLN_FOUND)

# LGAtomese
FIND_PACKAGE(LGAtomese 1.0.0 CONFIG)
IF (LGATOMESE_FOUND)
	MESSAGE(STATUS "LGAtomese found.")
	ADD_DEFINITIONS(-DHAVE_NLP)
	SET(HAVE_NLP 1)
ELSE (LGATOMESE_FOUND)
	MESSAGE(STATUS "LGAtomese was not found. Ghost will not be built.")
ENDIF (LGATOMESE_FOUND)

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
# This is required for Guile, Python and Cython

include(OpenCogFindGuile)
include(OpenCogFindPython)

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

# ===================================================================
# Include configuration.

# Set default include paths.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}
	${COGUTIL_INCLUDE_DIR} ${ATOMSPACE_INCLUDE_DIR})

# Macros that define how atom types get declared.
IF (NOT DEFINED ATOMSPACE_DATA_DIR)
	SET (ATOMSPACE_DATA_DIR "${COGUTIL_DATA_DIR}")
ENDIF (NOT DEFINED ATOMSPACE_DATA_DIR)

# Add the 'cmake' directory from atomspace to search path (should be
# the same as cogutil, but we add it just in case)
LIST(APPEND CMAKE_MODULE_PATH ${ATOMSPACE_DATA_DIR}/cmake)
INCLUDE(OpenCogMacros)
INCLUDE(OpenCogGuile)
INCLUDE(OpenCogCython)

# ==========================================================
# Decide what to build, based on the packages found.

IF(HAVE_ATOMSPACE AND HAVE_GUILE AND HAVE_BANK AND HAVE_URE)
	SET(HAVE_OPENPSI 1)
ENDIF(HAVE_ATOMSPACE AND HAVE_GUILE AND HAVE_BANK AND HAVE_URE)

IF(HAVE_BANK AND HAVE_OPENPSI AND HAVE_NLP AND HAVE_PLN)
	SET(HAVE_GHOST 1)
ENDIF(HAVE_BANK AND HAVE_OPENPSI AND HAVE_NLP AND HAVE_PLN)

ADD_SUBDIRECTORY(lib)
ADD_SUBDIRECTORY(opencog)

IF (CXXTEST_FOUND)
	ADD_CUSTOM_TARGET(tests)
	ADD_SUBDIRECTORY(tests EXCLUDE_FROM_ALL)
	IF (CMAKE_BUILD_TYPE STREQUAL "Coverage")
		# doing coverage stuff while running tests if this is the Coverage build
		ADD_CUSTOM_TARGET(check
			# TODO lcov should be found by cmake first
			# TODO set it up so that we can pick to run coverage per test, or
			# combined across all tests (the latter is MUCH faster). Use a define?
			# There is coverage specific stuff in AddCxxTest.cmake now...
			# -
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
		ADD_CUSTOM_TARGET(check
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
	"python3-dev (>= 3.6.7)"
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

SUMMARY_ADD("Cython bindings" "Cython (python) bindings" HAVE_CYTHON)
SUMMARY_ADD("Doxygen" "Code documentation" DOXYGEN_FOUND)
SUMMARY_ADD("Unit tests" "Unit tests" CXXTEST_FOUND)
SUMMARY_ADD("OpenPsi" "MicroPsi OpenCog implementation" HAVE_OPENPSI)
SUMMARY_ADD("Ghost" "Ghost chatbot" HAVE_GHOST)
SUMMARY_ADD("Haskell codes" "Logic via Lojban" HAVE_STACK AND BUILD_LOJBAN)

SUMMARY_SHOW()
