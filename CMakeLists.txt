CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
IF (COMMAND CMAKE_POLICY)
	CMAKE_POLICY(SET CMP0003 NEW)
	CMAKE_POLICY(SET CMP0005 OLD)
ENDIF (COMMAND CMAKE_POLICY)

PROJECT(OPENCOG)

################################
#uncomment to be in Release mode
################################
#SET(CMAKE_BUILD_TYPE Release)

IF (CMAKE_BUILD_TYPE STREQUAL "")
	SET(CMAKE_BUILD_TYPE Debug)
ENDIF (CMAKE_BUILD_TYPE STREQUAL "")

# add the 'lib' dir to cmake's module search path
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/lib/")

IF (CMAKE_COMPILER_IS_GNUCXX)
  IF (APPLE) 
    SET(CMAKE_C_FLAGS "-Wall -Wno-long-long -Wno-conversion")
    SET(CMAKE_C_FLAGS_DEBUG "-O0 -g")
    SET(CMAKE_C_FLAGS_RELEASE "-O2 -g0")
    # Vital to do this otherwise unresolved symbals everywhere:
    SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,-flat_namespace,-undefined,dynamic_lookup")
  ELSE (APPLE)
    SET(CMAKE_C_FLAGS "-Wall -fPIC")
    # Linas: Change debug flags to -O2 since I think that 1) -O0 offers
    # no debugging advantage that I'm aware of, and 2) -g does not hurt
    # performance in any way that I'm aware of.
    SET(CMAKE_C_FLAGS_DEBUG "-g -fstack-protector")
    SET(CMAKE_C_FLAGS_RELEASE "-O2 -g -fstack-protector")
  ENDIF (APPLE)
  
  # Nil: added -Wno-deprecated to avoid multiple warnings
  # due to hash_map which is deprecated in gcc 4.3
  # note that when replacing hash_map by unordered_map
  # it results in other warnings explaining that C++0x support is still
  # experimental, which is why I let hash_map for now
  # Nil: added -Wno-variadic-macros to avoid warnings regarding
  # using variadic in macro OC_ASSERT (the warning warns that this is
  # only available from C99, lol!)
  SET(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wno-deprecated -Wno-variadic-macros")

  SET(CMAKE_CXX_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})
  SET(CMAKE_CXX_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})
ENDIF (CMAKE_COMPILER_IS_GNUCXX)

# check dependencies
IF (WIN32)
	FIND_PACKAGE(PThreads REQUIRED)
	FIND_PACKAGE(STLPort REQUIRED)
	INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/win32" ${PTHREADS_INCLUDE_DIR} ${STLPORT_INCLUDE_DIR})
	LINK_LIBRARIES(${PTHREADS_LIBRARY} ${STLPORT_LIBRARIES})
	ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE)
ELSE (WIN32)
	LINK_LIBRARIES(pthread)
	IF (NOT APPLE)
	    SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--enable-new-dtags")
	ENDIF (NOT APPLE)
ENDIF (WIN32)

# hypertable requires boost iostreams
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE(Boost 1.37 COMPONENTS date_time filesystem iostreams regex serialization signals system thread REQUIRED)
FIND_PACKAGE(Boost 1.37 COMPONENTS python program_options)

IF(Boost_PROGRAM_OPTIONS_FOUND)
  MESSAGE(STATUS "Found Boost::program_options")
ELSE(Boost_PROGRAM_OPTIONS_FOUND)
  MESSAGE(STATUS "Boost program_options missing: needed for Spatial tools (MapTools).")  
ENDIF(Boost_PROGRAM_OPTIONS_FOUND)

FIND_PACKAGE(OpenGL)

IF(OPENGL_FOUND)
  MESSAGE(STATUS "OpenGL was found.")
ELSE(OPENGL_FOUND)
  MESSAGE(STATUS "OpenGL missing: needed for Spatial tools (MapExplorer, MapTools, etc.).")  
ENDIF(OPENGL_FOUND)

FIND_PACKAGE(SDL)

IF(SDL_FOUND)
  MESSAGE(STATUS "SDL was found.")
ELSE(SDL_FOUND)
  MESSAGE(STATUS "SDL missing: needed for Spatial tools (MapExplorer, MapTools, etc.).")  
ENDIF(SDL_FOUND)

FIND_PACKAGE(SDL_gfx)

IF(SDLGFX_FOUND)
  MESSAGE(STATUS "SDL_gfx was found")
ELSE(SDLGFX_FOUND)
  MESSAGE(STATUS "SDL_gfx missing: needed for Spatial tools (MapExplorer, MapTools, etc.).")
ENDIF(SDLGFX_FOUND)


# At this time, there is no reason to build with memcached for 
# FIND_PACKAGE(LibMemCached)

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
	SET(ODBC_FOUND 1)
	SET(ODBC_INCLUDE_DIRS ${UnixODBC_INCLUDE_DIRS})
	SET(ODBC_LIBRARIES ${UnixODBC_LIBRARIES})
	SET(ODBC_DIR_MESSAGE "UnixODBC was found.")
ELSE (UnixODBC_FOUND)
	SET(ODBC_FOUND 0)
	SET(ODBC_DIR_MESSAGE "UnixODBC was not found; persistence subsystem will not be built.\nTo over-ride, make sure UnixODBC_LIBRARIES and UnixODBC_INCLUDE_DIRS are set.")
ENDIF (UnixODBC_FOUND)
MESSAGE(STATUS "${ODBC_DIR_MESSAGE}")


# Gnu Guile scheme interpreter
# Versions prior to 1.8.6 have assorted serious problems with
# multi-threading.  If you need guile, you need this version.
FIND_PACKAGE(Guile 1.8.6)
IF (GUILE_FOUND)
	ADD_DEFINITIONS(-DHAVE_GUILE)
	SET(GUILE_DIR_MESSAGE "Guile was found.")
ELSE (GUILE_FOUND)
	SET(GUILE_DIR_MESSAGE "Guile was not found; the scheme shell will not be built.\nTo over-ride, make sure GUILE_LIBRARIES and GUILE_INCLUDE_DIRS are set.")
ENDIF (GUILE_FOUND)
MESSAGE(STATUS "${GUILE_DIR_MESSAGE}")


# Gnu Scientific Library
FIND_PACKAGE(GSL REQUIRED)
IF (GSL_FOUND)
	ADD_DEFINITIONS(-DHAVE_GSL)
ENDIF (GSL_FOUND)

# XML parsing library
FIND_PACKAGE(EXPAT)
IF (EXPAT_FOUND)
	ADD_DEFINITIONS(-DHAVE_EXPAT)
ELSE (EXPAT_FOUND)
	MESSAGE(STATUS "Expat XML parser missing: needed for hypertable support.")
ENDIF (EXPAT_FOUND)

# These are required for building the embodiment code
FIND_PACKAGE(XercesC)
IF (XERCESC_FOUND)
	INCLUDE_DIRECTORIES(${XERCESC_INCLUDE_DIR})
ELSE (XERCESC_FOUND)
	MESSAGE(STATUS "XercesC missing: needed for embodiment code.")
ENDIF (XERCESC_FOUND)

FIND_PACKAGE(Lua51)
IF (NOT LUA51_FOUND)
	MESSAGE(STATUS "Lua missing: needed for embodiment code.")
ENDIF (NOT LUA51_FOUND)

FIND_PACKAGE(Luabind)
IF (NOT LUABIND_FOUND)
	MESSAGE(STATUS "Luabind missing: needed for embodiment code.")
ENDIF (NOT LUABIND_FOUND)

# Not used at this time, its incompatible with the current code.
#
# FIND_PACKAGE(LinkGrammar)
# IF (LINK_GRAMMAR_FOUND)
# 	ADD_DEFINITIONS(-DHAVE_LINK_GRAMMAR)
# 	ADD_DEFINITIONS(-DLINK_GRAMMAR_DATA_DIR=\\"${LINK_GRAMMAR_DATA_DIR}/\\")
# ENDIF (LINK_GRAMMAR_FOUND)

# XML-RPC, used by the ubigraph module
FIND_PACKAGE(XMLRPC COMPONENTS client)
IF (NOT XMLRPC_FOUND)
    MESSAGE(STATUS "libxmlrpc missing: needed for ubigraph module.")
ENDIF (NOT XMLRPC_FOUND)
FIND_PACKAGE(CURL)
IF (NOT CURL_FOUND)
    MESSAGE(STATUS "libcurl missing: needed for ubigraph module.")
ENDIF (NOT CURL_FOUND)
IF (CURL_FOUND)
    IF (XMLRPC_FOUND)
        ADD_DEFINITIONS(-DHAVE_UBIGRAPH)
    ENDIF (XMLRPC_FOUND)
ENDIF (CURL_FOUND)

# At this time, there is no reason to build with memcached for
# any "normal" use, so just stub it out.
# IF (LIBMEMCACHED_FOUND)
#	ADD_DEFINITIONS(-DHAVE_LIBMEMCACHED)
# ENDIF (LIBMEMCACHED_FOUND)

# Hypertable library
FIND_PACKAGE(HyperTable)
IF (HYPERTABLE_FOUND)
	ADD_DEFINITIONS(-DHAVE_HYPERTABLE)
	MESSAGE(STATUS "Hypertable was found.")

	# Don't bother looking for SIGAR unlesss hyperable was found.
	FIND_PACKAGE(SIGAR)
	IF (SIGAR_FOUND)
	ELSE (SIGAR_FOUND)
		MESSAGE(STATUS "SIGAR was not founds! It is needed for Hypertable support.")
	ENDIF (SIGAR_FOUND)

ELSE (HYPERTABLE_FOUND)
	MESSAGE(STATUS "Hypertable was not found; hypertable support will not be built.\nTo over-ride, make sure HYPERTABLE_LIBRARIES and HYPERTABLE_INCLUDE_DIRS are set.")
ENDIF (HYPERTABLE_FOUND)

# Needed for testing
FIND_PACKAGE(PythonInterp)
IF (Boost_PYTHON_FOUND)
  # find python destination dir for python bindings
  FIND_PACKAGE(PythonLibs)

  IF (PYTHONLIBS_FOUND)
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
        MESSAGE(STATUS "Found Boost::Python and Python, we will build Python bindings")
    ENDIF ("${PYTHON_DEST}" STREQUAL "")
  ELSE(PYTHONLIBS_FOUND)
    MESSAGE(STATUS "Python not found, we won't build Python bindings." )
  ENDIF(PYTHONLIBS_FOUND)
ELSE (Boost_PYTHON_FOUND)
  MESSAGE(STATUS "Boost Python libraries were not found, we won't build Python bindings.")
ENDIF (Boost_PYTHON_FOUND)

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
INCLUDE_DIRECTORIES(${OPENCOG_SOURCE_DIR} ${Boost_INCLUDE_DIRS})

# rpath handling
SET(CMAKE_SKIP_BUILD_RPATH	FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/opencog")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

ADD_SUBDIRECTORY(opencog)
ADD_SUBDIRECTORY(lib)

ADD_CUSTOM_TARGET(tests)
ADD_SUBDIRECTORY(tests EXCLUDE_FROM_ALL)
ADD_CUSTOM_TARGET(test
	DEPENDS tests
	WORKING_DIRECTORY tests
	COMMAND ${CMAKE_CTEST_COMMAND} --force-new-ctest-process $(ARGS)
	COMMENT "Running tests..."
)

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
		WORKING_DIRECTORY ${OPENCOG_SOURCE_DIR}
		COMMAND bzr nick
		OUTPUT_VARIABLE BZR_NICK OUTPUT_STRIP_TRAILING_WHITESPACE)
	EXECUTE_PROCESS(
		WORKING_DIRECTORY ${OPENCOG_SOURCE_DIR}
		COMMAND bzr revno
		OUTPUT_VARIABLE BZR_REVNO OUTPUT_STRIP_TRAILING_WHITESPACE)
	CONFIGURE_FILE(
		${OPENCOG_SOURCE_DIR}/doc/doxygen.conf.in 
		${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	ADD_CUSTOM_TARGET ( 
		doxygen
		COMMAND ${DOXYGEN} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
		COMMENT "Generating documentation"
	) 
ELSE (DOXYGEN_FOUND)
	MESSAGE(STATUS "WARNING: Doxygen not found, API documentation will not be generated")
ENDIF (DOXYGEN_FOUND)
