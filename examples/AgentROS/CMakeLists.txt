cmake_minimum_required(VERSION 2.8.3)
project(AgentROS)
#set(ROS_BUILD_SHARED_LIBS 1)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  #genmsg
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#//assuming atomspace and cogutil are there
#//find libraries to link to in opencog and Module.h path
#//need cogserver libserver.so 
#SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/lib/")
find_library(COGUTIL_LIBRARY
	NAMES cogutil
	PATH /usr/lib /usr/lib64 /usr/local/lib /usr/local/lib64)

INCLUDE (CheckIncludeFiles)

# The location of cogutil is variable. Find the right location.
find_path(COGUTIL_INCLUDE_DIR opencog/util/cogutil.h
	PATH /usr/include /usr/local/include
)

set(CMAKE_REQUIRED_INCLUDES ${COGUTIL_INCLUDE_DIR})
CHECK_INCLUDE_FILES (opencog/util/cogutil.h HAVE_COGUTIL_H)

if (COGUTIL_LIBRARY AND HAVE_COGUTIL_H)
	set(COGUTIL_FOUND TRUE)
	set(COGUTIL_VERSION 0)
	FILE(READ "${COGUTIL_INCLUDE_DIR}/opencog/util/cogutil.h" _CU_H_CONTENTS)
	STRING(REGEX MATCH
		"#define COGUTIL_VERSION_STRING[  ]+\"([0-9.]+)\""
		_MATCH "${_CU_H_CONTENTS}")
	SET(COGUTIL_VERSION ${CMAKE_MATCH_1})
endif (COGUTIL_LIBRARY AND HAVE_COGUTIL_H)

if ( COGUTIL_FOUND )
	message( STATUS "Found CogUtil version ${COGUTIL_VERSION} at ${COGUTIL_LIBRARY}")
else ( COGUTIL_FOUND )
	message( STATUS "CogUtil not found")
endif ( COGUTIL_FOUND )

set(ATOMSPACE_LIBPATH
   /usr/lib/opencog /usr/lib64/opencog /usr/local/lib/opencog /usr/local/lib64/opencog)

find_library(ATOMSPACE_LIBRARY atomspace
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_atomspaceutils_LIBRARY atomspaceutils
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_atomutils_LIBRARY atomutils
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_clearbox_LIBRARY clearbox
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_execution_LIBRARY execution
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_lambda_LIBRARY lambda
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_persist_LIBRARY persist
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_persist-sql_LIBRARY persist-sql
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_PythonEval_LIBRARY PythonEval
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_query_LIBRARY query
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_ruleengine_LIBRARY ruleengine
   PATH ${ATOMSPACE_LIBPATH})

find_library(ATOMSPACE_smob_LIBRARY smob
   PATH ${ATOMSPACE_LIBPATH})

set(ATOMSPACE_LIBRARIES
	${ATOMSPACE_LIBRARY}
	${ATOMSPACE_atomspaceutils_LIBRARY}
	${ATOMSPACE_atomutils_LIBRARY}
	${ATOMSPACE_clearbox_LIBRARY}
	${ATOMSPACE_execution_LIBRARY}
	${ATOMSPACE_lambda_LIBRARY}
	${ATOMSPACE_persist_LIBRARY}
	${ATOMSPACE_persist-sql_LIBRARY}
	${ATOMSPACE_query_LIBRARY}
	${ATOMSPACE_ruleengine_LIBRARY}
	${ATOMSPACE_smob_LIBRARY}
)

INCLUDE (CheckIncludeFiles)

find_path(ATOMSPACE_INCLUDE_DIR opencog/atomspace/version.h
	PATH /usr/include /usr/local/include
)

set(CMAKE_REQUIRED_INCLUDES ${ATOMSPACE_INCLUDE_DIR})
CHECK_INCLUDE_FILES (opencog/atomspace/version.h HAVE_ATOMSPACE_H)

if (ATOMSPACE_LIBRARY AND HAVE_ATOMSPACE_H)
	set(ATOMSPACE_FOUND TRUE)
	set(ATOMSPACE_VERSION 0)
	FILE(READ "${ATOMSPACE_INCLUDE_DIR}/opencog/atomspace/version.h" _CU_H_CONTENTS)
	STRING(REGEX MATCH
		"#define ATOMSPACE_VERSION_STRING[  ]+\"([0-9.]+)\""
		_MATCH "${_CU_H_CONTENTS}")
	SET(ATOMSPACE_VERSION ${CMAKE_MATCH_1})
endif (ATOMSPACE_LIBRARY AND HAVE_ATOMSPACE_H)

if ( ATOMSPACE_FOUND )
#	message(STATUS "Found CogUtil version ${ATOMSPACE_VERSION} at ${ATOMSPACE_LIBRARY}")
	message(STATUS "Found AtomSpace at ${ATOMSPACE_LIBRARY}")
else ( ATOMSPACE_FOUND )
	message(STATUS "AtomSpace not found")
endif ( ATOMSPACE_FOUND )
# Check for cogutils first, then check for atomspace
# Cogutils
#FIND_PACKAGE(CogUtil 2.0.1 REQUIRED)
IF (COGUTIL_FOUND)
	MESSAGE(STATUS "CogUtil found.")
	ADD_DEFINITIONS(-DHAVE_COGUTIL)
	SET(HAVE_COGUTIL 1)
ELSE (COGUTIL_FOUND)
	MESSAGE(FATAL_ERROR "CogUtil missing: it is needed for everything!")
ENDIF (COGUTIL_FOUND)

# AtomSpace
#FIND_PACKAGE(AtomSpace 5.0.3 REQUIRED)
IF (ATOMSPACE_FOUND)
	MESSAGE(STATUS "AtomSpace found.")
	ADD_DEFINITIONS(-DHAVE_ATOMSPACE)
	SET(HAVE_ATOMSPACE 1)
ELSE (ATOMSPACE_FOUND)
	MESSAGE(FATAL_ERROR "AtomSpace missing: it is needed for everything!")
ENDIF (ATOMSPACE_FOUND)

#manual setting
SET(OPENCOG_SRC_DIR "~/code/ocog/opencog")
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   string_atom.srv
#   Service2.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES AgentROS
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OPENCOG_SRC_DIR}
  include/${PROJECT_NAME}
  include
)

## Declare a cpp library
 add_library(AgentROS
   src/AgentROS.cc
   #src/${PROJECT_NAME}/AgentROS.cpp
 )

## Declare a cpp executable
# add_executable(AgentROS_node src/AgentROS_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(AgentROS_node AgentROS_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(AgentROS
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS AgentROS AgentROS_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_AgentROS.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
