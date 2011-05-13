INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt

# How to Cython the .pyx file
add_custom_command(OUTPUT opencog/atomspace.cpp
  COMMAND
  ${CYTHON_EXECUTABLE} -o opencog/atomspace.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/opencog/atomspace.pyx"
  DEPENDS "opencog/atom.pyx" "opencog/classserver.pyx" "opencog/atomspace.pyx" "opencog/atomspace_details.pyx")

add_custom_command(OUTPUT opencog/cogserver.cpp
  COMMAND
  ${CYTHON_EXECUTABLE} -o opencog/cogserver.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/opencog/cogserver.pyx"
  DEPENDS "opencog/cogserver.pyx" "opencog/agent.pyx" "opencog/cogserver.pxd")

list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "opencog/cogserver.cpp" "opencog/atomspace.cpp")

add_custom_command(OUTPUT agent_finder.cpp agent_finder_api.h
  COMMAND
  ${CYTHON_EXECUTABLE} -f -o agent_finder.cpp -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/agent_finder.pyx"
  DEPENDS "agent_finder.pyx" "opencog/atomspace.pxd" "opencog/cogserver.pxd")
list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "agent_finder.cpp" "agent_finder_api.h")

# opencog.atomspace Python bindings
ADD_LIBRARY(atomspace_cython SHARED
    opencog/atomspace.cpp
)

TARGET_LINK_LIBRARIES(atomspace_cython
	${Boost_THREAD_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    atomspace
    util
)

SET_TARGET_PROPERTIES(atomspace_cython PROPERTIES
    PREFIX ""
    OUTPUT_NAME opencog/atomspace)

# opencog.cogserver Python bindings
ADD_LIBRARY(cogserver_cython SHARED
    opencog/cogserver.cpp
)

# cogserver just has the MindAgent template class at the moment
#TARGET_LINK_LIBRARIES(cogserver_cython
#)

SET_TARGET_PROPERTIES(cogserver_cython PROPERTIES
    PREFIX ""
    OUTPUT_NAME opencog/cogserver)

# direct file and path creation for stuff that doesn't need to be compiled
file(MAKE_DIRECTORY opencog)
# module init
file(COPY opencog/__init__.py DESTINATION opencog)
##

# utility library for PythonModule
ADD_LIBRARY(agent_finder SHARED
    agent_finder.cpp
)

TARGET_LINK_LIBRARIES(agent_finder
	${Boost_THREAD_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

ADD_DEPENDENCIES(agent_finder cogserver_cython atomspace_cython)

SET_TARGET_PROPERTIES(agent_finder PROPERTIES
    PREFIX ""
    OUTPUT_NAME agent_finder)
##

# Python module to be loaded by CogServer
ADD_LIBRARY (PythonModule SHARED
	PythonModule.cc 
    PyMindAgent.cc
)
ADD_DEPENDENCIES(PythonModule agent_finder)

TARGET_LINK_LIBRARIES(PythonModule
    server
    ${PYTHON_LIBRARIES}
)

IF (WIN32)
    INSTALL (TARGETS PythonModule DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
    INSTALL (TARGETS agent_finder DESTINATION "${DATADIR}/python")
    INSTALL (TARGETS atomspace_cython cogserver_cython DESTINATION "${DATADIR}/python/opencog")
ELSE (WIN32)
    INSTALL (TARGETS PythonModule LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
    INSTALL (TARGETS agent_finder LIBRARY DESTINATION "${DATADIR}/python")
    INSTALL (TARGETS atomspace_cython cogserver_cython LIBRARY DESTINATION "${DATADIR}/python/opencog")
ENDIF (WIN32)
INSTALL (FILES 
    opencog/__init__.py
    DESTINATION "${DATADIR}/python/opencog")

