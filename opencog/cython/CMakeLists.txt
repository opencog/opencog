INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt

# How to Cython the .pyx file
add_custom_command(OUTPUT opencog.cpp
  COMMAND
  ${CYTHON_EXECUTABLE} -o opencog.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/opencog.pyx")
#add_custom_command(OUTPUT atomspace.cpp
#COMMAND
#${CYTHON_EXECUTABLE} -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/atomspace.pyx")
#add_custom_command(OUTPUT agent.cpp
#COMMAND
#${CYTHON_EXECUTABLE} -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/agent.pyx")
#add_custom_command(OUTPUT classserver.cpp
#COMMAND
#${CYTHON_EXECUTABLE} -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/classserver.pyx")
add_custom_command(OUTPUT atom.cpp
    COMMAND
    ${CYTHON_EXECUTABLE} -o atom.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/atom.py")
list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "opencog.cpp" "atom.cpp") # "atomspace.cpp" "classserver.cpp" "agent.cpp")

# How to Cython the .pyx file
add_custom_command(OUTPUT agent_finder.cpp agent_finder_api.h
  COMMAND
  ${CYTHON_EXECUTABLE} -f -o agent_finder.cpp -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/agent_finder.pyx")
list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "agent_finder.cpp" "agent_finder_api.h")

# opencog Python bindings
ADD_LIBRARY(opencogcy SHARED
    opencog.cpp
    atom.cpp
)
# These dependencies are are all included within the opencog.pyx file
ADD_DEPENDENCIES(opencogcy atomspace.pyx agent.pyx classserver.pyx)

TARGET_LINK_LIBRARIES(opencogcy
    atomspace
    util
)

SET_TARGET_PROPERTIES(opencogcy PROPERTIES
    PREFIX ""
    OUTPUT_NAME opencog)
##

# utility library for PythonModule
ADD_LIBRARY(agent_finder SHARED
    agent_finder.cpp
)

ADD_DEPENDENCIES(agent_finder agent_finder_api.h)

SET_TARGET_PROPERTIES(agent_finder PROPERTIES
    PREFIX ""
    OUTPUT_NAME agent_finder)
##

# Python module to be loaded by CogServer
ADD_LIBRARY (PythonModule SHARED
	PythonModule.cc 
    PyMindAgent.cc
)

ADD_DEPENDENCIES(PythonModule 
    cython_bindings agent_finder opencog_atom_types)

TARGET_LINK_LIBRARIES(PythonModule
    server
    #python2.6_d
    ${PYTHON_LIBRARIES}
)

IF (WIN32)
    INSTALL (TARGETS PythonModule DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
    INSTALL (TARGETS PythonModule LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)
