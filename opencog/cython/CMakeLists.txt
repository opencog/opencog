INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt

# How to Cython the .pyx file
add_custom_command(OUTPUT opencog/pymoses.cpp
  COMMAND
  ${CYTHON_EXECUTABLE} -o opencog/pymoses.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/opencog/pymoses.pyx"
  DEPENDS "opencog/pymoses.pyx" "opencog/pymoses.pxd")

add_custom_command(OUTPUT opencog/atomspace.cpp
  COMMAND
  ${CYTHON_EXECUTABLE} -o opencog/atomspace.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/opencog/atomspace.pyx"
  DEPENDS "opencog/atom.pyx" "opencog/classserver.pyx" "opencog/atomspace.pyx" "opencog/atomspace.pxd"  "opencog/atomspace_details.pyx" opencog_atom_types)

add_custom_command(OUTPUT opencog/cogserver.cpp
  COMMAND
  ${CYTHON_EXECUTABLE} -o opencog/cogserver.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/opencog/cogserver.pyx"
  DEPENDS "opencog/cogserver.pyx" "opencog/request.pyx" "opencog/agent.pyx" "opencog/atomspace.pxd" "opencog/cogserver.pxd" opencog_atom_types)

add_custom_command(OUTPUT opencog/util.cpp
  COMMAND
  ${CYTHON_EXECUTABLE} -o opencog/util.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/opencog/util.pyx"
  DEPENDS "opencog/util.pyx" "opencog/atomspace.pxd" opencog_atom_types)

list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "opencog/cogserver.cpp" "opencog/atomspace.cpp"
    "opencog/util.cpp")

add_custom_command(OUTPUT agent_finder.cpp agent_finder_api.h
  COMMAND
  ${CYTHON_EXECUTABLE} -f -o agent_finder.cpp -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/agent_finder.pyx"
  DEPENDS "agent_finder.pyx" "opencog/atomspace.pxd" "opencog/cogserver.pxd")
list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "agent_finder.cpp" "agent_finder_api.h")

add_custom_command(OUTPUT logic_wrapper.cpp logic_wrapper_api.h
  COMMAND
  ${CYTHON_EXECUTABLE} -f -o logic_wrapper.cpp -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/logic_wrapper.pyx"
  DEPENDS "opencog/atomspace.pxd" opencog_atom_types)
list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "logic_wrapper.cpp" "logic_wrapper_api.h")

add_custom_command(OUTPUT scheme_wrapper.cpp
  COMMAND
  ${CYTHON_EXECUTABLE} -f -o scheme_wrapper.cpp -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/scheme_wrapper.pyx"
  DEPENDS "${PROJECT_SOURCE_DIR}/opencog/guile/SchemeEval.cc" "opencog/atomspace.pxd" opencog_atom_types)
list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "scheme_wrapper.cpp")

# opencog.moses Python bindings
ADD_LIBRARY(pymoses_cython SHARED
    opencog/pymoses.cpp
)

TARGET_LINK_LIBRARIES(pymoses_cython
        moses
        cogutil
        ${Boost_THREAD_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
)

SET_TARGET_PROPERTIES(pymoses_cython PROPERTIES
    PREFIX ""
    OUTPUT_NAME opencog/pymoses)

# opencog.atomspace Python bindings
ADD_LIBRARY(atomspace_cython SHARED
    opencog/atomspace.cpp
)

TARGET_LINK_LIBRARIES(atomspace_cython
	atomspace
	cogutil
	${Boost_THREAD_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
)

SET_TARGET_PROPERTIES(atomspace_cython PROPERTIES
    PREFIX ""
    OUTPUT_NAME opencog/atomspace)

# opencog.cogserver Python bindings
ADD_LIBRARY(cogserver_cython SHARED
    opencog/cogserver.cpp
)

ADD_DEPENDENCIES(atomspace_cython opencog_atom_types)

# cogserver just has the MindAgent template class at the moment
#TARGET_LINK_LIBRARIES(cogserver_cython
#)

SET_TARGET_PROPERTIES(cogserver_cython PROPERTIES
    PREFIX ""
    OUTPUT_NAME opencog/cogserver)

## util
ADD_LIBRARY(util_cython SHARED
    opencog/util.cpp
)

TARGET_LINK_LIBRARIES(util_cython
    #${Boost_THREAD_LIBRARY}
    #${Boost_SYSTEM_LIBRARY}
    #atomspace
    cogutil
)

SET_TARGET_PROPERTIES(util_cython PROPERTIES
    PREFIX ""
    OUTPUT_NAME opencog/util)


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

ADD_LIBRARY(PythonEval SHARED
    PythonEval.cc
)
ADD_DEPENDENCIES(PythonEval agent_finder)

TARGET_LINK_LIBRARIES(PythonEval
    server
    ${PYTHON_LIBRARIES}
)

# utility library for PythonModule
ADD_LIBRARY(logic_wrapper SHARED
    logic_wrapper.cpp
)

TARGET_LINK_LIBRARIES(logic_wrapper
    atomspace
)

ADD_DEPENDENCIES(logic_wrapper atomspace_cython)

SET_TARGET_PROPERTIES(logic_wrapper PROPERTIES
    PREFIX ""
    OUTPUT_NAME logic_wrapper)

IF(HAVE_GUILE)
	ADD_LIBRARY(scheme_wrapper SHARED
		scheme_wrapper.cpp
	)

	TARGET_LINK_LIBRARIES(scheme_wrapper
		scheme-shell smob
	)
	
	ADD_DEPENDENCIES(scheme_wrapper scheme-shell)

	SET_TARGET_PROPERTIES(scheme_wrapper PROPERTIES
		PREFIX ""
		OUTPUT_NAME scheme_wrapper)
ENDIF(HAVE_GUILE)

# Python module to be loaded by CogServer
ADD_LIBRARY (PythonModule SHARED
	PythonModule.cc 
    PyMindAgent.cc
    PyRequest.cc
)
#ADD_DEPENDENCIES(PythonModule agent_finder PythonEval logic_wrapper)
ADD_DEPENDENCIES(PythonModule agent_finder)

SET(PYTHONMODULE_LINK_LIBS server PythonEval ${PYTHON_LIBRARIES})

TARGET_LINK_LIBRARIES(PythonModule
    ${PYTHONMODULE_LINK_LIBS}
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

