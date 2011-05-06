INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
)

# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt

ADD_CUSTOM_TARGET(cython_bindings DEPENDS
    atomspace.pyx
    agent.pyx
    opencog.pyx
    classserver.pyx
    atom.py
)

ADD_CUSTOM_TARGET(agent_finder DEPENDS
    cython_bindings
    agent_finder.pyx
)

ADD_LIBRARY (PythonModule SHARED
	PythonModule.cc 
    PyMindAgent.cc
)

ADD_DEPENDENCIES(PythonModule cython_bindings agent_finder opencog_atom_types)

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
