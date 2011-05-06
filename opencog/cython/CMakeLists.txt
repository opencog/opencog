INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
)

ADD_LIBRARY (PythonModule SHARED
	PythonModule.cc 
    PyMindAgent.cc
)

ADD_DEPENDENCIES(PythonModule opencog_atom_types)

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
