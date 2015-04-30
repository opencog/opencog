
# ----------------------------------------
# Python module to be loaded by CogServer

INCLUDE_DIRECTORIES(
   ${PYTHON_INCLUDE_DIRS}
   ${CMAKE_CURRENT_SOURCE_DIR}
   ${CMAKE_CURRENT_BINARY_DIR}
)

ADD_LIBRARY (PythonModule SHARED
	PythonModule.cc 
	PyMindAgent.cc
	PyRequest.cc
)
TARGET_LINK_LIBRARIES(PythonModule
	PythonEval
	server
	${PYTHON_LIBRARIES}
)
ADD_DEPENDENCIES(PythonModule agent_finder)

IF (WIN32)
	INSTALL (TARGETS PythonModule DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)
