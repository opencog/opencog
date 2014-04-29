
# Need to use -fno-strict-aliasing when compiling cython code, in order
# to avoid nasty compiler warnings about aliasing.  Cython explicitly
# performs aliasing, in order to emulate python object  inheritance.
# See, for example, https://groups.google.com/forum/#!topic/cython-users/JV1-KvIUeIg
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")

ADD_SUBDIRECTORY (opencog)

INCLUDE_DIRECTORIES(
	${PYTHON_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
)

# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt



# direct file and path creation for stuff that doesn't need to be compiled
file(MAKE_DIRECTORY opencog)
# module init
file(COPY opencog/__init__.py DESTINATION opencog)
##


ADD_LIBRARY(PythonEval SHARED
	PythonEval.cc
)
ADD_DEPENDENCIES(PythonEval atomspace_cython)

TARGET_LINK_LIBRARIES(PythonEval
	${PYTHON_LIBRARIES}
)


# Python module to be loaded by CogServer
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
INSTALL (FILES 
	opencog/__init__.py
	DESTINATION "${DATADIR}/python/opencog")

