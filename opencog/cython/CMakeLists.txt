
# Need to use -fno-strict-aliasing when compiling cython code, in order
# to avoid nasty compiler warnings about aliasing.  Cython explicitly
# performs aliasing, in order to emulate python object  inheritance.
# See, for example, https://groups.google.com/forum/#!topic/cython-users/JV1-KvIUeIg
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")

INCLUDE_DIRECTORIES(
	${PYTHON_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
)

# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt

IF (HAVE_ATOMSPACE)
	# How to Cython the .pyx file
	add_custom_command(OUTPUT opencog/atomspace.cpp
		COMMAND
		${CYTHON_EXECUTABLE} -o opencog/atomspace.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/opencog/atomspace.pyx"
		DEPENDS "opencog/atom.pyx" "opencog/classserver.pyx" "opencog/atomspace.pyx" "opencog/atomspace.pxd"  "opencog/atomspace_details.pyx" opencog_atom_types)
ENDIF (HAVE_ATOMSPACE)

IF (HAVE_SERVER)
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
ENDIF(HAVE_SERVER)

IF (HAVE_ATOMSPACE AND HAVE_GUILE)
	add_custom_command(OUTPUT scheme_wrapper.cpp
		COMMAND
		${CYTHON_EXECUTABLE} -f -o scheme_wrapper.cpp -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/scheme_wrapper.pyx"
		DEPENDS "${PROJECT_SOURCE_DIR}/opencog/cython/PyScheme.cc" "opencog/atomspace.pxd" opencog_atom_types)
	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "scheme_wrapper.cpp")
ENDIF (HAVE_ATOMSPACE AND HAVE_GUILE)


IF (HAVE_MOSES)
	add_custom_command(OUTPUT opencog/pymoses.cpp
		COMMAND
		${CYTHON_EXECUTABLE} -o opencog/pymoses.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/opencog/pymoses.pyx"
		DEPENDS "opencog/pymoses.pyx" "opencog/pymoses.pxd")

	# opencog.pymoses Python bindings
	ADD_LIBRARY(moses_cython SHARED
		opencog/pymoses.cpp
	)

	TARGET_LINK_LIBRARIES(moses_cython
		moses_exec
		moses
		cogutil
		comboreduct
		feature_selection
		${PYTHON_LIBRARIES}
		${Boost_PROGRAM_OPTIONS_LIBRARY}
		${Boost_THREAD_LIBRARY}
	)

	IF (HAVE_MPI)
		TARGET_LINK_LIBRARIES (moses_cython
			# These two work for both MPICH2 and OpenMPI, for me.
			${MPI_LIBRARY}
			${MPI_EXTRA_LIBRARY}
		)
	ENDIF (HAVE_MPI)

	SET_TARGET_PROPERTIES(moses_cython PROPERTIES
		PREFIX ""
		OUTPUT_NAME opencog/pymoses)

ENDIF (HAVE_MOSES)

# opencog.atomspace Python bindings
ADD_LIBRARY(atomspace_cython SHARED
	opencog/atomspace.cpp
)

TARGET_LINK_LIBRARIES(atomspace_cython
	atomspace
	cogutil
	${PYTHON_LIBRARIES}
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

TARGET_LINK_LIBRARIES(cogserver_cython
	server
	${PYTHON_LIBRARIES}
)

SET_TARGET_PROPERTIES(cogserver_cython PROPERTIES
	PREFIX ""
	OUTPUT_NAME opencog/cogserver)

## util
ADD_LIBRARY(util_cython SHARED
	opencog/util.cpp
)

TARGET_LINK_LIBRARIES(util_cython
	cogutil
	${PYTHON_LIBRARIES}
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
	${PYTHON_LIBRARIES}
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
# ADD_DEPENDENCIES(PythonEval agent_finder)

TARGET_LINK_LIBRARIES(PythonEval
	${PYTHON_LIBRARIES}
)

IF(HAVE_GUILE)
	ADD_LIBRARY(scheme_wrapper SHARED
		PyScheme
		scheme_wrapper.cpp
	)

	TARGET_LINK_LIBRARIES(scheme_wrapper
		scheme-shell
		smob
		${PYTHON_LIBRARIES}
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
ADD_DEPENDENCIES(PythonModule agent_finder)

TARGET_LINK_LIBRARIES(PythonModule
	PythonEval
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

