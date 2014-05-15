
# Need to use -fno-strict-aliasing when compiling cython code, in order
# to avoid nasty compiler warnings about aliasing.  Cython explicitly
# performs aliasing, in order to emulate python object inheritance.
# See, for example, 
# https://groups.google.com/forum/#!topic/cython-users/JV1-KvIUeIg
#
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")

INCLUDE_DIRECTORIES(
	${PYTHON_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
)

SET(CYTHON_FLAGS "-f")

# Cythonizes one .pyx file into a .cpp file
# Additional arguments are dependencies
MACRO(CYTHON_ADD_MODULE_PYX name)
	IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${name}.pxd)
		SET(DEPENDS ${name}.pyx ${name}.pxd)
	ELSE(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${name}.pxd)
		SET(DEPENDS ${name}.pyx)
	ENDIF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${name}.pxd)
	# Allow the user to specify dependencies as optional arguments
	SET(DEPENDS ${DEPENDS} ${ARGN})
	ADD_CUSTOM_COMMAND(
		OUTPUT ${name}.cpp
		COMMAND ${CYTHON_EXECUTABLE}
		ARGS ${CYTHON_FLAGS} -o ${name}.cpp --cplus ${CMAKE_CURRENT_SOURCE_DIR}/${name}.pyx
		DEPENDS ${DEPENDS}
		COMMENT "Cythonizing ${name}.pyx")
ENDMACRO(CYTHON_ADD_MODULE_PYX)

# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt

IF (HAVE_ATOMSPACE)
	###################### logger ##########################################
	CYTHON_ADD_MODULE_PYX(logger)
	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "logger.cpp" 
		"../../util/Logger.h")

	ADD_LIBRARY(logger_cython SHARED
		logger.cpp
	)

	TARGET_LINK_LIBRARIES(logger_cython
		cogutil
		${PYTHON_LIBRARIES}
	)

	SET_TARGET_PROPERTIES(logger_cython PROPERTIES
		PREFIX ""
		OUTPUT_NAME logger)

	###################### atomspace ####################################
	CYTHON_ADD_MODULE_PYX(atomspace
		"atom.pyx" "classserver.pyx" "atomspace_details.pyx"
		"spacetime.pyx" opencog_atom_types
		"../../atomspace/TruthValue.h" "../../atomspace/SimpleTruthValue.h"
		"../../atomspace/ClassServer.h" "../../atomspace/Handle.h"
		"../../atomspace/AtomSpace.h"
		"../../spacetime/SpaceServer.h" "../../spacetime/TimeServer.h"
	)

	# list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "atomspace.cpp")
	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "atomspace_api.h")

	# opencog.atomspace Python bindings
	ADD_LIBRARY(atomspace_cython SHARED
		atomspace.cpp
	)

	# XXX FIXME Cython should not static link the atom-types, but
	# but should instead dynamic-load them, as needed.
	ADD_DEPENDENCIES(atomspace_cython opencog_atom_types)
	ADD_DEPENDENCIES(atomspace_cython nlp_atom_types)
	ADD_DEPENDENCIES(atomspace_cython attention_atom_types)
	ADD_DEPENDENCIES(atomspace_cython spacetime_atom_types)
	ADD_DEPENDENCIES(atomspace_cython pln_atom_types)
	ADD_DEPENDENCIES(atomspace_cython embodiment_atom_types)

	TARGET_LINK_LIBRARIES(atomspace_cython
		atomspace
		attention-types
		embodiment-types
		nlp-types
		PLNTypes
		cogutil
		${PYTHON_LIBRARIES}
		${Boost_THREAD_LIBRARY}
		${Boost_SYSTEM_LIBRARY}
	)

	SET_TARGET_PROPERTIES(atomspace_cython PROPERTIES
		PREFIX ""
		OUTPUT_NAME atomspace)

ENDIF (HAVE_ATOMSPACE)

IF (HAVE_ATOMSPACE AND HAVE_GUILE)
	############################## scheme wrapper #####################
	CYTHON_ADD_MODULE_PYX(scheme_wrapper
		"PyScheme.h" "atomspace.pxd" opencog_atom_types
		"../../guile/load-file.h" "../../query/PatternSCM.h")

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "scheme_wrapper.cpp")

	ADD_LIBRARY(scheme_wrapper SHARED
		PyScheme
		scheme_wrapper.cpp
	)

	TARGET_LINK_LIBRARIES(scheme_wrapper
		query
		smob
		${PYTHON_LIBRARIES}
	)
	
	SET_TARGET_PROPERTIES(scheme_wrapper PROPERTIES
		PREFIX ""
		OUTPUT_NAME scheme_wrapper)

ENDIF (HAVE_ATOMSPACE AND HAVE_GUILE)


IF (HAVE_SERVER)
	############################ cogserver #########################
	CYTHON_ADD_MODULE_PYX(cogserver
		"request.pyx" "agent.pyx" "atomspace.pxd" opencog_atom_types
		"agent_finder_types.h"
		"../../atomspace/TruthValue.h" "../../atomspace/SimpleTruthValue.h"
		"../../atomspace/ClassServer.h" "../../atomspace/Handle.h"
		"../../atomspace/AtomSpace.h"
		"../../spacetime/SpaceServer.h" "../../spacetime/TimeServer.h"
	)

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "cogserver.cpp")

	# opencog.cogserver Python bindings
	ADD_LIBRARY(cogserver_cython SHARED
		cogserver.cpp
	)

	TARGET_LINK_LIBRARIES(cogserver_cython
		server
		${PYTHON_LIBRARIES}
	)

	SET_TARGET_PROPERTIES(cogserver_cython PROPERTIES
		PREFIX ""
		OUTPUT_NAME cogserver)

	####################### agent finder ########################
	CYTHON_ADD_MODULE_PYX(agent_finder
		"atomspace.pxd" "cogserver.pxd"
		"../../server/Agent.h" "../../server/Request.h"
		"../../atomspace/TruthValue.h" "../../atomspace/SimpleTruthValue.h"
		"../../atomspace/ClassServer.h" "../../atomspace/Handle.h"
		"../../atomspace/AtomSpace.h"
		"../../spacetime/SpaceServer.h" "../../spacetime/TimeServer.h"
	)

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES
		"agent_finder.cpp" "agent_finder_api.h")

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

ENDIF(HAVE_SERVER)

IF (HAVE_MOSES)
	CYTHON_ADD_MODULE_PYX(pymoses
		"../../learning/moses/main/moses_exec.h"
	)

	# opencog.pymoses Python bindings
	ADD_LIBRARY(moses_cython SHARED
		pymoses.cpp
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
		OUTPUT_NAME pymoses)

ENDIF (HAVE_MOSES)


IF (WIN32)
	INSTALL (TARGETS agent_finder DESTINATION "${DATADIR}/python")
	INSTALL (TARGETS atomspace_cython cogserver_cython DESTINATION "${DATADIR}/python/opencog")
ELSE (WIN32)
	INSTALL (TARGETS agent_finder LIBRARY DESTINATION "${DATADIR}/python")
	INSTALL (TARGETS atomspace_cython cogserver_cython LIBRARY DESTINATION "${DATADIR}/python/opencog")
ENDIF (WIN32)

INSTALL (FILES 
	__init__.py
	DESTINATION "${DATADIR}/python/opencog")

