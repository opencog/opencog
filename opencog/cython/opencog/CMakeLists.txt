
# Need to use -fno-strict-aliasing when compiling cython code, in order
# to avoid nasty compiler warnings about aliasing.  Cython explicitly
# performs aliasing, in order to emulate python object inheritance.
# See, for example,
# https://groups.google.com/forum/#!topic/cython-users/JV1-KvIUeIg
#
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")

INCLUDE_DIRECTORIES(
	${ATOMSPACE_INCLUDE_DIR}
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
	ENDIF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${name}.pxd)
	# Allow the user to specify dependencies as optional arguments
	SET(DEPENDS ${DEPENDS} ${ARGN})
	ADD_CUSTOM_COMMAND(
		OUTPUT ${name}.cpp
		COMMAND ${CYTHON_EXECUTABLE}
		ARGS ${CYTHON_FLAGS} -I ${PROJECT_BINARY_DIR}
		      -I ${ATOMSPACE_INCLUDE_DIR}/opencog/cython
		      # -I /usr/local/include/opencog/cython
		      -o ${name}.cpp
		      --cplus ${CMAKE_CURRENT_SOURCE_DIR}/${name}.pyx
		DEPENDS ${DEPENDS}
		COMMENT "Cythonizing ${name}.pyx")
ENDMACRO(CYTHON_ADD_MODULE_PYX)


# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt

IF (HAVE_ATOMSPACE)
	############################## type constructors #####################
	CYTHON_ADD_MODULE_PYX(cogserver_type_constructors)

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "cogserver_type_constructors.cpp")

	ADD_LIBRARY(cogserver_type_constructors SHARED
		cogserver_type_constructors.cpp
	)

	# XXX FIXME Cython should not static link the atom-types, but
	# but should instead dynamic-load them, as needed.
	ADD_DEPENDENCIES(cogserver_type_constructors nlp_atom_types)
	ADD_DEPENDENCIES(cogserver_type_constructors attention_atom_types)
	ADD_DEPENDENCIES(cogserver_type_constructors spacetime_atom_types)
	# ADD_DEPENDENCIES(cogserver_type_constructors pln_atom_types)
	ADD_DEPENDENCIES(cogserver_type_constructors embodiment_atom_types)
	ADD_DEPENDENCIES(cogserver_type_constructors spacetime_atom_types)

	TARGET_LINK_LIBRARIES(cogserver_type_constructors
		attention-types
		embodiment-types
		nlp-types
		spacetime-types
		${ATOMSPACE_LIBRARIES}
		${PYTHON_LIBRARIES}
	)

	SET_TARGET_PROPERTIES(cogserver_type_constructors PROPERTIES
		PREFIX ""
		OUTPUT_NAME cogserver_type_constructors)

	INSTALL (TARGETS cogserver_type_constructors
		DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/opencog")

	IF (HAVE_OCTOMAP)

		############################ spacetime #########################
		CYTHON_ADD_MODULE_PYX(spacetime
			"${ATOMSPACE_INCLUDE_DIR}/opencog/cython/opencog/atomspace.pxd"
		)
		list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "spacetime.cpp")
		ADD_LIBRARY(spacetime_cython SHARED
			spacetime.cpp
		)

		TARGET_LINK_LIBRARIES(spacetime_cython
			${CMAKE_INSTALL_PREFIX}/lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/opencog/atomspace.so
			SpaceMap
			spacetime
			${ATOMSPACE_LIBRARIES}
			${COGUTIL_LIBRARY}
			${PYTHON_LIBRARIES}
		)

		SET_TARGET_PROPERTIES(spacetime_cython PROPERTIES
			PREFIX ""
			OUTPUT_NAME spacetime)

		INSTALL (TARGETS spacetime_cython
			DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/opencog")

		############################ spatial #########################
		CYTHON_ADD_MODULE_PYX(spatial
			"${ATOMSPACE_INCLUDE_DIR}/opencog/cython/opencog/atomspace.pxd"
			"../../spacetime/SpaceServer.h" "../../spacetime/TimeServer.h"
		)
		list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "spatial.cpp")
		ADD_LIBRARY(spatial_cython SHARED
			spatial.cpp
		)

		TARGET_LINK_LIBRARIES(spatial_cython
			${CMAKE_INSTALL_PREFIX}/lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/opencog/atomspace.so
			SpaceMap
			SpaceMapUtil
			spacetime
			${ATOMSPACE_LIBRARIES}
			${COGUTIL_LIBRARY}
			${PYTHON_LIBRARIES}
		)

		SET_TARGET_PROPERTIES(spatial_cython PROPERTIES
			PREFIX ""
			OUTPUT_NAME spatial)

		INSTALL (TARGETS spatial_cython
			DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/opencog")
	ENDIF (HAVE_OCTOMAP)
ENDIF (HAVE_ATOMSPACE)


IF (HAVE_SERVER)
	############################ cogserver #########################
	CYTHON_ADD_MODULE_PYX(cogserver
		"${ATOMSPACE_INCLUDE_DIR}/opencog/cython/opencog/atomspace.pxd"
		"request.pyx" "agent.pyx" "agent_finder_types.h"
		"../../cogserver/server/Agent.h" "../../cogserver/server/Request.h"
		"../../spacetime/SpaceServer.h" "../../spacetime/TimeServer.h"
	)

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "cogserver.cpp")

	# opencog.cogserver Python bindings
	ADD_LIBRARY(cogserver_cython SHARED
		cogserver.cpp
	)

	TARGET_LINK_LIBRARIES(cogserver_cython
		server
		${ATOMSPACE_LIBRARIES}
		${COGUTIL_LIBRARY}
		${PYTHON_LIBRARIES}
	)

	SET_TARGET_PROPERTIES(cogserver_cython PROPERTIES
		PREFIX ""
		OUTPUT_NAME cogserver)

	INSTALL (TARGETS cogserver_cython
		DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/opencog")

	####################### agent finder ########################
	CYTHON_ADD_MODULE_PYX(agent_finder
		"${ATOMSPACE_INCLUDE_DIR}/opencog/cython/opencog/atomspace.pxd"
		"cogserver.pxd"
		"../../cogserver/server/Agent.h" "../../cogserver/server/Request.h"
		"../../spacetime/SpaceServer.h" "../../spacetime/TimeServer.h"
	)

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES
		"agent_finder.cpp" "agent_finder_api.h")

	# utility library for PythonModule
	ADD_LIBRARY(agent_finder SHARED
		agent_finder.cpp
	)

	TARGET_LINK_LIBRARIES(agent_finder
		${ATOMSPACE_LIBRARIES}
		${PYTHON_LIBRARIES}
		${Boost_THREAD_LIBRARY}
		${Boost_SYSTEM_LIBRARY}
	)

	SET_TARGET_PROPERTIES(agent_finder PROPERTIES
		PREFIX ""
		OUTPUT_NAME agent_finder)

	INSTALL (TARGETS agent_finder
		DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/opencog")
ENDIF(HAVE_SERVER)

IF (HAVE_STATISTICS)
	CYTHON_ADD_MODULE_PYX(statistics
		"../../learning/statistics/DataProvider.h"
		"../../learning/statistics/Probability.h"
		"../../learning/statistics/Entropy.h"
		"../../learning/statistics/InteractionInformation.h"
	)

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "statistics.cpp")

	# opencog.statistics Python bindings
	ADD_LIBRARY(statistics_cython SHARED
		statistics.cpp
	)

	TARGET_LINK_LIBRARIES(statistics_cython
		statistics
		${PYTHON_LIBRARIES}
	)

	SET_TARGET_PROPERTIES(statistics_cython PROPERTIES
		PREFIX ""
		OUTPUT_NAME statistics)

	INSTALL (TARGETS statistics_cython
		DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/opencog")
ENDIF (HAVE_STATISTICS)

INSTALL (FILES
	__init__.py
	DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/opencog")
