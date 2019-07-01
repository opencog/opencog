
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

SET(CYTHON_FLAGS "-2" "-f" "-I" "${ATOMSPACE_INCLUDE_DIR}/opencog/cython")

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

	TARGET_LINK_LIBRARIES(cogserver_type_constructors
		attention-types
		nlp-types
		spacetime-types
		${ATOMSPACE_LIBRARIES}
		${PYTHON_LIBRARIES}
	)

	SET_TARGET_PROPERTIES(cogserver_type_constructors PROPERTIES
		PREFIX ""
		OUTPUT_NAME cogserver_type_constructors)

	INSTALL (TARGETS cogserver_type_constructors
		DESTINATION "${PYTHON_DEST}")

ENDIF (HAVE_ATOMSPACE)


IF (HAVE_SERVER)
	############################ cogserver #########################
	CYTHON_ADD_MODULE_PYX(cogserver
		"${ATOMSPACE_INCLUDE_DIR}/opencog/cython/opencog/atomspace.pxd"
		"request.pyx" "agent.pyx" "agent_finder_types.h"
		"../../cogserver/server/Agent.h" "../../cogserver/server/Request.h"
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
		DESTINATION "${PYTHON_DEST}")

	####################### agent finder ########################
	CYTHON_ADD_MODULE_PYX(agent_finder
		"${ATOMSPACE_INCLUDE_DIR}/opencog/cython/opencog/atomspace.pxd"
		"cogserver.pxd"
		"../../cogserver/server/Agent.h" "../../cogserver/server/Request.h"
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
		DESTINATION "${PYTHON_DEST}")
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
		DESTINATION "${PYTHON_DEST}")
ENDIF (HAVE_STATISTICS)

##################### OpenPsi ##################

CYTHON_ADD_MODULE_PYX(openpsi
    "openpsi.pyx"
    "../../openpsi/OpenPsiSCM.h"
    openpsi
)

list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "openpsi.cpp")

# opencog.openpsi Python bindings
ADD_LIBRARY(openpsi_cython SHARED
    openpsi.cpp
)

TARGET_LINK_LIBRARIES(openpsi_cython
    openpsi
    ${PYTHON_LIBRARIES}
)

SET_TARGET_PROPERTIES(openpsi_cython PROPERTIES
    PREFIX ""
    OUTPUT_NAME openpsi)

INSTALL (TARGETS openpsi_cython
    DESTINATION "${PYTHON_DEST}")
