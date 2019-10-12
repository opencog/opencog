
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

##################### NLP Types ##################

CYTHON_ADD_MODULE_PYX(nlp_types
	"../../nlp/types/nlp_types.pyx"
)

list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "nlp_types.cpp")

# opencog.nlp_types Python bindings
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

# Below is commented out because for some reason it does not
# work any more, but I can't figure out why.
#ADD_LIBRARY(nlp_types_cython SHARED
#	${CMAKE_BINARY_DIR}/opencog/nlp/types/nlp_types.cpp
#)

#TARGET_LINK_LIBRARIES(nlp_types_cython
#	nlp_types
#	${PYTHON_LIBRARIES}
#)

#SET_TARGET_PROPERTIES(nlp_types_cython PROPERTIES
#	PREFIX ""
#	OUTPUT_NAME nlp_types)

#INSTALL (TARGETS nlp_types_cython
#	DESTINATION "${PYTHON_DEST}")

##################### OpenPsi ##################

IF (HAVE_OPENPSI)
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
ENDIF (HAVE_OPENPSI)
