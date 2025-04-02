
# Need to use -fno-strict-aliasing when compiling cython code, in order
# to avoid nasty compiler warnings about aliasing.  Cython explicitly
# performs aliasing, in order to emulate python object inheritance.
# See, for example,
# https://groups.google.com/forum/#!topic/cython-users/JV1-KvIUeIg
#
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")

INCLUDE_DIRECTORIES(
	${ATOMSPACE_INCLUDE_DIR}
	${Python3_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
)

SET(CYTHON_FLAGS "-3" "-f" "-Wextra" # "-Werror"
	"-I" "${ATOMSPACE_INCLUDE_DIR}/opencog/cython"
	"-I" "${ATOMSPACE_INCLUDE_DIR}/opencog/cython/opencog")

# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt

##################### NLP Types ##################

CYTHON_ADD_MODULE_PYX(nlp_oc_types
	"nlp_oc_types.pyx"
)

list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "nlp_oc_types.cpp")

# opencog.nlp_oc_types Python bindings
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY(nlp_oc_types_cython SHARED
	nlp_oc_types.cpp
)

ADD_DEPENDENCIES(nlp_oc_types_cython nlp-oc-types)

TARGET_LINK_LIBRARIES(nlp_oc_types_cython
	nlp-oc-types
	${ATOMSPACE_LIBRARIES}
	${Python3_LIBRARIES}
)

SET_TARGET_PROPERTIES(nlp_oc_types_cython PROPERTIES
	PREFIX ""
	OUTPUT_NAME nlp_oc_types)

INSTALL (TARGETS nlp_oc_types_cython
	DESTINATION "${PYTHON_DEST}")

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
		${Python3_LIBRARIES}
	)

	SET_TARGET_PROPERTIES(openpsi_cython PROPERTIES
	    PREFIX ""
	    OUTPUT_NAME openpsi)

	INSTALL (TARGETS openpsi_cython
	    DESTINATION "${PYTHON_DEST}")
ENDIF (HAVE_OPENPSI)
