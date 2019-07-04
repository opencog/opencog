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

##################### Attention Bank ##################

CYTHON_ADD_MODULE_PYX(bank
    "bank.pyx"
)

list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "bank.cpp")

ADD_LIBRARY(bank_cython SHARED
    bank.cpp
)

TARGET_LINK_LIBRARIES(bank_cython
	attentionbonk
	${ATOMSPACE_LIBRARIES}
	${COGUTIL_LIBRARY}
	${PYTHON_LIBRARIES}
)


SET_TARGET_PROPERTIES(bank_cython PROPERTIES
    PREFIX ""
	OUTPUT_NAME bank)
	
add_custom_command(TARGET bank_cython POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:bank_cython> "${PROJECT_BINARY_DIR}/opencog/cython/opencog/bank.so"
)

INSTALL (TARGETS bank_cython
    DESTINATION "${PYTHON_DEST}")