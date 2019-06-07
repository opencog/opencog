#
# Need to use -fno-strict-aliasing when compiling cython code, in order
# to avoid nasty compiler warnings about aliasing.  Cython explicitly
# performs aliasing, in order to emulate python object inheritance.
# See, for example,
# https://groups.google.com/forum/#!topic/cython-users/JV1-KvIUeIg
#
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")

MESSAGE(STATUS "Setting python RPATH to ${PYTHON_DEST}")
SET(CMAKE_SHARED_LINKER_FLAGS
	"${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath -Wl,${PYTHON_DEST}")

INCLUDE_DIRECTORIES(
	${PYTHON_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
)

SET(CYTHON_FLAGS "-f")

# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt

###################### bank ##########################################
# list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "bank.cpp")

CYTHON_ADD_MODULE_PYX(bank
	# "../../../cython/opencog/atomspace.pxd"
	"bank.pyx"
)

ADD_LIBRARY(bank_cython
	bank.cpp
)

TARGET_LINK_LIBRARIES(bank_cython
	${NO_AS_NEEDED}
	attentionbonk
	${ATOMSPACE_LIBRARIES}
	${COGUTIL_LIBRARY}
	${PYTHON_LIBRARIES}
)

SET_TARGET_PROPERTIES(bank_cython PROPERTIES
	PREFIX ""
	OUTPUT_NAME bank)

# The cogserver needs the atomspace pxd file to get the Handle, Atom and
# AtomSpace defintions during compilation. So this must be installed.
INSTALL (FILES
	# __init__.py
	bank.pxd
	DESTINATION "include/opencog/cython/opencog"
)
