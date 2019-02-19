#
# Need to use -fno-strict-aliasing when compiling cython code, in order
# to avoid nasty compiler warnings about aliasing.  Cython explicitly
# performs aliasing, in order to emulate python object inheritance.
# See, for example, https://groups.google.com/forum/#!topic/cython-users/JV1-KvIUeIg
#
# Also suppress unused function warnings for static functions, since
# Cython generates a static function that it puts in a header which
# causes this warning to appear whenever anyone includes the header
# and doesn't use all the functions.
#
SET(CMAKE_CXX_FLAGS
	"${CMAKE_CXX_FLAGS} -fno-strict-aliasing -Wno-unused-function")

ADD_SUBDIRECTORY (opencog)

INCLUDE_DIRECTORIES(
	${PYTHON_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
)

# module init
file(MAKE_DIRECTORY opencog)
# file(COPY opencog/__init__.py DESTINATION opencog)
##
