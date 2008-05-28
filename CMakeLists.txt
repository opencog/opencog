PROJECT(OPENCOG)
SET(CMAKE_BUILD_TYPE Debug) #Release)

# add the 'vendor' dir to cmake's module search path
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/lib/")

IF(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector -Wall")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# check dependencies
FIND_PACKAGE(EXPAT REQUIRED)
FIND_PACKAGE(CSockets REQUIRED)
FIND_PACKAGE(OpenSSL REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(IODBC)
FIND_PACKAGE(Guile)

# global includes
INCLUDE_DIRECTORIES (${Boost_INCLUDE_DIRS})

# recurse into 'src' and 'tests' dirs
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests EXCLUDE_FROM_ALL)

# add 'tests' target (to _compile_ the tests) and the 'test'
# target (to _run_ the tests)
ADD_CUSTOM_TARGET(tests
	COMMAND make
	WORKING_DIRECTORY tests
)
ADD_CUSTOM_TARGET(test
	DEPENDS tests
	WORKING_DIRECTORY tests
	COMMAND $(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running tests..."
	COMMAND /usr/bin/ctest --force-new-ctest-process $(ARGS)
)
