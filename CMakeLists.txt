project (OPENCOG)

IF(CMAKE_COMPILER_IS_GNUCXX)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector -Wall")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

#SET(CMAKE_BUILD_TYPE Debug)
#SET(CMAKE_BUILD_TYPE Release)

add_subdirectory (src)
add_subdirectory (test)
