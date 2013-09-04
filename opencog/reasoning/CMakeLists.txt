IF(CMAKE_COMPILER_IS_GNUCXX)

   # XXX Setting -ansi will blow up the compile, as it over-rides the 
   # -std=gnu++0x flag, which then breaks the mt19937 rand generator.
   # Did you really need -ansi to make this work ??
   # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -ansi -pedantic -Wall")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -pedantic -Wall")

ENDIF(CMAKE_COMPILER_IS_GNUCXX)
IF(HAVE_PLN)
    ADD_SUBDIRECTORY (pln)
ENDIF(HAVE_PLN)
