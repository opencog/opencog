
# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (smob SHARED
	SchemeEval.cc
	SchemeExec.cc
	SchemePrimitive.cc
	SchemeSmob.cc
	SchemeSmobAtom.cc
	SchemeSmobAV.cc
	SchemeSmobGC.cc
	SchemeSmobNew.cc
	SchemeSmobTV.cc
	SchemeSmobVH.cc
	load-file
)

ADD_DEPENDENCIES(smob opencog_atom_types)

TARGET_LINK_LIBRARIES(smob
	${GUILE_LIBRARIES}
)

ADD_EXECUTABLE(PrimitiveExample
	PrimitiveExample.cc
)
TARGET_LINK_LIBRARIES(PrimitiveExample
	smob
	atomspace
	server
)

IF (WIN32)
    INSTALL (TARGETS smob DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
    INSTALL (TARGETS smob LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
	SchemeEval.h
	SchemePrimitive.h
	SchemeSmob.h
	load-file.h
	DESTINATION "include/${PROJECT_NAME}/guile"
)
