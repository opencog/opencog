#INCLUDE_DIRECTORIES(
#	${PROJECT_SOURCE_DIR}/src/atomspace
#	${PROJECT_SOURCE_DIR}/src/util
#)

ADD_LIBRARY(query SHARED
	DefaultPatternMatchCB.cc
	PatternMatch.cc
	PatternMatchEngine.cc
	QueryModule.cc
)

ADD_DEPENDENCIES(query
	opencog_atom_types
)

IF (HAVE_GUILE)
	TARGET_LINK_LIBRARIES(query
		smob
		server
	)
ELSE (HAVE_GUILE)
	TARGET_LINK_LIBRARIES(query
		server
	)
ENDIF (HAVE_GUILE)

IF (WIN32)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
   INSTALL (TARGETS query LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
	CrispLogicPMCB.h
	DefaultPatternMatchCB.h
	OutgoingTree.h
	PatternMatch.h
	PatternMatchCallback.h
	PatternMatchEngine.h
	QueryModule.h
	DESTINATION "include/${PROJECT_NAME}/query"
)
