#INCLUDE_DIRECTORIES(
#	${PROJECT_SOURCE_DIR}/src/atomspace
#	${PROJECT_SOURCE_DIR}/src/util
#)

# XXX FIXME TODO Until such time that the learning-server messages are
# removed from the spacemap, we have to build static, to avoid circular
# dependencies. That's just plain wrong.
# ADD_LIBRARY(query SHARED
ADD_LIBRARY(query STATIC
	DefaultPatternMatchCB.cc
	PatternMatch.cc
	PatternMatchEngine.cc
	QueryModule.cc
)

ADD_DEPENDENCIES(query
	opencog_atom_types
)

TARGET_LINK_LIBRARIES(query
	server
	space_server
)
IF (HAVE_GUILE)
	TARGET_LINK_LIBRARIES(query smob)
ENDIF (HAVE_GUILE)

IF (WIN32)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
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
