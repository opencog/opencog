# Build the query module
INCLUDE_DIRECTORIES(
	${PYTHON_INCLUDE_PATH}
)

ADD_LIBRARY(query SHARED
	DefaultPatternMatchCB.cc
	PatternMatch.cc
	PatternMatchEngine.cc
	QueryModule.cc
)

ADD_DEPENDENCIES(query
	opencog_atom_types
)

TARGET_LINK_LIBRARIES(query atomspace execution)

IF (WIN32)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

# Why are we installing all of these? Not all are for public consumption;
# some are internal-use-only header files...
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
