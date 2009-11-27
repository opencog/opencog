#INCLUDE_DIRECTORIES(
#	${OPENCOG_SOURCE_DIR}/src/atomspace
#	${OPENCOG_SOURCE_DIR}/src/util
#)

ADD_LIBRARY(query SHARED
	DefaultPatternMatchCB.cc
	PatternMatch.cc
	PatternMatchEngine.cc
	QueryModule.cc
)

ADD_DEPENDENCIES(query opencog_atom_types)

INSTALL (TARGETS query
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	CrispLogicPMCB.h
	DefaultPatternMatchCB.h
	OutgoingTree.h
	PatternMatch.h
	PatternMatchCallback.h
	PatternMatchEngine.h
	QueryModule.h
	DESTINATION "include/opencog/query"
)
