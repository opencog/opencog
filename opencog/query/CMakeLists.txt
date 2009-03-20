INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/util
)

ADD_LIBRARY(query SHARED
	PatternMatch.cc
	PatternMatchEngine.cc
)

ADD_DEPENDENCIES(query opencog_atom_types)

INSTALL (TARGETS query
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	DefaultPatternMatchCB.h
	OutgoingTree.h
	PatternMatch.h
	PatternMatchCallback.h
	PatternMatchEngine.h
	SimpleLogicPMCB.h
	DESTINATION "include/opencog/query"
)
