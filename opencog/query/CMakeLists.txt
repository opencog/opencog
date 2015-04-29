
# Build the query shlib
ADD_LIBRARY(query SHARED
	AttentionalFocusCB.cc
	Composition.cc
	DefaultPatternMatchCB.cc
	Implicator.cc
	Instantiator.cc
	PatternMatch.cc
	PatternMatchEngine.cc
	PatternSCM.cc
	Satisfier.cc
	FuzzyMatch/FuzzyPatternMatch.cc
	FuzzyMatch/FuzzyPatternMatchCB.cc
	FuzzyMatch/GraphEditDist.cc
)

ADD_DEPENDENCIES(query
	opencog_atom_types
)

TARGET_LINK_LIBRARIES(query
	atomutils
	lambda
	atomspace
	execution
)

IF (HAVE_GUILE)
	TARGET_LINK_LIBRARIES(query
		smob
	)
ENDIF (HAVE_GUILE)

IF (WIN32)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
	Implicator.h
	Instantiator.h
	Pattern.h
	PatternMatchCallback.h
	PatternSCM.h
	Satisfier.h
	DESTINATION "include/${PROJECT_NAME}/query"
)
