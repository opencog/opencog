
# Build the query shlib
ADD_LIBRARY(query SHARED
	DefaultPatternMatchCB.cc
	Implicator.cc
	Instantiator.cc
	PatternMatch.cc
	PatternMatchEngine.cc
	PatternSCM.cc
	PatternUtils.cc
	VirtualMatch.cc
	AttentionalFocusCB.cc
)

ADD_DEPENDENCIES(query
	opencog_atom_types
)

TARGET_LINK_LIBRARIES(query 
	atomspace 
	execution 
)

IF (HAVE_GUILE)
	TARGET_LINK_LIBRARIES(query 
		smob
	)
ENDIF (HAVE_GUILE)


# Build the query cogserver module
ADD_LIBRARY(QueryModule SHARED
	QueryModule.cc
)
TARGET_LINK_LIBRARIES(QueryModule query server)

IF (WIN32)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
	Implicator.h
	Instantiator.h
	PatternMatch.h
	PatternMatchCallback.h
	DESTINATION "include/${PROJECT_NAME}/query"
)
