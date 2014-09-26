#rule engine lib build
ADD_LIBRARY(ruleengine SHARED
pln/ForwardChainer.cc
pln/Chainer.cc
pln/ForwardChainInputMatchCB.cc
pln/ForwardChainPatternMatchCB.cc
RuleEngineModule.cc
InferenceSCM.cc
)

ADD_DEPENDENCIES(ruleengine
	opencog_atom_types
)

TARGET_LINK_LIBRARIES(ruleengine 
	query
	atomspace 
	execution 
	smob
)

# Build the RuelEngine cogserver module
ADD_LIBRARY(RuleEngineModule SHARED
	RuleEngineModule.cc
)
TARGET_LINK_LIBRARIES(RuleEngineModule ruleengine  server)

IF (WIN32)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
pln/ForwardChainer.h
pln/Chainer.h
pln/ForwardChainInputMatchCB.h
pln/ForwardChainPatternMatchCB.h
DESTINATION "include/${PROJECT_NAME}/reasoning/RuleEngine/rule-engine-src"
)

