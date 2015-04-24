#rule engine lib build
ADD_LIBRARY(ruleengine SHARED
	backwardchainer/BackwardChainer.cc
	RECommons.cc
	forwardchainer/ForwardChainer.cc
	backwardchainer/BCPatternMatch.cc
	forwardchainer/ForwardChainInputMatchCB.cc
	forwardchainer/ForwardChainPatternMatchCB.cc
	forwardchainer/ForwardChainerCallBack.h
	forwardchainer/DefaultForwardChainerCB.cc
	forwardchainer/FCMemory.cc	
	RuleEngineModule.cc
	InferenceSCM.cc
	Rule.cc
	JsonicControlPolicyParamLoader.cc
)

ADD_DEPENDENCIES(ruleengine
	opencog_atom_types
)

TARGET_LINK_LIBRARIES(ruleengine
	lambda
	query
	smob
	execution
	atomutils
	atomspace
)

# Build the RuelEngine cogserver module
ADD_LIBRARY(RuleEngineModule SHARED
	RuleEngineModule.cc
)

TARGET_LINK_LIBRARIES(RuleEngineModule ruleengine server)

IF (WIN32)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
   INSTALL (TARGETS query DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
	forwardchainer/ForwardChainer.h
	forwardchainer/ForwardChainInputMatchCB.h
	forwardchainer/ForwardChainPatternMatchCB.h
	backwardchainer/BCPatternMatch.h
	DESTINATION "include/${PROJECT_NAME}/reasoning/engine"
)

