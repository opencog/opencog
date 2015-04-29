#
# Rule engine lib build
#
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
	InferenceSCM.cc
	Rule.cc
	JsonicControlPolicyParamLoader.cc
)

ADD_DEPENDENCIES(ruleengine
	opencog_atom_types
)

TARGET_LINK_LIBRARIES(ruleengine
	${ATOMSPACE_LIBRARIES}
)

IF (WIN32)
   INSTALL (TARGETS reasoning DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
   INSTALL (TARGETS reasoning DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
	forwardchainer/ForwardChainer.h
	forwardchainer/ForwardChainInputMatchCB.h
	forwardchainer/ForwardChainPatternMatchCB.h
	backwardchainer/BCPatternMatch.h
	DESTINATION "include/${PROJECT_NAME}/reasoning/engine"
)
