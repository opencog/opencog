
ADD_LIBRARY (attentionbonk SHARED
	AFImplicator.cc
	AtomBins.cc
	AttentionalFocusCB.cc
	AttentionBank.cc
	AttentionSCM.cc
	AVUtils.cc
	ImportanceIndex.cc
	StochasticImportanceDiffusion.cc
)

TARGET_LINK_LIBRARIES(attentionbonk
	attentionval
	query-engine
	execution
	smob
	atombase
	truthvalue
	${COGUTIL_LIBRARY}
)

ADD_GUILE_EXTENSION(SCM_CONFIG attentionbonk "opencog-ext-path-attentionbonk")

IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
    ADD_DEFINITIONS(-DECAN_EXPERIMENT)
ENDIF(CMAKE_BUILD_TYPE MATCHES DEBUG)

INSTALL (TARGETS attentionbonk EXPORT AtomSpaceTargets
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	AFImplicator.h
	AtomBins.h
	AttentionBank.h
	AVUtils.h
	ImportanceIndex.h
	StochasticImportanceDiffusion.h
	DESTINATION "include/opencog/attentionbank/bank"
)
