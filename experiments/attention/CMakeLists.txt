INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY(attentionexperiment SHARED
	ExperimentSetupModule
	SentenceGenStimulateAgent
)

TARGET_LINK_LIBRARIES(attentionexperiment
	attention
	${COGSERVER_LIBRARIES}
	${ATOMSPACE_LIBRARIES}
)


ADD_DEPENDENCIES(attentionexperiment attention)
