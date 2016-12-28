INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY(attentionexperiment SHARED
	ExperimentSetupModule
	SentenceGenStimulateAgent
)

TARGET_LINK_LIBRARIES(attentionexperiment
	${ATOMSPACE_LIBRARIES}
	attention
	server
)


ADD_DEPENDENCIES(attentionexperiment attention)
ADD_DEPENDENCIES(attentionexperiment server)
ADD_DEPENDENCIES(attentionexperiment cogserver)
