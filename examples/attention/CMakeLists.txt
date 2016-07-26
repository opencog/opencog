# The atom_types.h file is written to the build directory

# PLNDynamicExpSetupModule
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

IF (TBB_FOUND)

	INCLUDE_DIRECTORIES (
		${TBB_INCLUDE_DIR}
		${CMAKE_BINARY_DIR}
	)

	LINK_DIRECTORIES(
		${TBB_INCLUDE_DIR}
		${TBB_LIBRARY_DIRS}
	)

	ADD_LIBRARY(attentionexperiment SHARED
		ExperimentSetupModule
		SentenceGenStimulateAgent
		Globals
	)

	TARGET_LINK_LIBRARIES(attentionexperiment
		${ATOMSPACE_LIBRARIES}
		attention
		server
		tbb
	)

	ADD_DEPENDENCIES(attentionexperiment mvstubheader)

ENDIF (TBB_FOUND)
