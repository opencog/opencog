# The atom_types.h file is written to the build directory

# PLNDynamicExpSetupModule
IF (TBB_FOUND)
    INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})
    
	INCLUDE_DIRECTORIES (
		${TBB_INCLUDE_DIR}
	)

	LINK_DIRECTORIES(
		${TBB_INCLUDE_DIR}
		${TBB_LIBRARY_DIRS}
	)

	ADD_LIBRARY(plndynamicsexpsetupmodule SHARED
		PLNDynamicsExpSetupModule
		SentenceGenStimulateAgent	
		ArtificialStimulatorAgent	
	)

	TARGET_LINK_LIBRARIES(plndynamicsexpsetupmodule
		${ATOMSPACE_LIBRARIES}
		server
		tbb
	)
ENDIF (TBB_FOUND)