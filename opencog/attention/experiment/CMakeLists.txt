# The atom_types.h file is written to the build directory

# PLNDynamicExpSetupModule
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

IF (TBB_FOUND)
    
    ADD_SUBDIRECTORY(tv-toolbox)
    
	INCLUDE_DIRECTORIES (
		${TBB_INCLUDE_DIR}
		${GHC_LIB_DIR}/include
		${CMAKE_BINARY_DIR}
	)

	LINK_DIRECTORIES(
		${TBB_INCLUDE_DIR}
		${TBB_LIBRARY_DIRS}
		${CMAKE_CURRENT_BINARY_DIR}/tv-toolbox
	)

	ADD_LIBRARY(dynamicsexpsetupmodule SHARED
		ExperimentSetupModule
		SentenceGenStimulateAgent	
		ArtificialStimulatorAgent
		SmokesDBFCAgent
	)
	
	TARGET_LINK_LIBRARIES(dynamicsexpsetupmodule
		${ATOMSPACE_LIBRARIES}
		attention
		server
		tbb
		jsd
	)

	ADD_DEPENDENCIES(dynamicsexpsetupmodule jsdistancehaskellinterface)
	
ENDIF (TBB_FOUND)