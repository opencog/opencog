# The atom_types.h file is written to the build directory

# PLNDynamicExpSetupModule
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

IF (TBB_FOUND)

    ADD_SUBDIRECTORY(tv-toolbox)

	INCLUDE_DIRECTORIES (
		${TBB_INCLUDE_DIR}
		${GHC_LIB_DIR}/include
        #hardcoded ghc lib dir for docker
        /home/opencog/.stack/programs/x86_64-linux/ghc-7.10.3/lib/ghc-7.10.3/include/
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
		Globals
	)

	TARGET_LINK_LIBRARIES(dynamicsexpsetupmodule
		${ATOMSPACE_LIBRARIES}
		attention
		server
		tbb
        #jsd
        HSTVToolBox-0.1.0.0-853kwxHqBcJGNhdPNUMaQK-ghc7.10.3
	)

	ADD_DEPENDENCIES(dynamicsexpsetupmodule mvstubheader)

ENDIF (TBB_FOUND)
