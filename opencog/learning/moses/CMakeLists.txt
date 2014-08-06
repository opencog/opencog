# MOSES' version, must be updated manually
SET (MOSES_VERSION_MAJOR 3)
SET (MOSES_VERSION_MINOR 6)
SET (MOSES_VERSION_PATCH 10)

ADD_DEFINITIONS(-DMOSES_VERSION_MAJOR="${MOSES_VERSION_MAJOR}")
ADD_DEFINITIONS(-DMOSES_VERSION_MINOR="${MOSES_VERSION_MINOR}")
ADD_DEFINITIONS(-DMOSES_VERSION_PATCH="${MOSES_VERSION_PATCH}")

# MOSES git describe, automatically updated
EXECUTE_PROCESS(
	COMMAND git describe
	OUTPUT_VARIABLE MOSES_GIT_DESCRIBE
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

IF (MOSES_GIT_DESCRIBE)
  ADD_DEFINITIONS(-DMOSES_GIT_DESCRIBE="${MOSES_GIT_DESCRIBE}")
ENDIF (MOSES_GIT_DESCRIBE)

ADD_SUBDIRECTORY (moses)
ADD_SUBDIRECTORY (scoring)
ADD_SUBDIRECTORY (metapopulation)
ADD_SUBDIRECTORY (deme)
ADD_SUBDIRECTORY (representation)
ADD_SUBDIRECTORY (optimization)
ADD_SUBDIRECTORY (eda)
ADD_SUBDIRECTORY (main)
ADD_SUBDIRECTORY (example-ant)
ADD_SUBDIRECTORY (example-progs)
ADD_SUBDIRECTORY (man)

ADD_LIBRARY (moses SHARED STATIC
	deme/deme_expander
	deme/feature_selector

	eda/initialization
	eda/local_structure

	metapopulation/diversity
	metapopulation/domination
	metapopulation/ensemble
	metapopulation/merging
	metapopulation/metapopulation
	metapopulation/metapop_params
	metapopulation/subsample

	moses/complexity
	moses/distributed_moses
	moses/local_moses
	moses/moses_main
	moses/mpi_moses
	moses/neighborhood_sampling
	moses/partial
	moses/types

	optimization/hill-climbing
	optimization/optimization
	optimization/star-anneal
	optimization/univariate

	representation/build_knobs
	representation/field_set
	representation/knob_mapper
	representation/knobs
	representation/representation

	scoring/behave_cscore
	scoring/bscores
	scoring/discriminating_bscore
	scoring/precision_bscore
	scoring/bscores
	scoring/time_dispersion
	scoring/ss_bscore
	scoring/scoring_base
	scoring/select_bscore

)

TARGET_LINK_LIBRARIES (moses
	comboreduct
	comboant
	feature_selection
	cogutil
	${Boost_PROGRAM_OPTIONS_LIBRARY}
)

IF (HAVE_MPI)
	TARGET_LINK_LIBRARIES (moses
		# These two work for both MPICH2 and OpenMPI, for me.
		${MPI_LIBRARY}
		${MPI_EXTRA_LIBRARY}
	)
ENDIF (HAVE_MPI)


# target_link_libraries(hello ${MPI_LIBRARIES})
# if(MPI_COMPILE_FLAGS)
#   set_target_properties(hello PROPERTIES
#     COMPILE_FLAGS "${MPI_COMPILE_FLAGS}")
# endif()
#
# if(MPI_LINK_FLAGS)
#   set_target_properties(hello PROPERTIES
#     LINK_FLAGS "${MPI_LINK_FLAGS}")
# endif()

# Install library
IF (WIN32)
	INSTALL(TARGETS moses DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
	INSTALL(TARGETS moses
	        LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}"    # lib*.so files
	        ARCHIVE DESTINATION "lib${LIB_DIR_SUFFIX}")   # lib*.a files
ENDIF (WIN32)
