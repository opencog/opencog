# MOSES' version, must be updated manually
SET (MOSES_VERSION_MAJOR 3)
SET (MOSES_VERSION_MINOR 1)
SET (MOSES_VERSION_PATCH 0)

ADD_DEFINITIONS(-DMOSES_VERSION_MAJOR="${MOSES_VERSION_MAJOR}")
ADD_DEFINITIONS(-DMOSES_VERSION_MINOR="${MOSES_VERSION_MINOR}")
ADD_DEFINITIONS(-DMOSES_VERSION_PATCH="${MOSES_VERSION_PATCH}")

# MOSES bzr revision number, automatically updated
EXECUTE_PROCESS(COMMAND bzr revno "${PROJECT_SOURCE_DIR}" OUTPUT_VARIABLE MOSES_BZR_REVNO)
ADD_DEFINITIONS(-DMOSES_BZR_REVNO="${MOSES_BZR_REVNO}")

ADD_SUBDIRECTORY (moses)
ADD_SUBDIRECTORY (representation)
ADD_SUBDIRECTORY (optimization)
ADD_SUBDIRECTORY (eda)
ADD_SUBDIRECTORY (main)
ADD_SUBDIRECTORY (example-ant)
ADD_SUBDIRECTORY (example-progs)
ADD_SUBDIRECTORY (man)

ADD_LIBRARY (moses SHARED
	representation/build_knobs
	representation/knob_mapper
	representation/knobs
	representation/representation
	representation/field_set
	moses/metapopulation
	moses/moses
	moses/distributed_moses
	moses/complexity
	moses/scoring
	moses/neighborhood_sampling
	moses/scoring_functions
	moses/types
	moses/using
	moses/pole_balancing
	eda/initialization
	eda/local_structure
)

TARGET_LINK_LIBRARIES (moses
	comboreduct
	util 
	${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

# Install library
IF (WIN32)
	INSTALL(TARGETS moses DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
	INSTALL(TARGETS moses LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)
