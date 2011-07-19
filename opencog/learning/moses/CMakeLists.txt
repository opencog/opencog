ADD_SUBDIRECTORY (moses)
ADD_SUBDIRECTORY (eda)
ADD_SUBDIRECTORY (main)

ADD_LIBRARY (moses SHARED

	moses/build_knobs
	moses/knob_mapper
	moses/knobs
    moses/metapopulation
	moses/moses
    moses/distributed_moses
	moses/representation
    moses/complexity
	moses/scoring
    moses/neighborhood_sampling
	moses/scoring_functions
	moses/types
	moses/using
    moses/pole_balancing
    eda/field_set
    eda/initialization
    eda/local_structure
    main/moses_exec
    main/moses_exec_def
)

TARGET_LINK_LIBRARIES (moses
        comboreduct
        util 
        ${Boost_PROGRAM_OPTIONS_LIBRARY} 
)


#install library
IF (WIN32)
    INSTALL(TARGETS moses DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
    INSTALL(TARGETS moses LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)
