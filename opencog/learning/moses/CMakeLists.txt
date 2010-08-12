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
)

TARGET_LINK_LIBRARIES (moses
        comboreduct
        util 
)


#install library

#INSTALL(TARGETS 
#	${PROJECT_NAME}
#	DESTINATION "lib/${PROJECT_NAME}" PERMISSIONS
#	OWNER_READ OWNER_WRITE OWNER_EXECUTE
#	GROUP_READ GROUP_EXECUTE
#	WORLD_READ WORLD_EXECUTE
#)

