#install header files
INSTALL(FILES

    ant_scoring.h
    build_knobs.h
    knob_mapper.h
    knobs.h
    metapopulation.h
    moses.h
    distributed_moses.h
    optimization.h
    representation.h
    scoring.h
    scoring_functions.h
    types.h
    using.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/moses" 
)

