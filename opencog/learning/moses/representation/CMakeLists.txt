#install header files
INSTALL(FILES
    build_knobs.h
    knob_mapper.h
    knobs.h
    representation.h
    scoring.h
    scoring_functions.h
    types.h
    using.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/representation" 
)
