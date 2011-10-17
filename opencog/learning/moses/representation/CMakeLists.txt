#install header files
INSTALL(FILES

	field_set.h    
	instance_set.h
    build_knobs.h
    knob_mapper.h
    knobs.h
    representation.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/representation" 
)
