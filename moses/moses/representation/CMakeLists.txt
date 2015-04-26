#install header files
INSTALL(FILES

	field_set.h    
	instance.h
	scored_instance.h
	instance_scorer.h
	instance_set.h
	build_knobs.h
	knob_mapper.h
	knobs.h
	representation.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/representation" 
)
