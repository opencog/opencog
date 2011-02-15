#install header files
INSTALL(FILES

	eda.h
	field_set.h
	initialization.h
	instance_set.h
	local_structure.h
	logging.h
	optimize.h
	replacement.h
	scoring_functions.h
	scoring.h
	scratch_model.h
	termination.h
	using.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/eda" 
)

