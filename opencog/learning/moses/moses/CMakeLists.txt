#install header files
INSTALL(FILES
	distributed_moses.h
	local_moses.h
	moses_params.h
	mpi_moses.h
	partial.h
	types.h
	using.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/moses"
)
