#install header files
INSTALL(FILES
	distributed_moses.h
	local_moses.h
	metapopulation.h
	moses_params.h
	mpi_moses.h
	partial.h
	scoring.h
	types.h
	using.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/moses"
)
