#install header files
INSTALL(FILES
    distributed_moses.h
    metapopulation.h
    moses.h
    partial.h
    scoring.h
    # scoring_functions.h -- dead code, remove this entirely someday.
    types.h
    using.h

	DESTINATION

	"include/${PROJECT_NAME}/learning/moses/moses" 
)
