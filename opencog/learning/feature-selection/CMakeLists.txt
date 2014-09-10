ADD_LIBRARY(feature_selection SHARED STATIC
	algo/deme_optimize
	algo/incremental
	algo/random
	algo/simple
	algo/stochastic_max_dependency
	scorers/moses_optim
	main/feature-selection
)

TARGET_LINK_LIBRARIES(feature_selection
	moses
	cogutil
	${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

ADD_SUBDIRECTORY(main)
ADD_SUBDIRECTORY(man)
