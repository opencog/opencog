ADD_LIBRARY(feature_selection SHARED STATIC
	algo/deme_optimize
	algo/incremental
	algo/simple
	algo/stochastic_max_dependency
	scorers/moses_optim
	main/feature-selection
)

TARGET_LINK_LIBRARIES(feature_selection
	util
	moses
	${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

ADD_SUBDIRECTORY(main)
ADD_SUBDIRECTORY(man)
