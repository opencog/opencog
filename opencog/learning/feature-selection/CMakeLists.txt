ADD_LIBRARY(feature_selection SHARED STATIC
	moses_based_scorer
	algo/deme_optimize
	main/feature-selection
)

TARGET_LINK_LIBRARIES(feature_selection
	util
	moses
	${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

ADD_SUBDIRECTORY(main)
ADD_SUBDIRECTORY(man)
