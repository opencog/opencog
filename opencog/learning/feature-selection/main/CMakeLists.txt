

ADD_EXECUTABLE (fs-main fs-main.cc)
TARGET_LINK_LIBRARIES(fs-main
	feature_selection
)
SET_TARGET_PROPERTIES(fs-main PROPERTIES OUTPUT_NAME "feature-selection")

ADD_EXECUTABLE (eval-features eval-features.cc)
TARGET_LINK_LIBRARIES(eval-features
	comboreduct
	comboant
	cogutil
	${Boost_PROGRAM_OPTIONS_LIBRARY} 
)

# install feature-selection
INSTALL(TARGETS fs-main eval-features RUNTIME DESTINATION bin)
