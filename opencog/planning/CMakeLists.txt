ADD_LIBRARY(planning SHARED
	Action
	ActionSelector
	Utilities
)

TARGET_LINK_LIBRARIES(planning
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS planning DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
