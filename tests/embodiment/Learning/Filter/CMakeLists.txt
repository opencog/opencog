
# At this time, none of these tests compile,
# because of the 2D spacemap ->3D transition.
IF(0)

ADD_CXXTEST(EntropyFilterUTest)
TARGET_LINK_LIBRARIES(EntropyFilterUTest
	AvatarComboVocabulary
	Filter
	comboreduct
	cogutil
	${PROJECT_LIBRARIES}
)

ADD_CXXTEST(ActionFilterUTest)
TARGET_LINK_LIBRARIES(ActionFilterUTest
	behavior
	AvatarComboVocabulary
	Filter
	comboreduct
	cogutil
	${PROJECT_LIBRARIES}
)

ADD_CXXTEST(EntityRelevanceFilterUTest)
TARGET_LINK_LIBRARIES(EntityRelevanceFilterUTest 
	Filter 
	cogutil
	${PROJECT_LIBRARIES}
)
IF(WIN32)
    TARGET_LINK_LIBRARIES(EntityRelevanceFilterUTest winmm)
ENDIF(WIN32)

ENDIF(0)
