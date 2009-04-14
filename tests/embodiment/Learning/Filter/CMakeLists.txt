ADD_CXXTEST(EntropyFilterUTest)
TARGET_LINK_LIBRARIES(EntropyFilterUTest
	PetComboVocabulary
	Filter
        comboreduct
        util
	${OPENCOG_LIBRARIES}
)

ADD_CXXTEST(ActionFilterUTest)
TARGET_LINK_LIBRARIES(ActionFilterUTest
	behavior
	PetComboVocabulary
	Filter
        comboreduct
        util
	${OPENCOG_LIBRARIES}
)

ADD_CXXTEST(EntityRelevanceFilterUTest)
TARGET_LINK_LIBRARIES(EntityRelevanceFilterUTest 
	Filter 
        util
	${OPENCOG_LIBRARIES}
)
IF(WIN32)
    TARGET_LINK_LIBRARIES(EntityRelevanceFilterUTest winmm)
ENDIF(WIN32)
