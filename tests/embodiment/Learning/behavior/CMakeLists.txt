INCLUDE_DIRECTORIES (
	${XERCESC_INCLUDE_DIR}
)
LINK_LIBRARIES (
	comboreduct
	behavior
        util
	${PROJECT_LIBRARIES}	
)

ADD_CXXTEST(BehaviorDescriptionMatcherUTest)
ADD_CXXTEST(CompositeBehaviorDescriptionUTest)
ADD_CXXTEST(BehaviorCategoryUTest)
ADD_CXXTEST(BDRetrieverUTest)

LINK_LIBRARIES(
	AvatarComboVocabulary
	${XERCESC_LIBRARIES}
	comboreduct
)

ADD_CXXTEST(BEUTest)
