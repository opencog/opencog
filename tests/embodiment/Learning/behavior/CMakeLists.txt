
LINK_LIBRARIES (
	Control
	behavior
	comboreduct
	util
	${PROJECT_LIBRARIES}	
)

ADD_CXXTEST(BehaviorDescriptionMatcherUTest)
ADD_CXXTEST(CompositeBehaviorDescriptionUTest)
ADD_CXXTEST(BehaviorCategoryUTest)
ADD_CXXTEST(BDRetrieverUTest)

LINK_LIBRARIES(
	Control
	AvatarComboVocabulary
	comboreduct
)

ADD_CXXTEST(BEUTest)
