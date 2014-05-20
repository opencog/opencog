
INCLUDE_DIRECTORIES (
	${PYTHON_INCLUDE_PATH}
)

LINK_LIBRARIES (
	Control
	behavior
	comboreduct
	comboant
	cogutil
	${PROJECT_LIBRARIES}	
)

ADD_CXXTEST(BehaviorDescriptionMatcherUTest)
ADD_CXXTEST(CompositeBehaviorDescriptionUTest)
ADD_CXXTEST(BehaviorCategoryUTest)
ADD_CXXTEST(BDRetrieverUTest)

LINK_LIBRARIES(
	oac
	lslib
	Control
	AvatarComboVocabulary
	comboreduct
)

#ADD_CXXTEST(BEUTest)
