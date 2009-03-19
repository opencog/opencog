INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/Learning/behavior
	${OPENCOG_SOURCE_DIR}/src/PetComboVocabulary
	${OPENCOG_SOURCE_DIR}/src/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/src/Spatial
        ${OPENCOG_SOURCE_DIR}/src/Spatial/Math
	${OPENCOG_SOURCE_DIR}/src/Learning
	${OPENCOG_SOURCE_DIR}/src/Learning/LearningServer
	${OPENCOG_SOURCE_DIR}/src/Control/OperationalPetController
	${OPENCOG_SOURCE_DIR}/src/Control
	${XERCESC_INCLUDE_DIR}
	${OPENCOG_SOURCE_DIR}/src/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/test/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/src/Control/MessagingSystem
	${OPENCOG_SOURCE_DIR}/src/Control/PredicateUpdaters
	${OPENCOG_SOURCE_DIR}/src/Control/Predavese
	${OPENCOG_SOURCE_DIR}/src/Control/Procedure
	${OPENCOG_SOURCE_DIR}/src/Control/Procedure/BuiltIn
	${OPENCOG_SOURCE_DIR}/src/WorldWrapper
	${OPENCOG_SOURCE_DIR}/test/Control/OperationalPetController
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/src/Learning/behavior
	${OPENCOG_BINARY_DIR}/src/Control/OperationalPetController
	${OPENCOG_BINARY_DIR}/src/PetComboVocabulary
	${OPENCOG_BINARY_DIR}/src/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/src/WorldWrapper
)

ADD_CXXTEST(BehaviorDescriptionMatcherUTest)
TARGET_LINK_LIBRARIES(BehaviorDescriptionMatcherUTest
   behavior
   util
   ${OPENCOG_LIBRARIES}
)


ADD_CXXTEST(CompositeBehaviorDescriptionUTest)
TARGET_LINK_LIBRARIES(CompositeBehaviorDescriptionUTest
	behavior
        util
	${OPENCOG_LIBRARIES}
)

ADD_CXXTEST(BehaviorCategoryUTest)
TARGET_LINK_LIBRARIES(BehaviorCategoryUTest
	behavior
        util
	${OPENCOG_LIBRARIES}
)

ADD_CXXTEST(BDRetrieverUTest)
TARGET_LINK_LIBRARIES(BDRetrieverUTest
	behavior
	PetComboVocabulary
	${XERCESC_LIBRARIES}
	comboreduct
	${OPENCOG_LIBRARIES}	
)

ADD_CXXTEST(BEUTest)
TARGET_LINK_LIBRARIES(BEUTest
        behavior
	${OPENCOG_LIBRARIES}	
)        
