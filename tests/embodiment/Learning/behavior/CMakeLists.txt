INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/LearningServer
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${XERCESC_INCLUDE_DIR}
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/tests/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PredicateUpdaters
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure/BuiltIn
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
	${OPENCOG_SOURCE_DIR}/opencog/atomspace
	${OPENCOG_SOURCE_DIR}/tests/embodiment/Control/OperationalPetController
	${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/behavior
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/OperationalPetController
	${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
	${OPENCOG_BINARY_DIR}/opencog/util
	${OPENCOG_BINARY_DIR}/opencog/atomspace
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
