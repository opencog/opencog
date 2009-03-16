INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/hillclimbing
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/FitnessEstimator 
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/Filter 
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/ImitationLearningAPI
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/RewritingRules
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog/learning
    ${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/hillclimbing
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/FitnessEstimator
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/RewritingRules
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/Filter
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/learning/moses
)

IF(COMMAND cmake_policy)
    CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

ADD_EXECUTABLE (interactive-hillclimbing interactive-hillclimbing)
TARGET_LINK_LIBRARIES (interactive-hillclimbing
	hillclimbing
	util
	comboreduct
)

ADD_EXECUTABLE (ant-hillclimbing ant-hillclimbing)
TARGET_LINK_LIBRARIES (ant-hillclimbing
	hillclimbing
	util
	comboreduct
)

ADD_EXECUTABLE (moses-petbrain moses-petbrain)
TARGET_LINK_LIBRARIES (moses-petbrain
	${MOSES_LIBRARY}
	hillclimbing
	util
	comboreduct
)

ADD_EXECUTABLE (post_learning-rewriter post_learning-rewriter)
TARGET_LINK_LIBRARIES (post_learning-rewriter
	RewritingRules
)
