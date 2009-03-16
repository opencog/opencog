ADD_LIBRARY(RewritingRules
	hillclimbing_full_reduction
	hillclimbing_perception_reduction
	hillclimbing_action_reduction
	post_learning_rules
	post_learning_rewriting
	RewritingRules
)

INCLUDE_DIRECTORIES(
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES(
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/util
)

TARGET_LINK_LIBRARIES(RewritingRules
	util
	comboreduct
	PetComboVocabulary
)
