ADD_LIBRARY(PetComboVocabulary
	pet_builtin_action
	pet_perception
	pet_action_symbol
	pet_indefinite_object
	PetComboVocabulary
)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/util
)

TARGET_LINK_LIBRARIES(PetComboVocabulary
	comboreduct
	util
)
