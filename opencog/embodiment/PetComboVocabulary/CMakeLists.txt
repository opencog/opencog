ADD_LIBRARY(PetComboVocabulary
	pet_builtin_action
	pet_perception
	pet_action_symbol
	pet_indefinite_object
	PetComboVocabulary
)

TARGET_LINK_LIBRARIES(PetComboVocabulary
	comboreduct
	util
)
