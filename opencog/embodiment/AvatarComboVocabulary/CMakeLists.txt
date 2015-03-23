ADD_LIBRARY(AvatarComboVocabulary
	avatar_builtin_action
	avatar_perception
	avatar_action_symbol
	avatar_indefinite_object
	AvatarComboVocabulary
)

TARGET_LINK_LIBRARIES(AvatarComboVocabulary
	comboreduct
	comboant
	util
)
