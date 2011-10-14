ADD_LIBRARY(Filter
	EntropyFilter
	ActionFilter
	EntityRelevanceFilter
)

TARGET_LINK_LIBRARIES(Filter
	PAI
	util
	comboreduct
	AvatarComboVocabulary
	behavior
	RewritingRules
	WorldWrapper
	AtomSpaceExtensions
)
