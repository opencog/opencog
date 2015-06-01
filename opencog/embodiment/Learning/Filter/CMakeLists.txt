ADD_LIBRARY(Filter
	EntropyFilter
	ActionFilter
	EntityRelevanceFilter
)

TARGET_LINK_LIBRARIES(Filter
	PAI
	${COGUTIL_LIBRARY}
	comboreduct
	AvatarComboVocabulary
	behavior
	RewritingRules
	WorldWrapper
	AtomSpaceExtensions
)
