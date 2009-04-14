ADD_LIBRARY(PetaverseHC
	petaverse-hillclimber
        interactive-hillclimber
	HCPetaverseVocabularyProvider
)

TARGET_LINK_LIBRARIES(PetaverseHC
        hillclimbing
	RewritingRules
	comboreduct
	util
	FitnessEstimator
	PetComboVocabulary
)
